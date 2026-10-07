#!/usr/bin/env python3
"""Visualize filtered (ranked) CGN grasps on the scene point cloud.

Standalone — not used by run_phase2.sh. Host system Python with Open3D:

  /usr/bin/python3 scripts/visualize_ranked_grasps.py \\
      --ranked output/grasps/scene_XXXX_ranked.json

Optional:
  --scene PATH          (default: source_scene from the JSON)
  --top-k 5             grasps per object (default: all kept in JSON)
  --object-id 4 5 7     only these object ids
  --max-points 200000   downsample cloud
  --no-best-highlight   same thickness for all grasps
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

try:
    import open3d as o3d
except ImportError as e:
    raise SystemExit(
        "Need Open3D. Install with:\n"
        "  /usr/bin/python3 -m pip install --user open3d\n"
        f"({e})"
    ) from e

def _gripper_local_points(width_m: float, finger_depth: float = 0.06) -> tuple[np.ndarray, list[tuple[int, int]]]:
    """Parallel-jaw wireframe in CGN grasp frame (X = open, Z = approach)."""
    hw = 0.5 * float(np.clip(width_m, 0.005, 0.12))
    # 0: wrist/TCP, 1: mid base, 2/4: finger bases, 3/5: finger tips
    pts = np.array(
        [
            [0.0, 0.0, -0.02],
            [0.0, 0.0, 0.0],
            [-hw, 0.0, 0.0],
            [-hw, 0.0, finger_depth],
            [hw, 0.0, 0.0],
            [hw, 0.0, finger_depth],
        ],
        dtype=np.float64,
    )
    edges = [(0, 1), (1, 2), (2, 3), (1, 4), (4, 5), (2, 4)]
    return pts, edges


def depth_to_cloud(
    depth: np.ndarray,
    K: np.ndarray,
    rgb: np.ndarray | None,
    z_min: float,
    z_max: float,
    max_points: int,
) -> o3d.geometry.PointCloud:
    h, w = depth.shape[:2]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u, v = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    z = depth.astype(np.float64)
    valid = np.isfinite(z) & (z > z_min) & (z < z_max)
    u, v, z = u[valid], v[valid], z[valid]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    pts = np.stack([x, y, z], axis=1)

    colors = None
    if rgb is not None:
        rgb_f = rgb.reshape(-1, 3)[valid.reshape(-1)]
        colors = rgb_f.astype(np.float64) / 255.0

    if len(pts) > max_points:
        idx = np.random.choice(len(pts), max_points, replace=False)
        pts = pts[idx]
        if colors is not None:
            colors = colors[idx]

    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pts)
    if colors is not None:
        pc.colors = o3d.utility.Vector3dVector(colors)
    return pc


def gripper_line_set(T: np.ndarray, width: float, color: np.ndarray) -> o3d.geometry.LineSet:
    """Build a parallel-jaw wireframe at pose T (4x4, camera optical frame)."""
    w = float(width) if width is not None and np.isfinite(width) else 0.04
    local, edges = _gripper_local_points(w)
    pts = (local @ T[:3, :3].T) + T[:3, 3]

    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(pts)
    ls.lines = o3d.utility.Vector2iVector(np.asarray(edges, dtype=np.int32))
    col = np.tile(np.asarray(color, dtype=np.float64).reshape(1, 3), (len(edges), 1))
    ls.colors = o3d.utility.Vector3dVector(col)
    return ls


def approach_arrow(T: np.ndarray, color: np.ndarray, length: float = 0.06) -> o3d.geometry.LineSet:
    o = T[:3, 3]
    tip = o + T[:3, 2] * length
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(np.stack([o, tip], axis=0))
    ls.lines = o3d.utility.Vector2iVector([[0, 1]])
    ls.colors = o3d.utility.Vector3dVector([color])
    return ls


def contact_sphere(xyz: np.ndarray, color: np.ndarray, radius: float = 0.006) -> o3d.geometry.TriangleMesh:
    s = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    s.compute_vertex_normals()
    s.paint_uniform_color(color)
    s.translate(xyz)
    return s


def object_color(i: int) -> np.ndarray:
    # Distinct hues (Open3D RGB 0..1)
    table = [
        [0.95, 0.45, 0.10],
        [0.10, 0.75, 0.95],
        [0.35, 0.90, 0.35],
        [0.90, 0.25, 0.75],
        [0.95, 0.90, 0.20],
        [0.55, 0.40, 0.95],
        [0.20, 0.95, 0.70],
        [0.95, 0.35, 0.35],
    ]
    return np.asarray(table[i % len(table)], dtype=np.float64)


def main():
    ap = argparse.ArgumentParser(description="Visualize ranked CGN grasps on scene cloud")
    ap.add_argument("--ranked", required=True, help="*_ranked.json from rank_cgn_grasps.py")
    ap.add_argument("--scene", default="", help="scene_*.npz (default: source_scene in JSON)")
    ap.add_argument("--top-k", type=int, default=0, help="max grasps per object (0 = all in JSON)")
    ap.add_argument("--object-id", type=int, nargs="*", default=[], help="only these object ids")
    ap.add_argument("--max-points", type=int, default=200000)
    ap.add_argument("--z-min", type=float, default=0.25)
    ap.add_argument("--z-max", type=float, default=1.8)
    ap.add_argument("--no-best-highlight", action="store_true")
    ap.add_argument("--show-approach", action="store_true", help="draw approach axis for each grasp")
    args = ap.parse_args()

    ranked_path = Path(args.ranked).resolve()
    payload = json.loads(ranked_path.read_text())
    scene_path = Path(args.scene) if args.scene else Path(payload.get("source_scene") or "")
    if not scene_path.is_file():
        raise SystemExit(f"scene npz not found: {scene_path} (pass --scene)")

    s = np.load(scene_path, allow_pickle=True)
    depth = np.asarray(s["depth"], dtype=np.float32)
    K = np.asarray(s["K"], dtype=np.float64).reshape(3, 3)
    rgb = np.asarray(s["rgb"]) if "rgb" in s.files else None

    geoms: list = []
    cloud = depth_to_cloud(depth, K, rgb, args.z_min, args.z_max, args.max_points)
    geoms.append(cloud)

    # Camera frame at origin
    geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.08))

    want = set(args.object_id) if args.object_id else None
    n_drawn = 0
    for oi, obj in enumerate(payload.get("objects", [])):
        oid = int(obj["object_id"])
        if want is not None and oid not in want:
            continue
        grasps = list(obj.get("grasps") or [])
        if args.top_k > 0:
            grasps = grasps[: args.top_k]
        if not grasps:
            continue
        base_col = object_color(oi)
        print(f"object {oid}: drawing {len(grasps)} grasp(s)  best_score={grasps[0]['score']:.3f}")

        for gi, g in enumerate(grasps):
            T = np.asarray(g["T_cam"], dtype=np.float64)
            w = g.get("width_m")
            if w is None:
                w = 0.04
            is_best = gi == 0 and not args.no_best_highlight
            col = np.array([0.05, 0.95, 0.15]) if is_best else base_col
            geoms.append(gripper_line_set(T, float(w), col))
            if args.show_approach:
                geoms.append(approach_arrow(T, col))
            c = np.asarray(g["contact_cam"], dtype=np.float64)
            geoms.append(contact_sphere(c, col, radius=0.008 if is_best else 0.005))
            n_drawn += 1

    if n_drawn == 0:
        raise SystemExit("no grasps to draw (empty JSON or filtered by --object-id)")

    print(f"scene: {scene_path}")
    print(f"ranked: {ranked_path}")
    print(f"drawn {n_drawn} grasps — close the window to exit")
    o3d.visualization.draw_geometries(
        geoms,
        window_name=f"Ranked grasps — {ranked_path.name}",
        width=1280,
        height=800,
        point_show_normal=False,
    )


if __name__ == "__main__":
    main()
