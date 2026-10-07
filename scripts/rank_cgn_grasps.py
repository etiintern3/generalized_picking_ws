#!/usr/bin/env python3
"""Rank Contact-GraspNet scene predictions for the 2F-85.

Reads CGN results/predictions_*.npz (dicts keyed by object id) and optionally
the matching scene_*.npz (rgb, depth, K, segmap).

Writes:
  output/grasps/<stem>_ranked.json
  output/grasps/<stem>_ranked.npz
  output/grasps/<stem>_preview.png  (if --scene given)

Poses are 4x4 in the camera optical frame (OpenCV: x right, y down, z forward).
No MoveIt / base_link TF here (Phase 3).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

MAX_WIDTH_M = 0.085
DEFAULT_MIN_SCORE = 0.10
DEFAULT_TOP_K = 10


def _item(arr):
    if isinstance(arr, np.ndarray) and arr.dtype == object:
        return arr.item()
    return arr


def _as_dict(obj) -> dict:
    obj = _item(obj)
    if not isinstance(obj, dict):
        raise TypeError(f'expected dict of per-object arrays, got {type(obj)}')
    return {int(k): np.asarray(v) for k, v in obj.items()}


def fit_table_plane(xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray] | None:
    """Return (point_on_plane, unit normal pointing roughly toward camera)."""
    if len(xyz) < 50:
        return None
    c = xyz.mean(axis=0)
    _, _, vh = np.linalg.svd(xyz - c, full_matrices=False)
    n = vh[-1]
    n = n / (np.linalg.norm(n) + 1e-12)
    # Camera looks along +Z; table normal should face the camera (n·(-Z) > 0).
    if n[2] > 0:
        n = -n
    return c, n


def backproject(depth: np.ndarray, K: np.ndarray, z_min: float, z_max: float) -> np.ndarray:
    h, w = depth.shape[:2]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    z = depth.astype(np.float32)
    valid = np.isfinite(z) & (z > z_min) & (z < z_max)
    z = z[valid]
    x = (u[valid] - cx) * z / fx
    y = (v[valid] - cy) * z / fy
    return np.stack([x, y, z], axis=1)


def rank_object(
    poses: np.ndarray,
    scores: np.ndarray,
    contacts: np.ndarray,
    widths: np.ndarray | None,
    max_width: float,
    min_score: float,
    table: tuple[np.ndarray, np.ndarray] | None,
    min_height: float,
    max_into_table: float,
) -> tuple[list[dict], dict[str, int]]:
    """Return (kept grasps, reject counts for tuning)."""
    reject = {
        'bad_pose': 0,
        'low_score': 0,
        'wide': 0,
        'bad_contact': 0,
        'below_table': 0,
        'into_table': 0,
    }
    poses = np.asarray(poses, dtype=np.float64)
    if poses.ndim == 2:
        poses = poses[None, ...]
    if poses.ndim != 3 or poses.shape[-2:] != (4, 4):
        return [], reject
    n = poses.shape[0]
    scores = np.asarray(scores, dtype=np.float64).reshape(-1)
    contacts = np.asarray(contacts, dtype=np.float64).reshape(-1, 3)
    if widths is None:
        widths = np.full(n, np.nan)
    else:
        widths = np.asarray(widths, dtype=np.float64).reshape(-1)

    kept = []
    for i in range(n):
        T = poses[i]
        if T.shape != (4, 4) or not np.isfinite(T).all():
            reject['bad_pose'] += 1
            continue
        sc = float(scores[i]) if i < len(scores) and np.isfinite(scores[i]) else 0.0
        if sc < min_score:
            reject['low_score'] += 1
            continue
        w = float(widths[i]) if i < len(widths) and np.isfinite(widths[i]) else None
        if w is not None and w > max_width + 1e-6:
            reject['wide'] += 1
            continue
        c = contacts[i] if i < len(contacts) else T[:3, 3]
        if not np.isfinite(c).all():
            reject['bad_contact'] += 1
            continue
        approach = T[:3, 2].astype(np.float64)
        approach = approach / (np.linalg.norm(approach) + 1e-12)

        if table is not None:
            p0, nrm = table
            height = float(np.dot(c - p0, nrm))
            # Contact should sit on the camera side of the table, not through it.
            if height < min_height:
                reject['below_table'] += 1
                continue
            # Optional: reject approaches that point strongly into the table.
            # Disabled when max_into_table >= 1.0 — front-camera top grasps have
            # approach ≈ down, which looks "into table" even when contacts are valid.
            if max_into_table < 1.0:
                into = float(np.dot(approach, -nrm))
                if into > max_into_table:
                    reject['into_table'] += 1
                    continue

        kept.append(
            {
                'score': sc,
                'width_m': None if w is None else round(w, 5),
                'contact_cam': [round(float(x), 5) for x in c],
                'approach_cam': [round(float(x), 5) for x in approach],
                'T_cam': np.round(T, 6).tolist(),
            }
        )
    kept.sort(key=lambda g: -g['score'])
    return kept, reject


def preview_png(scene: dict, objects: list[dict], out_png: Path) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    rgb = scene['rgb']
    K = scene['K']
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    ax.imshow(rgb)
    ax.set_title('Top ranked grasps (contact · size ∝ score)')
    colors = plt.cm.tab10.colors
    for oi, obj in enumerate(objects):
        col = colors[oi % 10]
        for gi, g in enumerate(obj['grasps'][:5]):
            x, y, z = g['contact_cam']
            if z <= 1e-6:
                continue
            u = fx * x / z + cx
            v = fy * y / z + cy
            ms = 8 + 12 * min(g['score'] / max(obj['grasps'][0]['score'], 1e-6), 1.0)
            ax.plot(u, v, 'o', color=col, markersize=ms, markeredgecolor='white', markeredgewidth=0.4)
            if gi == 0:
                ax.text(u + 6, v, f"obj {obj['object_id']}", color=col, fontsize=8)
    ax.axis('off')
    fig.tight_layout()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description='Rank CGN grasps for Robotiq 2F-85')
    ap.add_argument('--pred', required=True, help='CGN results/predictions_*.npz')
    ap.add_argument('--scene', default='', help='optional scene_*.npz (rgb, depth, K, segmap)')
    ap.add_argument('--out-dir', default='', help='default: <ws>/output/grasps')
    ap.add_argument('--max-width', type=float, default=MAX_WIDTH_M)
    ap.add_argument('--min-score', type=float, default=DEFAULT_MIN_SCORE)
    ap.add_argument('--top-k', type=int, default=DEFAULT_TOP_K)
    ap.add_argument('--min-height', type=float, default=0.008, help='min contact height above table (m)')
    ap.add_argument(
        '--max-into-table',
        type=float,
        default=1.0,
        help='max approach·(-table_normal); >=1.0 disables (default: off — front-cam top grasps look "into" table)',
    )
    ap.add_argument(
        '--enable-into-table',
        action='store_true',
        help='use legacy into-table filter (sets max-into-table=0.35 unless you also pass --max-into-table)',
    )
    args = ap.parse_args()
    if args.enable_into_table and args.max_into_table >= 1.0:
        args.max_into_table = 0.35

    pred_path = Path(args.pred).resolve()
    data = np.load(pred_path, allow_pickle=True)
    grasps = _as_dict(data['pred_grasps_cam'])
    scores = _as_dict(data['scores'])
    contacts = _as_dict(data['contact_pts'])
    widths = None
    if 'gripper_openings' in data.files:
        widths = _as_dict(data['gripper_openings'])

    table = None
    scene = None
    if args.scene:
        s = np.load(args.scene, allow_pickle=True)
        scene = {k: s[k] for k in s.files}
        scene['K'] = np.asarray(scene['K'], dtype=np.float64).reshape(3, 3)
        depth = np.asarray(scene['depth'], dtype=np.float32)
        seg = scene.get('segmap', scene.get('seg'))
        xyz = backproject(depth, scene['K'], 0.35, 1.6)
        if seg is not None:
            # Prefer non-object pixels for the table fit when we can.
            h, w = depth.shape[:2]
            fx, fy, cx, cy = scene['K'][0, 0], scene['K'][1, 1], scene['K'][0, 2], scene['K'][1, 2]
            u, v = np.meshgrid(np.arange(w), np.arange(h))
            z = depth
            valid = np.isfinite(z) & (z > 0.35) & (z < 1.6) & (np.asarray(seg) == 0)
            if int(valid.sum()) > 200:
                zz = z[valid]
                xyz = np.stack([(u[valid] - cx) * zz / fx, (v[valid] - cy) * zz / fy, zz], axis=1)
        table = fit_table_plane(xyz)

    objects = []
    reject_by_obj = {}
    for oid in sorted(grasps.keys()):
        w = None if widths is None else widths.get(oid)
        ranked, reject = rank_object(
            grasps[oid],
            scores.get(oid, np.array([])),
            contacts.get(oid, np.zeros((0, 3))),
            w,
            max_width=args.max_width,
            min_score=args.min_score,
            table=table,
            min_height=args.min_height,
            max_into_table=args.max_into_table,
        )
        reject_by_obj[oid] = reject
        ranked = ranked[: args.top_k]
        objects.append(
            {
                'object_id': oid,
                'n_raw': int(np.asarray(grasps[oid]).reshape(-1, 4, 4).shape[0]),
                'n_kept': len(ranked),
                'reject': reject,
                'grasps': ranked,
            }
        )

    ws = Path('/home/satwik/generalized_picking_ws')
    out_dir = Path(args.out_dir) if args.out_dir else ws / 'output' / 'grasps'
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = pred_path.stem.replace('predictions_', '')
    payload = {
        'source_pred': str(pred_path),
        'source_scene': args.scene or None,
        'frame': 'camera_optical (OpenCV: x right, y down, z forward)',
        'max_width_m': args.max_width,
        'min_score': args.min_score,
        'min_height_m': args.min_height,
        'max_into_table': args.max_into_table,
        'table_filter': table is not None,
        'objects': objects,
    }
    json_path = out_dir / f'{stem}_ranked.json'
    json_path.write_text(json.dumps(payload, indent=2))

    np.savez(
        out_dir / f'{stem}_ranked.npz',
        summary=np.array(payload, dtype=object),
    )

    print(f'wrote {json_path}')
    print(
        f'filters: min_score={args.min_score} max_width={args.max_width} '
        f'min_height={args.min_height} max_into_table={args.max_into_table} '
        f'table_filter={table is not None}'
    )
    for obj in objects:
        best = obj['grasps'][0]['score'] if obj['grasps'] else None
        print(
            f"  object {obj['object_id']}: raw={obj['n_raw']} kept={obj['n_kept']}"
            + (f" best_score={best:.3f}" if best is not None else ' (none)')
        )
        r = obj['reject']
        dropped = {k: v for k, v in r.items() if v}
        if dropped and obj['n_kept'] < obj['n_raw']:
            print(f"    rejected: {dropped}")

    if scene is not None and 'rgb' in scene:
        png = out_dir / f'{stem}_preview.png'
        preview_png(scene, objects, png)
        print(f'wrote {png}')

    if widths is None:
        print(
            'note: predictions file has no gripper_openings; re-run CGN inference.py '
            '(updated save) to store widths, or rely on DATA.gripper_width:0.085 at generate time.'
        )


if __name__ == '__main__':
    main()
