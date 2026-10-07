#!/usr/bin/env python3
"""Front-camera SAM masks → PLY: area + IoU NMS + planarity filter (drop table planes).

Unlike masks_to_pointclouds_nms.py (avg-Z tray floor), this keeps masks whose
3D points are NOT well fit by a single plane (objects) and drops near-planar
masks (table patches). Does not modify the NMS script.
"""
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from isaac_ros_tensor_list_interfaces.msg import TensorList
from cv_bridge import CvBridge

# Front-camera tabletop crop (same as frontcam_table_grid_prompts.py)
X_MIN, X_MAX = 200, 1050
Y_MIN, Y_MAX = 500, 720

MIN_AREA = 3000
MAX_AREA = 180000
IOU_MERGE = 0.9

MIN_DEPTH = 0.05
MAX_DEPTH = 5.0
MIN_POINTS = 500

# Plane-fit RMS residual (meters). Below → treat as table plane and drop.
# Tune from logged plane_RMS values: table patches are low; object masks higher.
PLANAR_RMS_MAX = 0.004

OUT_DIR = Path('/workspaces/isaac_ros-dev/output/clouds_planar')
SAVE_EVERY_N = 30
MAX_POINTS_SAVE = 20000


def write_ply(path: Path, xyz: np.ndarray):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w') as f:
        f.write('ply\nformat ascii 1.0\n')
        f.write(f'element vertex {len(xyz)}\n')
        f.write('property float x\nproperty float y\nproperty float z\n')
        f.write('end_header\n')
        for x, y, z in xyz:
            f.write(f'{x:.6f} {y:.6f} {z:.6f}\n')


def mask_iou(a: np.ndarray, b: np.ndarray) -> float:
    inter = np.logical_and(a, b).sum()
    union = np.logical_or(a, b).sum()
    return float(inter) / float(union) if union else 0.0


def nms_masks(candidates, iou_thresh: float):
    candidates = sorted(candidates, key=lambda x: x[0])
    kept = []
    for area, m in candidates:
        if any(mask_iou(m, km) >= iou_thresh for _, km in kept):
            continue
        kept.append((area, m))
    return kept


def plane_rms(xyz: np.ndarray) -> float:
    """RMS distance of points to best-fit plane (SVD)."""
    if len(xyz) < 3:
        return 0.0
    c = xyz.mean(axis=0)
    _, s, vh = np.linalg.svd(xyz - c, full_matrices=False)
    normal = vh[-1]
    nrm = np.linalg.norm(normal)
    if nrm < 1e-12:
        return 0.0
    normal = normal / nrm
    dist = (xyz - c) @ normal
    return float(np.sqrt(np.mean(dist * dist)))


class MasksToCloudsPlanar(Node):
    def __init__(self):
        super().__init__('masks_to_pointclouds_planar')
        self.bridge = CvBridge()
        self.K = None
        self.depth = None
        self.frame = 0
        self.create_subscription(CameraInfo, '/camera/realsense/camera_info', self.on_info, 10)
        self.create_subscription(Image, '/camera/realsense/depth', self.on_depth, 10)
        self.create_subscription(
            TensorList, '/segment_anything/raw_segmentation_mask', self.on_masks, 10
        )
        self.get_logger().info(
            f'planar filter | ROI=({X_MIN},{Y_MIN})-({X_MAX},{Y_MAX}) | '
            f'area=[{MIN_AREA},{MAX_AREA}] | drop if plane_RMS<{PLANAR_RMS_MAX} m | '
            f'out={OUT_DIR}'
        )

    def on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def on_depth(self, msg: Image):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def on_masks(self, msg: TensorList):
        if self.K is None or self.depth is None or not msg.tensors:
            return

        t = msg.tensors[0]
        n, _, h, w = t.shape.dims
        masks = np.frombuffer(bytes(t.data), dtype=np.uint8).reshape(n, h, w)

        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        depth = self.depth
        if depth.shape[0] != h or depth.shape[1] != w:
            self.get_logger().warn(f'depth {depth.shape} != mask {(h, w)}; skip')
            return

        self.frame += 1

        candidates = []
        for i in range(n):
            m = masks[i] > 0
            roi = np.zeros_like(m)
            roi[Y_MIN:Y_MAX, X_MIN:X_MAX] = True
            m = m & roi
            area = int(m.sum())
            if area < MIN_AREA or area > MAX_AREA:
                continue
            candidates.append((area, m))

        kept_masks = nms_masks(candidates, IOU_MERGE)

        clouds = []
        dropped_planar = 0
        for area, m in kept_masks:
            ys, xs = np.where(m)
            z = depth[ys, xs]
            valid = np.isfinite(z) & (z > MIN_DEPTH) & (z < MAX_DEPTH)
            xs, ys, z = xs[valid], ys[valid], z[valid]
            if len(z) < MIN_POINTS:
                continue

            X = (xs - cx) * z / fx
            Y = (ys - cy) * z / fy
            xyz = np.stack([X, Y, z], axis=1)

            rms = plane_rms(xyz)
            z_span = float(z.max() - z.min())
            if rms < PLANAR_RMS_MAX:
                dropped_planar += 1
                continue

            clouds.append((area, xyz, rms, z_span))

        self.get_logger().info(
            f'frame {self.frame}: {n} raw -> {len(candidates)} area -> '
            f'{len(kept_masks)} NMS -> {dropped_planar} planar drop -> {len(clouds)} clouds'
        )
        for j, (area, xyz, rms, z_span) in enumerate(clouds):
            self.get_logger().info(
                f'  cloud[{j}] area={area} n_pts={len(xyz)} '
                f'plane_RMS={rms:.5f} m z_span={z_span:.4f} m'
            )

        if self.frame % SAVE_EVERY_N != 0 or not clouds:
            return

        stamp = f'{msg.header.stamp.sec}_{msg.header.stamp.nanosec}'
        for j, (area, xyz, rms, z_span) in enumerate(clouds):
            if len(xyz) > MAX_POINTS_SAVE:
                idx = np.random.choice(len(xyz), MAX_POINTS_SAVE, replace=False)
                xyz = xyz[idx]
            out = OUT_DIR / f'cloud_{stamp}_{j}_a{area}.ply'
            write_ply(out, xyz)
            self.get_logger().info(
                f'saved {out} ({len(xyz)} pts, plane_RMS={rms:.5f}, z_span={z_span:.4f})'
            )


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = MasksToCloudsPlanar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
