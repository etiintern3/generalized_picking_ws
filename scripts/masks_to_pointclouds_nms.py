#!/usr/bin/env python3
"""Filter SAM masks, merge duplicates (2D IoU NMS, keep smaller), back-project to PLY."""
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from isaac_ros_tensor_list_interfaces.msg import TensorList
from cv_bridge import CvBridge

# Tray interior ROI (pixels) — same tuned values as masks_to_pointclouds.py
X_MIN, X_MAX = 540, 705
Y_MIN, Y_MAX = 215, 475

# Area filter (pixels inside mask, after ROI)
MIN_AREA = 100
MAX_AREA = 4000

# If two masks overlap more than this, keep only the smaller one
IOU_MERGE = 0.4

# Depth validity (meters)
MIN_DEPTH = 0.05
MAX_DEPTH = 5.0
MIN_POINTS = 500

# Drop clouds whose average Z is at tray floor distance from camera (meters)
TRAY_Z_MIN = 1.30
TRAY_Z_MAX = 1.31

OUT_DIR = Path('/workspaces/isaac_ros-dev/output/clouds_nms')
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
    """candidates: list of (area, bool_mask). Keep smaller, drop high-IoU duplicates."""
    # Smallest first so tighter masks win; later (larger) overlaps are skipped.
    candidates = sorted(candidates, key=lambda x: x[0])
    kept = []
    for area, m in candidates:
        if any(mask_iou(m, km) >= iou_thresh for _, km in kept):
            continue
        kept.append((area, m))
    return kept


class MasksToCloudsNMS(Node):
    def __init__(self):
        super().__init__('masks_to_pointclouds_nms')
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
            f'NMS IoU>={IOU_MERGE} | ROI=({X_MIN},{Y_MIN})-({X_MAX},{Y_MAX}) | '
            f'area=[{MIN_AREA},{MAX_AREA}] | drop avg_Z in [{TRAY_Z_MIN},{TRAY_Z_MAX}] | '
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

        # 1) Area + ROI filter → candidate masks
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

        # 2) NMS: merge duplicate masks of the same object
        kept_masks = nms_masks(candidates, IOU_MERGE)

        # 3) Back-project surviving masks; drop tray-floor clouds by avg Z
        clouds = []
        dropped_tray_z = 0
        for area, m in kept_masks:
            ys, xs = np.where(m)
            z = depth[ys, xs]
            valid = np.isfinite(z) & (z > MIN_DEPTH) & (z < MAX_DEPTH)
            xs, ys, z = xs[valid], ys[valid], z[valid]
            if len(z) < MIN_POINTS:
                continue
            z_avg = float(z.mean())
            if TRAY_Z_MIN <= z_avg <= TRAY_Z_MAX:
                dropped_tray_z += 1
                continue
            X = (xs - cx) * z / fx
            Y = (ys - cy) * z / fy
            xyz = np.stack([X, Y, z], axis=1)
            clouds.append((area, xyz, z_avg))

        self.get_logger().info(
            f'frame {self.frame}: {n} raw -> {len(candidates)} area -> '
            f'{len(kept_masks)} NMS -> {dropped_tray_z} tray-Z drop -> {len(clouds)} clouds'
        )
        for j, (area, xyz, z_avg) in enumerate(clouds):
            self.get_logger().info(
                f'  cloud[{j}] area={area} n_pts={len(xyz)} avg_Z={z_avg:.4f} m'
            )

        if self.frame % SAVE_EVERY_N != 0 or not clouds:
            return

        stamp = f'{msg.header.stamp.sec}_{msg.header.stamp.nanosec}'
        for j, (area, xyz, z_avg) in enumerate(clouds):
            if len(xyz) > MAX_POINTS_SAVE:
                idx = np.random.choice(len(xyz), MAX_POINTS_SAVE, replace=False)
                xyz = xyz[idx]
            out = OUT_DIR / f'cloud_{stamp}_{j}_a{area}.ply'
            write_ply(out, xyz)
            self.get_logger().info(f'saved {out} ({len(xyz)} pts, avg_Z={z_avg:.4f})')


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = MasksToCloudsNMS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
