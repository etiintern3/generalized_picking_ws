#!/usr/bin/env python3
"""Filter SAM masks and back-project with depth to PLY point clouds."""
import struct
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from isaac_ros_tensor_list_interfaces.msg import TensorList
from cv_bridge import CvBridge
# Tray interior ROI (pixels)
#X_MIN, X_MAX = 540, 705
#Y_MIN, Y_MAX = 215, 475
X_MIN, X_MAX = 220, 1000
Y_MIN, Y_MAX = 500, 720
# Area filter (pixels inside mask)
MIN_AREA = 1000
MAX_AREA = 8000   # drop huge tray-bleed blobs
OUT_DIR = Path('/workspaces/isaac_ros-dev/output/clouds')
SAVE_EVERY_N = 30  # save about once per ~2s at 15 Hz
def write_ply(path: Path, xyz: np.ndarray):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w') as f:
        f.write('ply\nformat ascii 1.0\n')
        f.write(f'element vertex {len(xyz)}\n')
        f.write('property float x\nproperty float y\nproperty float z\n')
        f.write('end_header\n')
        for x, y, z in xyz:
            f.write(f'{x:.6f} {y:.6f} {z:.6f}\n')
class MasksToClouds(Node):
    def __init__(self):
        super().__init__('masks_to_pointclouds')
        self.bridge = CvBridge()
        self.K = None
        self.depth = None
        self.depth_stamp = None
        self.frame = 0
        self.create_subscription(CameraInfo, '/camera/realsense/camera_info', self.on_info, 10)
        self.create_subscription(Image, '/camera/realsense/depth', self.on_depth, 10)
        self.create_subscription(TensorList, '/segment_anything/raw_segmentation_mask', self.on_masks, 10)
        self.get_logger().info('Waiting for camera_info, depth, masks...')
    def on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
    def on_depth(self, msg: Image):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.depth_stamp = msg.header.stamp
    def on_masks(self, msg: TensorList):
        if self.K is None or self.depth is None:
            return
        if not msg.tensors:
            return
        t = msg.tensors[0]
        # shape [N, 1, H, W], uint8
        n, _, h, w = t.shape.dims
        masks = np.frombuffer(bytes(t.data), dtype=np.uint8).reshape(n, h, w)
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        depth = self.depth
        if depth.shape[0] != h or depth.shape[1] != w:
            self.get_logger().warn(f'depth {depth.shape} != mask {(h, w)}; skip')
            return
        self.frame += 1
        kept = []
        for i in range(n):
            m = masks[i] > 0
            # Restrict to tray ROI
            roi = np.zeros_like(m)
            roi[Y_MIN:Y_MAX, X_MIN:X_MAX] = True
            m = m & roi
            area = int(m.sum())
            if area < MIN_AREA or area > MAX_AREA:
                continue
            ys, xs = np.where(m)
            z = depth[ys, xs]
            valid = np.isfinite(z) & (z > 0.05) & (z < 5.0)
            xs, ys, z = xs[valid], ys[valid], z[valid]
            if len(z) < 500:
                continue
            X = (xs - cx) * z / fx
            Y = (ys - cy) * z / fy
            xyz = np.stack([X, Y, z], axis=1)
            kept.append((area, xyz))
        self.get_logger().info(f'frame {self.frame}: {n} masks -> {len(kept)} after filter')
        if self.frame % SAVE_EVERY_N != 0 or not kept:
            return
        stamp = f'{msg.header.stamp.sec}_{msg.header.stamp.nanosec}'
        for j, (area, xyz) in enumerate(kept):
            # subsample for smaller files
            if len(xyz) > 20000:
                idx = np.random.choice(len(xyz), 20000, replace=False)
                xyz = xyz[idx]
            out = OUT_DIR / f'cloud_{stamp}_{j}_a{area}.ply'
            write_ply(out, xyz)
            self.get_logger().info(f'saved {out} ({len(xyz)} pts)')
def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = MasksToClouds()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
