#!/usr/bin/env python3
"""Dump one front-camera scene for Contact-GraspNet (full depth + object segmap).

This is NOT the isolated-object PLY path. CGN gets:
  - full RGB-D scene (table + objects in the depth cloud)
  - instance segmap for kept object masks only (after area/NMS/planarity)

Run with CGN: --local_regions --filter_grasps
"""
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from isaac_ros_tensor_list_interfaces.msg import TensorList
from cv_bridge import CvBridge

# Match frontcam + planar filter (tune to match masks_to_pointclouds_planar.py)
X_MIN, X_MAX = 200, 1050
Y_MIN, Y_MAX = 500, 720
MIN_AREA = 3000
MAX_AREA = 180000
IOU_MERGE = 0.9
MIN_DEPTH = 0.05
MAX_DEPTH = 5.0
MIN_POINTS = 500
PLANAR_RMS_MAX = 0.004

OUT_DIR = Path('/workspaces/isaac_ros-dev/output/cgn_scenes')
SAVE_EVERY_N = 60  # save less often; one good frame is enough


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
    if len(xyz) < 3:
        return 0.0
    c = xyz.mean(axis=0)
    _, _, vh = np.linalg.svd(xyz - c, full_matrices=False)
    normal = vh[-1]
    nrm = np.linalg.norm(normal)
    if nrm < 1e-12:
        return 0.0
    normal = normal / nrm
    dist = (xyz - c) @ normal
    return float(np.sqrt(np.mean(dist * dist)))


class SamSceneToCgn(Node):
    def __init__(self):
        super().__init__('sam_scene_to_cgn')
        self.bridge = CvBridge()
        self.K = None
        self.depth = None
        self.rgb = None
        self.frame = 0
        self.create_subscription(CameraInfo, '/camera/realsense/camera_info', self.on_info, 10)
        self.create_subscription(Image, '/camera/realsense/depth', self.on_depth, 10)
        self.create_subscription(Image, '/camera/realsense/rgb', self.on_rgb, 10)
        self.create_subscription(
            TensorList, '/segment_anything/raw_segmentation_mask', self.on_masks, 10
        )
        self.get_logger().info(
            f'CGN scene dump | ROI=({X_MIN},{Y_MIN})-({X_MAX},{Y_MAX}) | '
            f'planar_RMS<{PLANAR_RMS_MAX} drop | out={OUT_DIR}'
        )

    def on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def on_depth(self, msg: Image):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

    def on_rgb(self, msg: Image):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def on_masks(self, msg: TensorList):
        if self.K is None or self.depth is None or self.rgb is None or not msg.tensors:
            return

        t = msg.tensors[0]
        n, _, h, w = t.shape.dims
        masks = np.frombuffer(bytes(t.data), dtype=np.uint8).reshape(n, h, w)
        depth = self.depth
        rgb = self.rgb
        if depth.shape[:2] != (h, w) or rgb.shape[:2] != (h, w):
            self.get_logger().warn(
                f'size mismatch depth={depth.shape} rgb={rgb.shape} mask={(h,w)}; skip'
            )
            return

        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
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

        kept = nms_masks(candidates, IOU_MERGE)

        seg = np.zeros((h, w), dtype=np.int32)
        obj_id = 0
        for area, m in kept:
            ys, xs = np.where(m)
            z = depth[ys, xs]
            valid = np.isfinite(z) & (z > MIN_DEPTH) & (z < MAX_DEPTH)
            xs_v, ys_v, z_v = xs[valid], ys[valid], z[valid]
            if len(z_v) < MIN_POINTS:
                continue
            X = (xs_v - cx) * z_v / fx
            Y = (ys_v - cy) * z_v / fy
            xyz = np.stack([X, Y, z_v], axis=1)
            if plane_rms(xyz) < PLANAR_RMS_MAX:
                continue
            obj_id += 1
            seg[m] = obj_id

        n_obj = int(seg.max())
        self.get_logger().info(
            f'frame {self.frame}: {n} raw -> {len(candidates)} area -> '
            f'{len(kept)} NMS -> {n_obj} object ids in segmap'
        )

        if self.frame % SAVE_EVERY_N != 0 or n_obj == 0:
            return

        stamp = f'{msg.header.stamp.sec}_{msg.header.stamp.nanosec}'
        out = OUT_DIR / f'scene_{stamp}.npz'
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        # Keys match Contact-GraspNet load_available_input_data
        np.savez(
            out,
            rgb=rgb.astype(np.uint8),
            depth=depth.astype(np.float32),
            K=self.K.astype(np.float64),
            segmap=seg.astype(np.int32),
        )
        self.get_logger().info(f'saved {out}  objects={n_obj}  (full scene depth + seg)')


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = SamSceneToCgn()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
