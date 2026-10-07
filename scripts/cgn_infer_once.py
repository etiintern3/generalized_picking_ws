#!/usr/bin/env python3
"""Headless Contact-GraspNet inference (no Open3D / matplotlib windows).

Run inside the jp_cgnet container with conda env jp_cgnet active.
Same options as contact_graspnet/inference.py, but skips visualization
so run_phase2.sh can finish without manual window closes.
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

import numpy as np
import tensorflow.compat.v1 as tf

tf.disable_eager_execution()
for gpu in tf.config.experimental.list_physical_devices("GPU"):
    tf.config.experimental.set_memory_growth(gpu, True)

_WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CGNET_ROOT = os.path.join(_WS, "third_party", "cgnet")
if not os.path.isdir(os.path.join(CGNET_ROOT, "contact_graspnet")):
    CGNET_ROOT = os.getcwd()
# Match `python contact_graspnet/inference.py`: script dir first, then repo root.
_CGN_PKG = os.path.join(CGNET_ROOT, "contact_graspnet")
sys.path.insert(0, CGNET_ROOT)
sys.path.insert(0, _CGN_PKG)
os.chdir(CGNET_ROOT)

import config_utils  # noqa: E402
from contact_grasp_estimator import GraspEstimator  # noqa: E402
from data import load_available_input_data  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ckpt_dir",
        default="checkpoints/scene_test_2048_bs3_hor_sigma_001",
    )
    parser.add_argument("--np_path", required=True)
    parser.add_argument("--z_range", default="[0.3,1.8]")
    parser.add_argument("--local_regions", action="store_true")
    parser.add_argument("--filter_grasps", action="store_true")
    parser.add_argument("--skip_border_objects", action="store_true")
    parser.add_argument("--forward_passes", type=int, default=5)
    parser.add_argument("--arg_configs", nargs="*", type=str, default=[])
    args = parser.parse_args()

    global_config = config_utils.load_config(
        args.ckpt_dir, batch_size=args.forward_passes, arg_configs=args.arg_configs
    )
    z_range = eval(str(args.z_range))

    grasp_estimator = GraspEstimator(global_config)
    grasp_estimator.build_network()
    saver = tf.train.Saver(save_relative_paths=True)
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    config.allow_soft_placement = True
    sess = tf.Session(config=config)
    grasp_estimator.load_weights(sess, saver, args.ckpt_dir, mode="test")

    os.makedirs("results", exist_ok=True)
    paths = glob.glob(args.np_path)
    if not paths:
        print(f"No files found: {args.np_path}", file=sys.stderr)
        sys.exit(1)

    for p in paths:
        print("Loading", p)
        segmap, rgb, depth, cam_K, pc_full, pc_colors = load_available_input_data(p, K=None)
        if segmap is None and (args.local_regions or args.filter_grasps):
            raise ValueError("Need segmentation map for --local_regions / --filter_grasps")

        if pc_full is None:
            print("Converting depth to point cloud(s)...")
            pc_full, pc_segments, _pc_colors = grasp_estimator.extract_point_clouds(
                depth,
                cam_K,
                segmap=segmap,
                rgb=rgb,
                skip_border_objects=args.skip_border_objects,
                z_range=z_range,
            )
        else:
            pc_segments = {}

        print("Generating Grasps...")
        pred_grasps_cam, scores, contact_pts, gripper_openings = (
            grasp_estimator.predict_scene_grasps(
                sess,
                pc_full,
                pc_segments=pc_segments,
                local_regions=args.local_regions,
                filter_grasps=args.filter_grasps,
                forward_passes=args.forward_passes,
            )
        )

        out_name = os.path.basename(p.replace("png", "npz").replace("npy", "npz"))
        out_path = os.path.join("results", f"predictions_{out_name}")
        np.savez(
            out_path,
            pred_grasps_cam=pred_grasps_cam,
            scores=scores,
            contact_pts=contact_pts,
            gripper_openings=gripper_openings,
        )
        print(f"saved {os.path.abspath(out_path)}")


if __name__ == "__main__":
    main()
