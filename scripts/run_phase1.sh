#!/usr/bin/env bash
# One-command Phase 1 bringup (run INSIDE Isaac ROS Dev container).
# Prereq: Isaac Sim publishing /camera/realsense/{rgb,depth,camera_info}
#
# Usage:
#   bash ${ISAAC_ROS_WS}/scripts/run_phase1.sh
#   USE_RQT=0 bash ${ISAAC_ROS_WS}/scripts/run_phase1.sh    # no rqt window
#   USE_VIZ=0 bash ${ISAAC_ROS_WS}/scripts/run_phase1.sh    # no colored-mask viz
#
# Ctrl+C stops everything started by this script.
set -eo pipefail

WS="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"
SCRIPTS="${WS}/scripts"
ASSETS="${WS}/isaac_ros_assets"
USE_RQT="${USE_RQT:-1}"
USE_VIZ="${USE_VIZ:-1}"

# ROS setup.bash references optional unset vars; disable nounset around it
set +u
source /opt/ros/humble/setup.bash
set -u

if ! ros2 pkg prefix isaac_ros_examples >/dev/null 2>&1; then
  echo "[phase1] isaac_ros_examples missing — run:"
  echo "  bash ${SCRIPTS}/bootstrap_isaac_ros_packages.sh"
  echo "or rebuild with enable_persistent_isaac_ros.sh"
  exit 1
fi

if ! ros2 topic list 2>/dev/null | grep -q '/camera/realsense/rgb'; then
  echo "[phase1] WARNING: /camera/realsense/rgb not seen yet."
  echo "         Start Isaac Sim / bag first. Continuing anyway..."
fi

PIDS=()
cleanup() {
  echo ""
  echo "[phase1] Shutting down..."
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  echo "[phase1] Done."
}
trap cleanup EXIT INT TERM

echo "[phase1] Starting SAM launch..."
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py \
  launch_fragments:=segment_anything \
  interface_specs_file:="${ASSETS}/isaac_ros_segment_anything/sim_d455_interface_specs.json" \
  sam_model_repository_paths:=["${ASSETS}/models"] \
  &
PIDS+=($!)

# Let SAM nodes come up, then set bbox mode
bash "${SCRIPTS}/set_sam_bbox_mode.sh"

echo "[phase1] Starting tray grid prompts..."
python3 "${SCRIPTS}/tray_grid_prompts.py" &
PIDS+=($!)

echo "[phase1] Starting masks → point clouds (NMS)..."
python3 "${SCRIPTS}/masks_to_pointclouds_nms.py" &
PIDS+=($!)

if [[ "${USE_VIZ}" == "1" ]]; then
  echo "[phase1] Starting visualize_mask..."
  ros2 run isaac_ros_segment_anything visualize_mask.py --ros-args \
    --remap /yolov8_encoder/resize/image:=/camera/realsense/rgb \
    &
  PIDS+=($!)
fi

if [[ "${USE_RQT}" == "1" ]]; then
  echo "[phase1] Starting rqt_image_view..."
  ros2 run rqt_image_view rqt_image_view /segment_anything/colored_segmentation_mask &
  PIDS+=($!)
fi

echo ""
echo "[phase1] All started. PLYs → ${WS}/output/clouds_nms/"
echo "[phase1] Ctrl+C to stop everything."
echo ""

# Wait on background jobs (Ctrl+C triggers cleanup)
wait
