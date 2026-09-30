#!/usr/bin/env bash
# Run INSIDE the Isaac ROS Dev container if packages are missing.
# Prefer enable_persistent_isaac_ros.sh (host) so this is rarely needed.
set -euo pipefail

sudo apt-get update
sudo apt-get install -y \
  ros-humble-isaac-ros-examples \
  ros-humble-isaac-ros-segment-anything \
  ros-humble-cv-bridge \
  python3-numpy

pip3 install --break-system-packages 'numpy<2'

source /opt/ros/humble/setup.bash
ros2 pkg prefix isaac_ros_examples
ros2 pkg prefix isaac_ros_segment_anything
python3 -c "import numpy; import cv2; print('numpy', numpy.__version__, 'cv2 OK')"
echo "Bootstrap done."
