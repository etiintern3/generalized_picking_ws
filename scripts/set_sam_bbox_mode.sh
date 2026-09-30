#!/usr/bin/env bash
# Wait until SAM encoder is up, then force bbox prompts.
set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
set -u

echo "[phase1] Waiting for /sam_data_encoder_node ..."
for i in $(seq 1 60); do
  if ros2 node list 2>/dev/null | grep -q '/sam_data_encoder_node'; then
    break
  fi
  sleep 1
done

if ! ros2 node list 2>/dev/null | grep -q '/sam_data_encoder_node'; then
  echo "[phase1] ERROR: sam_data_encoder_node not found"
  exit 1
fi

ros2 param set /sam_data_encoder_node prompt_input_type bbox
echo "[phase1] prompt_input_type := bbox"
ros2 param get /sam_data_encoder_node prompt_input_type
