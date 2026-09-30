# Tutorial — Phase 1 Week 3: Point Clouds & Bringup (continuation)

**For:** people who already finished [Week 1](./TUTORIAL_Phase1_Week1_Isaac_ROS_SAM.md) and [Week 2](./TUTORIAL_Phase1_Week2_Grid_and_Masks.md).

**You already have:** `~/generalized_picking_ws`, Isaac ROS Dev, SAM ONNX, flat tray, working bbox grid masks.

**Do not** clone the GitHub repo over your existing workspace. Work in the folder you already use:

```bash
export ISAAC_ROS_WS=$HOME/generalized_picking_ws   # or your path
cd ${ISAAC_ROS_WS}
```

**Optional later:** copy finished scripts up to GitHub / pull only if you know how to merge. For this tutorial, create files with the commands below (same as we did in the lab).

**Do not modify** `~/ur_ws`.

---

## 0. Goal

Turn SAM masks + depth into **object-only `.ply` point clouds**, then optionally one bringup script.

---

## 1. Confirm camera + masks still work

**Host:** Isaac Sim running, then:

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /camera/realsense/rgb
ros2 topic hz /camera/realsense/depth
ros2 topic echo /camera/realsense/depth --once | head -20
# encoding should be 32FC1, size 1280x720
```

**Host → container:**

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh
```

If packages missing after a fresh container:

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-isaac-ros-examples ros-humble-isaac-ros-segment-anything ros-humble-cv-bridge python3-numpy
pip3 install --break-system-packages 'numpy<2'
source /opt/ros/humble/setup.bash
```

Start SAM + grid as in Week 2 (multi-terminal). Confirm colored masks look good.

Inspect mask topic:

```bash
source /opt/ros/humble/setup.bash
ros2 topic info /segment_anything/raw_segmentation_mask -v
# Type: isaac_ros_tensor_list_interfaces/msg/TensorList
```

Quick tensor shape check:

```bash
python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from isaac_ros_tensor_list_interfaces.msg import TensorList
rclpy.init(); node = Node('m'); got={}
def cb(msg):
    t=msg.tensors[0]
    got['shape']=list(t.shape.dims); got['nbytes']=len(t.data)
sub=node.create_subscription(TensorList,'/segment_anything/raw_segmentation_mask',cb,10)
t0=time.time()
while 'shape' not in got and time.time()-t0<8: rclpy.spin_once(node,timeout_sec=0.2)
print(got or 'NO MASK'); node.destroy_node(); rclpy.shutdown()
PY
# expect something like shape [N, 1, 720, 1280]
```

---

## 2. Optional — record a rosbag (host)

```bash
source /opt/ros/humble/setup.bash
mkdir -p ${ISAAC_ROS_WS}/bags
ros2 bag record -o ${ISAAC_ROS_WS}/bags/tray_d455_$(date +%Y%m%d) \
  /camera/realsense/rgb \
  /camera/realsense/depth \
  /camera/realsense/camera_info
# Ctrl+C after 30–60 s

# Play with ROS 2 (not `rosbag`):
ros2 bag play -l ${ISAAC_ROS_WS}/bags/tray_d455_YYYYMMDD
```

---

## 3. Point clouds — first script (area + ROI only)

### 3.1 Create the file (host or container — same mounted path)

```bash
mkdir -p ${ISAAC_ROS_WS}/scripts ${ISAAC_ROS_WS}/output/clouds
```

Create `${ISAAC_ROS_WS}/scripts/masks_to_pointclouds.py` with your editor, or:

```bash
# If you have the file from the team GitHub and want only this script:
#   curl -fsSL -o ${ISAAC_ROS_WS}/scripts/masks_to_pointclouds.py \
#     https://raw.githubusercontent.com/etiintern3/generalized_picking_ws/main/scripts/masks_to_pointclouds.py
```

Or paste the version you already use from the repo / previous session. Core idea:

```text
masks → ROI crop → area filter → depth back-project → .ply
```

**Run inside container** (SAM + grid already running):

```bash
source /opt/ros/humble/setup.bash
python3 ${ISAAC_ROS_WS}/scripts/masks_to_pointclouds.py
```

Check:

```bash
ls -lh ${ISAAC_ROS_WS}/output/clouds/ | tail
```

View `.ply` on the **host** (Open3D / MeshLab). Path on host = same folder under `~/generalized_picking_ws/output/clouds/`.

Tune `MIN_AREA` / `MAX_AREA` / ROI in that file until clouds match objects.

---

## 4. Duplicate clouds → NMS script (keep smaller mask)

**Problem:** 3 objects but 4 PLYs — overlapping prompts.

**Do not edit** `masks_to_pointclouds.py`. Create a new file:

```bash
mkdir -p ${ISAAC_ROS_WS}/output/clouds_nms
```

Add `scripts/masks_to_pointclouds_nms.py` (from team repo raw URL or copy from your working tree). It adds:

- IoU NMS — keep **smaller** overlapping mask  
- Tray-Z filter — drop avg Z in `[TRAY_Z_MIN, TRAY_Z_MAX]` (e.g. 1.30–1.31 m)

```bash
# Optional download if you trust the GitHub copy:
curl -fsSL -o ${ISAAC_ROS_WS}/scripts/masks_to_pointclouds_nms.py \
  https://raw.githubusercontent.com/etiintern3/generalized_picking_ws/main/scripts/masks_to_pointclouds_nms.py
```

**Run (container):**

```bash
source /opt/ros/humble/setup.bash
python3 ${ISAAC_ROS_WS}/scripts/masks_to_pointclouds_nms.py
```

Logs like:

```text
6 raw -> 4 area -> 3 NMS -> 0 tray-Z drop -> 3 clouds
```

```bash
ls -lh ${ISAAC_ROS_WS}/output/clouds_nms/ | tail
```

### Grid box size reminder (Week 2)

If `BOX_W` ≈ tray width, two objects become **one** mask. Prefer boxes like **70×70** with a denser grid, not 160-wide boxes on a ~165 px tray.

---

## 5. Multi-terminal Phase 1 (same style as Week 1–2)

All commands **inside** `admin@` container unless noted. Sim publishing on host.

### Terminal A — SAM

```bash
source /opt/ros/humble/setup.bash
cd ${ISAAC_ROS_WS}

ros2 launch isaac_ros_examples isaac_ros_examples.launch.py \
  launch_fragments:=segment_anything \
  interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_segment_anything/sim_d455_interface_specs.json \
  sam_model_repository_paths:=[${ISAAC_ROS_WS}/isaac_ros_assets/models]
```

### Terminal B — bbox mode + grid

```bash
source /opt/ros/humble/setup.bash
ros2 param set /sam_data_encoder_node prompt_input_type bbox

python3 ${ISAAC_ROS_WS}/scripts/tray_grid_prompts.py
```

If `tray_grid_prompts.py` does not exist yet, create it (ROI / 5×5 / 70×70 as in Week 2 success), or:

```bash
curl -fsSL -o ${ISAAC_ROS_WS}/scripts/tray_grid_prompts.py \
  https://raw.githubusercontent.com/etiintern3/generalized_picking_ws/main/scripts/tray_grid_prompts.py
```

### Terminal C — point clouds

```bash
source /opt/ros/humble/setup.bash
python3 ${ISAAC_ROS_WS}/scripts/masks_to_pointclouds_nms.py
```

### Terminal D — visualize (optional)

```bash
source /opt/ros/humble/setup.bash
ros2 run isaac_ros_segment_anything visualize_mask.py --ros-args \
  --remap /yolov8_encoder/resize/image:=/camera/realsense/rgb

ros2 run rqt_image_view rqt_image_view /segment_anything/colored_segmentation_mask
```

---

## 6. Optional — one-command bringup (after multi-terminal works)

Still **no full-repo clone**. Fetch only the helper scripts if needed:

```bash
cd ${ISAAC_ROS_WS}/scripts
curl -fsSL -O https://raw.githubusercontent.com/etiintern3/generalized_picking_ws/main/scripts/run_phase1.sh
curl -fsSL -O https://raw.githubusercontent.com/etiintern3/generalized_picking_ws/main/scripts/set_sam_bbox_mode.sh
chmod +x run_phase1.sh set_sam_bbox_mode.sh
```

Then (Sim up, **one** container terminal):

```bash
bash ${ISAAC_ROS_WS}/scripts/run_phase1.sh
```

Ctrl+C stops everything.  
`USE_RQT=0` / `USE_VIZ=0` to skip GUI pieces.

---

## 7. Optional — persistent apt packages

```bash
# Host — once
mkdir -p ${ISAAC_ROS_WS}/docker ${ISAAC_ROS_WS}/scripts
curl -fsSL -o ${ISAAC_ROS_WS}/scripts/enable_persistent_isaac_ros.sh \
  https://raw.githubusercontent.com/etiintern3/generalized_picking_ws/main/scripts/enable_persistent_isaac_ros.sh
curl -fsSL -o ${ISAAC_ROS_WS}/docker/Dockerfile.perception \
  https://raw.githubusercontent.com/etiintern3/generalized_picking_ws/main/docker/Dockerfile.perception
curl -fsSL -o ${ISAAC_ROS_WS}/scripts/bootstrap_isaac_ros_packages.sh \
  https://raw.githubusercontent.com/etiintern3/generalized_picking_ws/main/scripts/bootstrap_isaac_ros_packages.sh
chmod +x ${ISAAC_ROS_WS}/scripts/enable_persistent_isaac_ros.sh \
         ${ISAAC_ROS_WS}/scripts/bootstrap_isaac_ros_packages.sh

bash ${ISAAC_ROS_WS}/scripts/enable_persistent_isaac_ros.sh

# Exit all containers, then:
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh
```

Or quick bootstrap inside container each time:

```bash
bash ${ISAAC_ROS_WS}/scripts/bootstrap_isaac_ros_packages.sh
```

---

## 8. Phase 1 done when

- Colored masks cover objects in the tray  
- `output/clouds_nms/` has about **one cloud per object**  
- Floor-only clouds are filtered out  

**Next:** Phase 2 grasp generation — or see the [standalone full-pipeline tutorial](./TUTORIAL_Phase1_Standalone_From_GitHub.md) if onboarding someone on a new machine.
