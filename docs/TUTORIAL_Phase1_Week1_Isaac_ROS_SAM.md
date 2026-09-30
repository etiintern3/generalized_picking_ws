# Tutorial — Phase 1 Week 1: Isaac ROS SAM1 on Isaac Sim (UR10e Tray Cell)

**Project:** Generalized picking (UR10e + Robotiq 2F-85)  
**Goal of this tutorial:** From a clean machine state to **Segment Anything (SAM1)** running on your Isaac Sim RealSense D455 view of the tray, with a **manual prompt** producing a visible mask on an object.

**Do not modify** `~/ur_ws` or the working pick-and-place script. This work lives in:

```text
/home/satwik/generalized_picking_ws
```

---

## 0. What we decided (context)

| Decision | Choice |
|----------|--------|
| Perception model | **SAM1** via **Isaac ROS** (`isaac_ros_segment_anything`) |
| Why not SAM2 | Phase 1 is single-frame masks, not video tracking |
| Why not Meta standalone | Prefer Isaac ROS for ROS2 / later nvblox path |
| Why not YOLO alone | Need **novel** objects without per-object training |
| Isaac ROS version | **3.2** (matches **ROS 2 Humble** on Ubuntu 22.04) |
| Not Isaac ROS 4.x/5.x | Those target Jazzy; your MoveIt stack is Humble |
| Runtime | Isaac ROS **Dev Docker** container |
| Isaac Sim | **5.1.0** on the **host** (not inside the container) |
| “Mirroring” | MoveIt → `/joint_states` → remapper → `/joint_command_isaac` → Sim (unchanged in Phase 1) |

**Important SAM concept — prompts:**  
Isaac ROS SAM does **not** auto-find all objects. You must send a **prompt** (point or bounding box) on topic `/prompts`. No prompt (or stamp `0`) → often **no mask**.

---

## 1. Machine baseline (what we verified)

```bash
echo "ROS_DISTRO=$ROS_DISTRO"    # humble
which ros2                       # /opt/ros/humble/bin/ros2
nvidia-smi --query-gpu=name,memory.total --format=csv
# NVIDIA GeForce RTX 3090, 24576 MiB  → full SAM1 is fine
```

Isaac Sim start (host):

```bash
cd ~/isaac-sim
./isaac-sim.selector.sh
```

---

## 2. Create the new workspace

```bash
mkdir -p /home/satwik/generalized_picking_ws/src
cd /home/satwik/generalized_picking_ws
git init

echo 'export ISAAC_ROS_WS=/home/satwik/generalized_picking_ws' >> ~/.bashrc
source ~/.bashrc
echo "ISAAC_ROS_WS=$ISAAC_ROS_WS"
```

---

## 3. Docker GPU access

```bash
sudo usermod -aG docker $USER
# Log out and back in (required for group change)

groups | grep -q docker && echo "docker group: YES"

docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```

---

## 4. Install git-lfs (host)

`run_dev.sh` requires git-lfs:

```bash
sudo apt-get install -y git-lfs
git lfs install
git lfs version
```

---

## 5. Clone Isaac ROS common (Humble / release-3.2) and enter Dev container

```bash
source ~/.bashrc
cd ${ISAAC_ROS_WS}/src
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common

cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh
```

First run **builds a large Docker image** (many GB, can take a long time).

**Issues we hit and fixes:**

| Problem | Fix |
|---------|-----|
| `permission denied` on docker.sock | Add user to `docker` group + re-login |
| Script exits immediately / “root privileges” in some environments | Run as normal user after docker group works; ensure `git-lfs` installed |
| Build fails on CV-CUDA `wget` SSL | Retry `./scripts/run_dev.sh` (layers cached); or pre-download debs |

Success looks like:

```text
admin@HOSTNAME:/workspaces/isaac_ros-dev$
echo $ROS_DISTRO   # humble
pwd                # /workspaces/isaac_ros-dev
```

**Note:** Host path `~/generalized_picking_ws` is mounted as `/workspaces/isaac_ros-dev` inside the container (`ISAAC_ROS_WS` points there).

Re-attach later:

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh   # attaches if container already running
```

---

## 6. Install Isaac ROS Segment Anything (inside container)

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-isaac-ros-segment-anything

source /opt/ros/humble/setup.bash
ros2 pkg prefix isaac_ros_segment_anything
ros2 pkg executables isaac_ros_segment_anything
# torch_to_onnx.py
# visualize_mask.py
```

---

## 7. Download SAM weights (host) and place in workspace

On **host** (`satwik@...`):

```bash
cd ~/Downloads
wget -c https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth -O vit_b.pth

mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_segment_anything
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/models/triton/segment_anything/1

mv ~/Downloads/vit_b.pth ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_segment_anything/vit_b.pth
ls -lh ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_segment_anything/vit_b.pth
# ~358M
```

---

## 8. Convert PyTorch → ONNX (inside container)

### 8.1 Dependencies

```bash
cd ${ISAAC_ROS_WS}
pip3 install --break-system-packages git+https://github.com/facebookresearch/segment-anything.git
pip3 install --break-system-packages torchvision
pip3 install --break-system-packages onnxscript onnx
```

Optional (if colcon complains later about setuptools):

```bash
pip3 install --break-system-packages 'setuptools>=65,<80'
pip3 install --break-system-packages 'numpy<2'   # needed for OpenCV / visualize_mask
```

### 8.2 Why the stock exporter failed

Container has **new PyTorch**; Isaac ROS 3.2 `torch_to_onnx.py` uses the old export API. New exporter errors on `dynamic_axes`. Fix: copy script and add `dynamo=False`.

```bash
mkdir -p ${ISAAC_ROS_WS}/scripts
cp /opt/ros/humble/lib/isaac_ros_segment_anything/torch_to_onnx.py \
  ${ISAAC_ROS_WS}/scripts/torch_to_onnx_legacy.py

sed -i 's/dynamic_axes=dynamic_axes,/dynamic_axes=dynamic_axes,\n                dynamo=False,/' \
  ${ISAAC_ROS_WS}/scripts/torch_to_onnx_legacy.py

sed -n '145,165p' ${ISAAC_ROS_WS}/scripts/torch_to_onnx_legacy.py
# confirm dynamo=False is present
```

### 8.3 Export

```bash
rm -f ${ISAAC_ROS_WS}/isaac_ros_assets/models/triton/segment_anything/1/model.onnx

python3 ${ISAAC_ROS_WS}/scripts/torch_to_onnx_legacy.py \
  --checkpoint ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_segment_anything/vit_b.pth \
  --output ${ISAAC_ROS_WS}/isaac_ros_assets/models/triton/segment_anything/1/model.onnx \
  --model-type vit_b \
  --sam-type SAM

ls -lh ${ISAAC_ROS_WS}/isaac_ros_assets/models/triton/segment_anything/1/model.onnx
# ~359M
```

---

## 9. NGC quickstart assets + Triton layout (Isaac ROS 3.2 paths)

Isaac ROS **3.2** expects:

```text
isaac_ros_assets/models/segment_anything/config.pbtxt
isaac_ros_assets/models/segment_anything/1/model.onnx
```

`sam_config_onnx.pbtxt` comes from NGC assets (not the apt package alone).

### 9.1 Download assets (inside container)

```bash
sudo apt-get install -y curl jq tar
cd ${ISAAC_ROS_WS}

NGC_ORG="nvidia"
NGC_TEAM="isaac"
NGC_RESOURCE="isaac_ros_segment_anything_assets"
NGC_FILENAME="quickstart.tar.gz"
MAJOR_VERSION=3
MINOR_VERSION=2
VERSION_REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/versions"
AVAILABLE_VERSIONS=$(curl -s -H "Accept: application/json" "$VERSION_REQ_URL")
LATEST_VERSION_ID=$(echo "$AVAILABLE_VERSIONS" | jq -r "
  .recipeVersions[]
  | .versionId as \$v
  | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\"))
  | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber}
  | select(.major == $MAJOR_VERSION and .minor <= $MINOR_VERSION)
  | \$v
" | sort -V | tail -n 1)
echo "Using NGC version: $LATEST_VERSION_ID"   # e.g. 3.1.0

FILE_REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/versions/$LATEST_VERSION_ID/files/$NGC_FILENAME"
curl -LO --request GET "${FILE_REQ_URL}"
tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets
rm ${NGC_FILENAME}

ls ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_segment_anything/
# should include sam_config_onnx.pbtxt, segment_anything_sample_data/, etc.
```

### 9.2 Place model where 3.2 expects it

```bash
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/models/segment_anything/1

cp ${ISAAC_ROS_WS}/isaac_ros_assets/models/triton/segment_anything/1/model.onnx \
   ${ISAAC_ROS_WS}/isaac_ros_assets/models/segment_anything/1/model.onnx

cp ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_segment_anything/sam_config_onnx.pbtxt \
   ${ISAAC_ROS_WS}/isaac_ros_assets/models/segment_anything/config.pbtxt

ls -lh ${ISAAC_ROS_WS}/isaac_ros_assets/models/segment_anything/
ls -lh ${ISAAC_ROS_WS}/isaac_ros_assets/models/segment_anything/1/
```

---

## 10. Smoke test A — NVIDIA sample rosbag

### Terminal A (container) — launch SAM

```bash
sudo apt-get install -y ros-humble-isaac-ros-examples
source /opt/ros/humble/setup.bash
cd ${ISAAC_ROS_WS}

ros2 launch isaac_ros_examples isaac_ros_examples.launch.py \
  launch_fragments:=segment_anything \
  interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_segment_anything/quickstart_interface_specs.json \
  sam_model_repository_paths:=[${ISAAC_ROS_WS}/isaac_ros_assets/models]
```

### Terminal B — play bag

```bash
# host: re-enter container
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh

source /opt/ros/humble/setup.bash
ros2 bag play -l ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_segment_anything/segment_anything_sample_data/
```

### Terminal C — visualize

If `visualize_mask` fails with NumPy / OpenCV:

```bash
pip3 install --break-system-packages 'numpy<2'
```

Then:

```bash
source /opt/ros/humble/setup.bash
ros2 run isaac_ros_segment_anything visualize_mask.py --ros-args \
  --remap /yolov8_encoder/resize/image:=/image

ros2 run rqt_image_view rqt_image_view /segment_anything/colored_segmentation_mask
```

**Success:** colored masks visible; e.g. `ros2 topic hz /segment_anything/raw_segmentation_mask` shows a rate.

Stop bag/launch when done (`Ctrl+C`).

---

## 11. Smoke test B — Your Isaac Sim tray + D455

### 11.1 Camera topics (host, Sim scene running)

Already publishing:

```text
/camera/realsense/rgb          sensor_msgs/Image
/camera/realsense/depth
/camera/realsense/camera_info
```

Verified:

```text
width: 1280, height: 720, encoding: rgb8
even dims: True   # required by Isaac ROS SAM
```

Start Sim:

```bash
cd ~/isaac-sim && ./isaac-sim.selector.sh
```

Load your scene (table, black source tray with objects, pink place tray, D455 looking down).

### 11.2 Interface specs for 1280×720 (container)

```bash
cat > ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_segment_anything/sim_d455_interface_specs.json <<'EOF'
{
    "input_image": {
        "width": 1280,
        "height": 720
    },
    "subscribed_topics": {
        "image": "/camera/realsense/rgb",
        "camera_info": "/camera/realsense/camera_info",
        "prompt": "/prompts"
    }
}
EOF
```

### 11.3 Launch SAM on Sim camera (container)

Sim on host + Isaac ROS container both use host networking → same ROS graph.

```bash
source /opt/ros/humble/setup.bash
cd ${ISAAC_ROS_WS}

ros2 launch isaac_ros_examples isaac_ros_examples.launch.py \
  launch_fragments:=segment_anything \
  interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_segment_anything/sim_d455_interface_specs.json \
  sam_model_repository_paths:=[${ISAAC_ROS_WS}/isaac_ros_assets/models]
```

Check:

```bash
ros2 topic hz /camera/realsense/rgb
ros2 topic hz /segment_anything/resized_image   # should show tray
```

### 11.4 Critical lesson — stamp prompts with the image header

**Broken:** publishing `/prompts` with `header.stamp = 0` → no (or rare) `raw_segmentation_mask`.  
**Working:** stamp every prompt with the same header as `/camera/realsense/rgb`.

```bash
source /opt/ros/humble/setup.bash

python3 - <<'PY'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

rclpy.init()
node = Node('manual_sam_prompt_sync')
pub = node.create_publisher(Detection2DArray, '/prompts', 10)
count = {'n': 0}

def on_image(img: Image):
    msg = Detection2DArray()
    msg.header = img.header  # CRITICAL
    det = Detection2D()
    det.header = img.header
    # Tune these pixel coords for your object in the black tray
    det.bbox.center.position.x = 400.0
    det.bbox.center.position.y = 350.0
    det.bbox.size_x = 100.0
    det.bbox.size_y = 80.0
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = 'object'
    hyp.hypothesis.score = 1.0
    det.results.append(hyp)
    msg.detections.append(det)
    pub.publish(msg)
    count['n'] += 1
    if count['n'] % 20 == 0:
        print(f'published {count["n"]} stamped prompts')

sub = node.create_subscription(Image, '/camera/realsense/rgb', on_image, 10)
print('Publishing stamped prompts. Ctrl+C to stop.')
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
node.destroy_node()
rclpy.shutdown()
PY
```

### 11.5 Visualize

```bash
# Fix numpy if needed
pip3 install --break-system-packages 'numpy<2'

ros2 run isaac_ros_segment_anything visualize_mask.py --ros-args \
  --remap /yolov8_encoder/resize/image:=/camera/realsense/rgb

ros2 run rqt_image_view rqt_image_view /segment_anything/colored_segmentation_mask
```

**Success (achieved):** colored mask on a box in the tray; `raw_segmentation_mask` has a non-zero rate.

If `colored_segmentation_mask` looks gray with no tray: usually `visualize_mask` remap wrong or **no raw masks** (check stamps / `hz`).

---

## 12. Architecture reminder (what is talking to what)

```text
[Host] Isaac Sim 5.1
        D455 RGB/Depth/CameraInfo
              |
              v
   /camera/realsense/rgb  (etc.)
              |
              |  (ROS 2, host network)
              v
[Docker] Isaac ROS Dev (Humble)
        resize/pad/normalize → Triton SAM ONNX
        /prompts  (Detection2DArray, stamped!)
              |
              v
   /segment_anything/raw_segmentation_mask
   /segment_anything/colored_segmentation_mask  (via visualize_mask)
```

Pick-and-place / MoveIt (`ur_ws`) is **not** involved in this tutorial.

---

## 13. Useful paths cheat sheet

| Item | Path |
|------|------|
| Workspace / `ISAAC_ROS_WS` | `/home/satwik/generalized_picking_ws` |
| Container mount | `/workspaces/isaac_ros-dev` |
| run_dev | `.../src/isaac_ros_common/scripts/run_dev.sh` |
| ONNX model | `isaac_ros_assets/models/segment_anything/1/model.onnx` |
| Triton config | `isaac_ros_assets/models/segment_anything/config.pbtxt` |
| Sim interface specs | `isaac_ros_assets/isaac_ros_segment_anything/sim_d455_interface_specs.json` |
| Legacy exporter | `scripts/torch_to_onnx_legacy.py` |
| Leave alone | `~/ur_ws`, `tutorial5_full.py` |

---

## 14. What’s next (not done yet)

1. **Grid prompts** over the black source tray (class-agnostic, no clicking)  
2. Mask filtering (drop tray / tiny / duplicates)  
3. Depth back-projection → per-object point clouds  
4. Novel-object milestone → then Phase 2 (grasp nets)

---

## 15. Quick restart checklist (daily)

1. Host: start Isaac Sim + load scene (camera publishing).  
2. Host: `cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh`  
3. Container: launch SAM with `sim_d455_interface_specs.json`  
4. Container: stamped prompt publisher (or later: grid node)  
5. Container: `visualize_mask` + `rqt_image_view` on colored mask  

---

*Documented from the Phase 1 Week 1 guided setup session. Update this file as grid / point-cloud steps land.*
