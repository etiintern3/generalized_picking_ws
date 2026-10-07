# Tutorial — Phase 1 Standalone: Full Pipeline from GitHub

**For:** a **new machine** or anyone who did **not** follow Week 1–2.  
**Goal:** clone the repo and run the complete Phase 1 pipeline (Sim camera → SAM → object point clouds).

**Repo:** https://github.com/etiintern3/generalized_picking_ws  

If you **already** finished Week 1–2 in `~/generalized_picking_ws`, do **not** clone on top of that folder. Use [Week 3 continuation](./TUTORIAL_Phase1_Week3_PointClouds_Continuation.md) instead.

**Do not modify** `~/ur_ws`.

---

## 0. What you need

| Item | Notes |
|------|--------|
| Ubuntu 22.04 | ROS 2 Humble |
| NVIDIA GPU + Docker | Isaac ROS Dev |
| Isaac Sim 5.1 | Scene with D455 → `/camera/realsense/{rgb,depth,camera_info}` |
| Flat tray + objects | Mesh trays hurt SAM |

Detailed SAM/Docker debugging: [Week 1](./TUTORIAL_Phase1_Week1_Isaac_ROS_SAM.md), [Week 2](./TUTORIAL_Phase1_Week2_Grid_and_Masks.md).

---

## 1. Clone into a new directory

```bash
cd ~
git clone https://github.com/etiintern3/generalized_picking_ws.git
cd generalized_picking_ws

echo 'export ISAAC_ROS_WS=$HOME/generalized_picking_ws' >> ~/.bashrc
source ~/.bashrc
echo "ISAAC_ROS_WS=$ISAAC_ROS_WS"
```

### In git vs not in git

| Shipped in repo | You must add (large / local) |
|-----------------|------------------------------|
| Scripts, docs, `docker/Dockerfile.perception` | `src/isaac_ros_common` |
| Interface JSONs + `sam_config_onnx.pbtxt` | `vit_b.pth`, `models/.../model.onnx` |
| | Bags / `output/` clouds |

---

## 2. Docker + Isaac ROS Dev (Humble / 3.2)

```bash
# docker group + GPU (see Week 1 if this fails)
groups | grep docker
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi

sudo apt-get install -y git-lfs
git lfs install

mkdir -p ${ISAAC_ROS_WS}/src
cd ${ISAAC_ROS_WS}/src
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common

cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh
```

Prompt: `admin@...:/workspaces/isaac_ros-dev$`

---

## 3. Install ROS packages inside the container

```bash
bash ${ISAAC_ROS_WS}/scripts/bootstrap_isaac_ros_packages.sh
```

Check:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix isaac_ros_segment_anything
ros2 pkg prefix isaac_ros_examples
```

### Optional — bake packages into the image (once)

**Host:**

```bash
bash ${ISAAC_ROS_WS}/scripts/enable_persistent_isaac_ros.sh
# exit all containers
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh
```

Details: Week 1 container issues + [Week 3 § persistent packages](./TUTORIAL_Phase1_Week3_PointClouds_Continuation.md).

---

## 4. Download SAM weights and export ONNX

**Host:**

```bash
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_segment_anything
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/models/segment_anything/1

cd ~/Downloads
wget -c https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth -O vit_b.pth
mv vit_b.pth ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_segment_anything/vit_b.pth
```

**Container:**

```bash
source /opt/ros/humble/setup.bash
pip3 install --break-system-packages git+https://github.com/facebookresearch/segment-anything.git
pip3 install --break-system-packages onnxscript onnx torchvision

python3 ${ISAAC_ROS_WS}/scripts/torch_to_onnx_legacy.py \
  --checkpoint ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_segment_anything/vit_b.pth \
  --output ${ISAAC_ROS_WS}/isaac_ros_assets/models/segment_anything/1/model.onnx \
  --model-type vit_b \
  --sam-type SAM

cp ${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_segment_anything/sam_config_onnx.pbtxt \
   ${ISAAC_ROS_WS}/isaac_ros_assets/models/segment_anything/config.pbtxt

ls -lh ${ISAAC_ROS_WS}/isaac_ros_assets/models/segment_anything/1/model.onnx
```

Expect ~359M ONNX. (Export quirks / `dynamo=False`: Week 1.)

Repo already has `sim_d455_interface_specs.json` (1280×720 → `/camera/realsense/...`).

---

## 5. Isaac Sim scene

```bash
# Host
cd ~/isaac-sim && ./isaac-sim.selector.sh
```

Requirements:

- RealSense-like camera publishing:
  - `/camera/realsense/rgb`
  - `/camera/realsense/depth` (`32FC1`)
  - `/camera/realsense/camera_info`
- Flat tray + objects  
- Prefer 1280×720 even dimensions  

If topic names differ, edit  
`isaac_ros_assets/isaac_ros_segment_anything/sim_d455_interface_specs.json`  
and remaps in `scripts/run_phase1.sh`.

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /camera/realsense/rgb
ros2 topic hz /camera/realsense/depth
```

---

## 6. Calibrate ROI / tray Z (first time on your scene)

Defaults in repo scripts (`540–705`, `215–475`, tray Z `1.30–1.31`) match **our** cell. Your Sim may differ.

### ROI overlay

```bash
# Container
source /opt/ros/humble/setup.bash
python3 - <<'PY'
import cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
X_MIN,X_MAX,Y_MIN,Y_MAX = 540,705,215,475
rclpy.init(); node=Node('roi'); bridge=CvBridge()
pub=node.create_publisher(Image,'/debug/roi_overlay',10)
def cb(msg):
    img=bridge.imgmsg_to_cv2(msg,'rgb8'); h,w=img.shape[:2]
    out=img.copy()
    cv2.rectangle(out,(max(0,X_MIN),max(0,Y_MIN)),(min(w-1,X_MAX),min(h-1,Y_MAX)),(0,255,0),3)
    pub.publish(bridge.cv2_to_imgmsg(out,'rgb8'))
node.create_subscription(Image,'/camera/realsense/rgb',cb,10)
try: rclpy.spin(node)
except KeyboardInterrupt: pass
node.destroy_node(); rclpy.shutdown()
PY
```

```bash
ros2 run rqt_image_view rqt_image_view /debug/roi_overlay
```

Edit numbers; copy into `scripts/tray_grid_prompts.py` and `scripts/masks_to_pointclouds_nms.py`.

Grid: keep `BOX_W/H` **much smaller than tray width** (e.g. 70×70). Week 2 explains why.

Tray Z: from NMS logs (`avg_Z=...`), set `TRAY_Z_MIN/MAX` to the floor band.

---

## 7. Run the full pipeline (one command)

**Host:** Sim running.  
**Container:**

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh
# then:
bash ${ISAAC_ROS_WS}/scripts/run_phase1.sh
```

This starts:

1. Isaac ROS SAM (`sim_d455_interface_specs.json`)  
2. `prompt_input_type:=bbox`  
3. `tray_grid_prompts.py`  
4. `masks_to_pointclouds_nms.py`  
5. visualize + rqt  

Ctrl+C stops all.

```bash
USE_RQT=0 bash ${ISAAC_ROS_WS}/scripts/run_phase1.sh
```

### Check clouds (host)

```bash
ls -lh ${ISAAC_ROS_WS}/output/clouds_nms/ | tail
```

Open a `.ply` in MeshLab / CloudCompare / Open3D.

---

## 8. Manual multi-terminal alternative

Same as [Week 3 §5](./TUTORIAL_Phase1_Week3_PointClouds_Continuation.md) (SAM / grid / NMS / viz in four terminals). Scripts already live under `${ISAAC_ROS_WS}/scripts/` after clone.

---

## 9. Optional rosbag

```bash
# Host
source /opt/ros/humble/setup.bash
mkdir -p ${ISAAC_ROS_WS}/bags
ros2 bag record -o ${ISAAC_ROS_WS}/bags/tray_d455_$(date +%Y%m%d) \
  /camera/realsense/rgb /camera/realsense/depth /camera/realsense/camera_info
```

Replay instead of Sim when testing perception only (`ros2 bag play -l ...`).

---

## 10. Success criteria

- Masks on tray objects in rqt  
- About one `.ply` per object under `output/clouds_nms/`  
- No tray-floor-only clouds  

→ Phase 1 complete.

**Next:** [Phase 2 — Grasp generation](./TUTORIAL_Phase2_Grasp_Generation.md) (Contact-GraspNet on the full scene, 2F-85 width cap). After you pull Phase 2, `run_phase1.sh` uses the **front-camera** prompt + planar cloud scripts — see that tutorial §0.

---

## 11. Where to read more

| Topic | Doc |
|--------|-----|
| Docker / ONNX / first SAM masks | [Week 1](./TUTORIAL_Phase1_Week1_Isaac_ROS_SAM.md) |
| ROI, grid, bbox size, flat tray | [Week 2](./TUTORIAL_Phase1_Week2_Grid_and_Masks.md) |
| Continuation without re-cloning | [Week 3](./TUTORIAL_Phase1_Week3_PointClouds_Continuation.md) |
| Grasps (after this tutorial) | [Phase 2](./TUTORIAL_Phase2_Grasp_Generation.md) |
