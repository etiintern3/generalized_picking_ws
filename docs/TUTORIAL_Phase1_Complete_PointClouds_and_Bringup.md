# Tutorial — Phase 1 Complete: Point Clouds, Filters, One-Command Bringup

**Project:** Generalized picking (UR10e + Robotiq 2F-85)  
**Workspace:** `/home/satwik/generalized_picking_ws`  

**Previous tutorials:**
1. [Week 1 — Isaac ROS SAM setup](./TUTORIAL_Phase1_Week1_Isaac_ROS_SAM.md)  
2. [Week 2 — Tray ROI, grid prompts, mask quality](./TUTORIAL_Phase1_Week2_Grid_and_Masks.md)  

**This tutorial:** Everything after that — depth → point clouds, NMS / tray-Z filters, rosbag backup, persistent container packages, and **one-command Phase 1**.

**Do not modify** `~/ur_ws`.

---

## 0. Phase 1 end state (achieved)

Given novel objects in the flat tray under the Sim D455:

1. Isaac ROS SAM1 produces masks (bbox grid prompts).  
2. Filters keep object masks (ROI, area, IoU NMS, tray depth).  
3. Depth + intrinsics → **one point cloud per object** (`.ply`).  
4. Floor-only clouds are dropped.  

**Definition of done:** point clouds of **objects only** — Phase 1 complete for going on to Phase 2 (grasps).

---

## 1. Record a rosbag (backup when Sim breaks)

On the **host**, with Sim publishing camera topics:

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/generalized_picking_ws/bags

ros2 bag record -o ~/generalized_picking_ws/bags/tray_d455_$(date +%Y%m%d) \
  /camera/realsense/rgb \
  /camera/realsense/depth \
  /camera/realsense/camera_info
# Ctrl+C after ~30–60 s
```

Inspect / play with **ROS 2** (not ROS 1 `rosbag`):

```bash
ros2 bag info ~/generalized_picking_ws/bags/tray_d455_*
ros2 bag play -l ~/generalized_picking_ws/bags/tray_d455_YYYYMMDD
```

Depth format used in Sim: **32FC1**, 1280×720 (meters).

---

## 2. Raw mask format (Isaac ROS)

```bash
ros2 topic info /segment_anything/raw_segmentation_mask -v
# Type: isaac_ros_tensor_list_interfaces/msg/TensorList
```

Typical tensor:

```text
shape [N, 1, 720, 1280], uint8 (1 byte/pixel)
```

Example: `N=6` masks from the grid batch.

---

## 3. First point-cloud script (no NMS)

`scripts/masks_to_pointclouds.py`

**Pipeline:**

```text
camera_info → K (fx, fy, cx, cy)
depth       → Z per pixel
raw masks   → N binary masks
     ↓
ROI crop + area filter
     ↓
back-project: X=(u-cx)*Z/fx, Y=(v-cy)*Z/fy
     ↓
save ASCII .ply every N frames
```

**Run inside container** (needs `isaac_ros_*` msgs):

```bash
source /opt/ros/humble/setup.bash
python3 ${ISAAC_ROS_WS}/scripts/masks_to_pointclouds.py
```

Output: `${ISAAC_ROS_WS}/output/clouds/`

View PLYs on the **host** (GUI):

```bash
ls /home/satwik/generalized_picking_ws/output/clouds/
# Open3D / MeshLab / CloudCompare on host paths
```

Inside the container, use `/workspaces/isaac_ros-dev/output/clouds/` (same files).

---

## 4. Duplicate clouds → NMS script

**Problem:** 3 objects in scene but 4 `.ply` files — overlapping grid bboxes → two masks on one object.

**Fix:** new file (did **not** edit the first script):

`scripts/masks_to_pointclouds_nms.py`

- Same ROI / area / back-projection  
- **2D IoU NMS:** if overlap ≥ `IOU_MERGE`, keep the **smaller** mask  
- Optional **tray Z filter:** drop cloud if `TRAY_Z_MIN ≤ avg_Z ≤ TRAY_Z_MAX` (floor distance)  
- Writes to `output/clouds_nms/`

```bash
python3 ${ISAAC_ROS_WS}/scripts/masks_to_pointclouds_nms.py
```

Log example:

```text
6 raw -> 4 area -> 3 NMS -> 0 tray-Z drop -> 3 clouds
```

### Tunable constants (edit the file)

| Param | Role |
|--------|------|
| `X_MIN/X_MAX/Y_MIN/Y_MAX` | Tray interior ROI |
| `MIN_AREA` / `MAX_AREA` | Drop tiny / huge masks |
| `IOU_MERGE` | Duplicate merge threshold |
| `TRAY_Z_MIN` / `TRAY_Z_MAX` | Drop floor-height clouds (e.g. 1.30–1.31 m) |
| `MIN_POINTS` | Min valid depth points |

### Grid box size lesson

Prompt boxes must be **much smaller than tray width**.  
Example: tray width ≈ 165 px → `BOX_W=160` covers both objects → **one merged mask**.  
Working grid used later: `NX,NY=5,5`, `BOX_W=BOX_H=70` (in `tray_grid_prompts.py`).

---

## 5. Live stats for all N masks (optional)

One-shot table (area, n_pts, avg_Z, keep?) — use the inspect snippet from the Week 2 sessions, or check logs from the NMS node (`cloud[j] area=... avg_Z=...`).

---

## 6. Stop reinstalling packages every container start

**Cause:** `run_dev.sh` uses `--rm` → container wiped on exit.

### Permanent fix (recommended)

On **host**, once:

```bash
bash ${ISAAC_ROS_WS}/scripts/enable_persistent_isaac_ros.sh
# writes ~/.isaac_ros_common-config → image key ros2_humble.perception

# Exit all container shells, then:
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh
```

Layer: `docker/Dockerfile.perception` installs:

- `ros-humble-isaac-ros-examples`  
- `ros-humble-isaac-ros-segment-anything`  
- `ros-humble-cv-bridge`  
- `python3-numpy` + `numpy<2`

### Temporary one-liner inside container

```bash
bash ${ISAAC_ROS_WS}/scripts/bootstrap_isaac_ros_packages.sh
```

---

## 7. One-command Phase 1 bringup

Instead of many terminals, **inside the container** (Sim already running):

```bash
source /opt/ros/humble/setup.bash   # optional; script sources ROS itself
bash ${ISAAC_ROS_WS}/scripts/run_phase1.sh
```

Starts:

1. SAM launch (`sim_d455_interface_specs.json`)  
2. `prompt_input_type:=bbox` (waits for encoder)  
3. `tray_grid_prompts.py`  
4. `masks_to_pointclouds_nms.py`  
5. `visualize_mask` + `rqt_image_view` (optional)

**Ctrl+C** stops all.

```bash
USE_RQT=0 bash ${ISAAC_ROS_WS}/scripts/run_phase1.sh
USE_VIZ=0 bash ${ISAAC_ROS_WS}/scripts/run_phase1.sh
```

### Related files

| File | Role |
|------|------|
| `scripts/run_phase1.sh` | Single entrypoint |
| `scripts/tray_grid_prompts.py` | Stamped bbox grid |
| `scripts/set_sam_bbox_mode.sh` | Wait + set bbox |
| `scripts/masks_to_pointclouds_nms.py` | Filter + NMS + tray-Z + PLY |
| `scripts/masks_to_pointclouds.py` | Earlier version (no NMS) |
| `isaac_ros_assets/.../sim_d455_interface_specs.json` | 1280×720 camera wiring |

**Note:** `run_phase1.sh` must `set +u` around `source /opt/ros/humble/setup.bash` (ROS setup uses unbound vars).

---

## 8. Daily restart checklist (Phase 1)

1. Host: Isaac Sim + scene (or `ros2 bag play -l ...`).  
2. Host: `cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh`  
3. Container:  
   ```bash
   bash ${ISAAC_ROS_WS}/scripts/run_phase1.sh
   ```  
4. Confirm colored masks in rqt; PLYs appear under `output/clouds_nms/`.  

If packages missing: bootstrap script or perception image rebuild (§6).

---

## 9. Current working settings (reference)

Tune in the script files if the tray/camera move.

**Grid (`tray_grid_prompts.py`):**

```text
ROI: (540,215)–(705,475)
NX, NY = 5, 5
BOX_W, BOX_H = 70, 70
prompt_input_type = bbox
```

**Clouds (`masks_to_pointclouds_nms.py`):**  
ROI / area / `IOU_MERGE` / `TRAY_Z_*` as currently edited in that file.

**Topics:**

```text
/camera/realsense/rgb
/camera/realsense/depth          # 32FC1
/camera/realsense/camera_info
/prompts
/segment_anything/raw_segmentation_mask
/segment_anything/colored_segmentation_mask
```

---

## 10. What’s next (Phase 2 — not this tutorial)

- Feed per-object PLY / point clouds into a pretrained grasp network (Contact-GraspNet / AnyGrasp).  
- Filter grasps by Robotiq 2F-85 max opening (85 mm).  
- Keep MoveIt / `ur_ws` separate until execution integration.

---

## 11. Manager-style summary of this stretch

- Generated object-only 3D point clouds from SAM + depth.  
- Added duplicate-mask merging and tray-depth filtering.  
- Recorded camera rosbag for offline work.  
- Made container packages persistent (custom Docker layer).  
- Reduced Phase 1 to a single bringup command.  
- **Phase 1 perception pipeline complete.**

---

*Documented to close Phase 1 before grasp generation.*
