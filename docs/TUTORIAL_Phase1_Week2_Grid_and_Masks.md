# Tutorial — Phase 1 Week 2 (so far): Tray ROI, Grid Prompts, Mask Quality

**Project:** Generalized picking (UR10e + Robotiq 2F-85)  
**Workspace:** `/home/satwik/generalized_picking_ws`  
**Follows:** [TUTORIAL_Phase1_Week1_Isaac_ROS_SAM.md](./TUTORIAL_Phase1_Week1_Isaac_ROS_SAM.md)

**Goal of this tutorial:** Document everything done after the Week 1 smoke test — live tray ROI, grid prompting, SAM parameter lessons, tray geometry, and current best settings.  
**Not done yet in this doc:** mask filtering code + depth → point clouds (next).

**Do not modify** `~/ur_ws`.


---

## 1. Daily restart (host + container)

### Open / attach Dev container

```bash
source ~/.bashrc
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh
```

Prompt should be:

```text
admin@...:/workspaces/isaac_ros-dev$
```

Then:

```bash
source /opt/ros/humble/setup.bash
```

### If packages are “not found” after a fresh container

`run_dev.sh` uses a disposable container; `apt` installs can vanish when it fully stops. Reinstall:

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-isaac-ros-examples ros-humble-isaac-ros-segment-anything
pip3 install --break-system-packages 'numpy<2'
source /opt/ros/humble/setup.bash
```

ONNX/assets under `${ISAAC_ROS_WS}` stay on the host mount and do **not** need re-export.

### Launch SAM on Sim camera

Isaac Sim running (host) with camera publishing, then in container:

```bash
cd ${ISAAC_ROS_WS}
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py \
  launch_fragments:=segment_anything \
  interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_segment_anything/sim_d455_interface_specs.json \
  sam_model_repository_paths:=[${ISAAC_ROS_WS}/isaac_ros_assets/models]
```

Set encoder mode (important — see §4):

```bash
ros2 param set /sam_data_encoder_node prompt_input_type bbox
ros2 param get /sam_data_encoder_node orig_img_dims
# expect: [720, 1280]
```

Visualize:

```bash
ros2 run isaac_ros_segment_anything visualize_mask.py --ros-args \
  --remap /yolov8_encoder/resize/image:=/camera/realsense/rgb

ros2 run rqt_image_view rqt_image_view /segment_anything/colored_segmentation_mask
```

**Topics to trust:**

| Topic | Use |
|--------|-----|
| `/camera/realsense/rgb` | True Sim view |
| `/segment_anything/resized_image` | What SAM sees |
| `/segment_anything/colored_segmentation_mask` | Overlay to judge masks |

**Ignore for judging quality:** `/segment_anything/color_converted_image` (often half-black / padded intermediate — not a broken camera).

---

## 2. Measure tray ROI (green overlay)

The ROI probe that only publishes SAM prompts does **not** draw a box on the image. Use an overlay topic instead.

```bash
source /opt/ros/humble/setup.bash
# if needed: sudo apt-get install -y ros-humble-cv-bridge

python3 - <<'PY'
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# EDIT and re-run until green box = tray INTERIOR only
X_MIN, X_MAX = 535, 730
Y_MIN, Y_MAX = 215, 490

rclpy.init()
node = Node('roi_overlay')
bridge = CvBridge()
pub = node.create_publisher(Image, '/debug/roi_overlay', 10)

def on_image(msg: Image):
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    h, w = img.shape[:2]
    x0, x1 = max(0, min(w - 1, X_MIN)), max(0, min(w - 1, X_MAX))
    y0, y1 = max(0, min(h - 1, Y_MIN)), max(0, min(h - 1, Y_MAX))
    out = img.copy()
    cv2.rectangle(out, (x0, y0), (x1, y1), (0, 255, 0), 3)
    cv2.putText(out, f'ROI {x0},{y0}->{x1},{y1}',
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    pub.publish(bridge.cv2_to_imgmsg(out, encoding='rgb8'))

node.create_subscription(Image, '/camera/realsense/rgb', on_image, 10)
print('View /debug/roi_overlay in rqt. Ctrl+C to stop.')
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
node.destroy_node(); rclpy.shutdown()
PY
```

```bash
ros2 run rqt_image_view rqt_image_view /debug/roi_overlay
```

Image coords: origin **top-left**, x right, y down; full frame **1280×720**.

### Locked ROI (flat tray, interior only — no rim)

```text
X_MIN, X_MAX = 535, 730
Y_MIN, Y_MAX = 215, 490
```

(Earlier mesh-tray ROI was `525–750` / `180–530`; re-measure after any tray change.)

---

## 3. Grid prompts (class-agnostic)

Idea: sprinkle **bounding-box prompts** over the tray ROI (Isaac ROS SAM is prompt-based; this is the ROS version of “auto” grid prompting).

**Critical:** every prompt must use the **same `header.stamp` as the RGB image** (Week 1 lesson).

---

## 4. Lessons learned — why early grids looked bad

### 4.1 Tiny masks / points on corners

Symptoms:

- Colored blobs only on corners or small patches of boxes  
- Grid with `POINT_BOX = 20` and default `prompt_input_type:=bbox`

Cause: Isaac ROS treated prompts as **tiny bboxes**, so SAM returned tiny regions.

```bash
ros2 param get /sam_data_encoder_node prompt_input_type
# was: bbox
```

Trying `point` mode:

```bash
ros2 param set /sam_data_encoder_node prompt_input_type point
```

did **not** reliably give full-object masks on this stack.

**What worked:** keep **`bbox`**, use **large** boxes that cover most of an object (e.g. ~120×80 or ~160×110).

### 4.2 `orig_img_dims`

Already correct for our camera:

```bash
ros2 param get /sam_data_encoder_node orig_img_dims
# Integer values are: array('q', [720, 1280])   # [H, W]
```

Do **not** set floats (`[720.0,1280.0]`) — type must stay integer array.

### 4.3 Wire-mesh tray

Mesh bottom caused **speckled / striped** masks (SAM following the grid).  

**Fix:** flat opaque tray bottom (or plain table). Flat tray made solid object masks possible.

### 4.4 Partial / holed masks with flat tray

With 3 boxes: some full, some half, some with holes — usually **bbox not covering the whole object** or competing grid prompts.

**Fix that worked:** coarser grid + larger boxes, e.g. `NX,NY = 2,3` and `BOX_W,BOX_H = 160,110` over the locked ROI → **all 3 boxes covered**.

### 4.5 Overlaps and tray bleed

When objects overlap, masks are often good; **sometimes** a mask grows to include tray floor.

| Impact later | |
|--------------|--|
| Point cloud | Includes tray points |
| Grasp net | May grasp tray / bad geometry |

**Guidance:** occasional bleed is expected; handle with **filtering + reject/retry** (next steps). Prefer slightly separated objects for the first point-cloud milestone. Not a reason to stop Phase 1.

---

## 5. Current best grid script (working)

Prereqs: Sim + SAM launch + `prompt_input_type:=bbox`.

```bash
source /opt/ros/humble/setup.bash
ros2 param set /sam_data_encoder_node prompt_input_type bbox

python3 - <<'PY'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

X_MIN, X_MAX = 535, 730
Y_MIN, Y_MAX = 215, 490

NX, NY = 2, 3
BOX_W, BOX_H = 160.0, 110.0

CELL_W = (X_MAX - X_MIN) / NX
CELL_H = (Y_MAX - Y_MIN) / NY
xs = [X_MIN + (i + 0.5) * CELL_W for i in range(NX)]
ys = [Y_MIN + (j + 0.5) * CELL_H for j in range(NY)]
centers = [(x, y) for y in ys for x in xs]

rclpy.init()
node = Node('tray_grid_large_bbox')
pub = node.create_publisher(Detection2DArray, '/prompts', 10)

def on_image(img: Image):
    msg = Detection2DArray()
    msg.header = img.header
    for (x, y) in centers:
        det = Detection2D()
        det.header = img.header
        det.bbox.center.position.x = float(x)
        det.bbox.center.position.y = float(y)
        det.bbox.size_x = BOX_W
        det.bbox.size_y = BOX_H
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = 'grid'
        hyp.hypothesis.score = 1.0
        det.results.append(hyp)
        msg.detections.append(det)
    pub.publish(msg)

node.create_subscription(Image, '/camera/realsense/rgb', on_image, 10)
print(f'ROI ({X_MIN},{Y_MIN})-({X_MAX},{Y_MAX})  {len(centers)} boxes {BOX_W}x{BOX_H}')
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
node.destroy_node(); rclpy.shutdown()
PY
```

**Success criterion (achieved):** three objects in the flat tray get full colored masks on `/segment_anything/colored_segmentation_mask`.

---

## 6. Settings cheat sheet (as of this tutorial)

| Item | Value |
|------|--------|
| Image | `/camera/realsense/rgb` · 1280×720 · rgb8 |
| Depth (next) | `/camera/realsense/depth` |
| Camera info | `/camera/realsense/camera_info` |
| Prompts | `/prompts` · `vision_msgs/Detection2DArray` · **stamped** |
| `prompt_input_type` | **`bbox`** (not tiny point-boxes) |
| `orig_img_dims` | `[720, 1280]` |
| Tray ROI | `(535,215)–(730,490)` interior |
| Grid | `2×3`, box `160×110` |
| Tray hardware | **Flat bottom** (not wire mesh) |

---

## 7. What’s next (not in this tutorial yet)

1. **Week 2.4 — Mask filtering**  
   Drop huge tray blobs, tiny noise, duplicates; optional depth above tray floor.  

2. **Week 2.5 — Point clouds**  
   Masked depth + intrinsics → one cloud per object; visualize/save PLY.  

3. **Week 3 — Novel objects milestone**  
   Then Phase 2 (grasp nets).

---

## 8. Quick “does it still work?” checklist

1. Host: Isaac Sim scene + camera topics.  
2. `./scripts/run_dev.sh` → `source /opt/ros/humble/setup.bash`  
3. Reinstall examples/SAM apt packages if missing.  
4. Launch SAM + `prompt_input_type:=bbox`  
5. Run large-bbox grid script with locked ROI  
6. `visualize_mask` + rqt on **colored_segmentation_mask**  
7. Confirm objects fully covered  

---

*Documented from Phase 1 Week 2 guided sessions (ROI overlay, grid, bbox sizing, flat tray, overlap notes).*
