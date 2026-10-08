# Phase 2 handoff — context for a new agent

**Date:** 2026-09-30  
**Repo:** https://github.com/etiintern3/generalized_picking_ws  
**Local workspace:** `/home/satwik/generalized_picking_ws` (`ISAAC_ROS_WS`)  
**Do not modify:** `/home/satwik/ur_ws` (working MoveIt pick-place + joint remapper)

Feed this file (and optionally `docs/PHASE1_README.md`) to a new Cursor agent before starting Phase 2.

---

## 1. Project goal (big picture)

Yamaha / UR10e + Robotiq 2F-85 in Isaac Sim → generalized picking of **novel objects** (no CAD, no per-object training).

Roadmap phases:

| Phase | Status |
|-------|--------|
| 0 Baseline pick-place (known poses) | Done (in `ur_ws`) |
| **1 Class-agnostic perception → object point clouds** | **Done** |
| **2 Grasp pose generation** | **Done** — see [`TUTORIAL_Phase2_Grasp_Generation.md`](./TUTORIAL_Phase2_Grasp_Generation.md) |
| **3 Motion planning** | **NEXT** — see [`PHASE3_HANDOFF.md`](./PHASE3_HANDOFF.md) |
| 4 Grasp robustness | Later |
| 5 Domain randomization | Later |
| 6 Real robot | Later |

Phase 2 target (from original roadmap): feed each object’s point cloud into a **pretrained** grasp network (Contact-GraspNet / AnyGrasp / GraspNet-1B), get ranked 6-DoF grasps, filter by **2F-85 max opening 85 mm**.

---

## 2. What “mirroring” means (control — leave alone)

- MoveIt (Humble) plans; publishes joint states  
- Remapper: MoveIt gripper joint names → Isaac Sim names  
- Isaac Sim robot listens on `/joint_command_isaac`  
- This is **not** dual-arm; it is Sim mirroring MoveIt  

Working pick script (read-only reference):  
`/home/satwik/ur_ws/src/pick_place_scripts/pick_place_scripts/tutorial5_full.py`

Phase 2 should consume **point clouds from Phase 1**, not rewrite control.

---

## 3. Phase 1 stack (what already works)

| Item | Detail |
|------|--------|
| OS / ROS | Ubuntu 22.04, ROS 2 **Humble** |
| Isaac Sim | **5.1.0** at `~/isaac-sim`, start: `./isaac-sim.selector.sh` |
| Isaac ROS | Dev Docker via `isaac_ros_common` **release-3.2** |
| SAM | Isaac ROS **SAM1** (`isaac_ros_segment_anything`), not SAM2/SAM3 |
| Camera topics | `/camera/realsense/rgb`, `/depth` (**32FC1**), `/camera_info` — 1280×720 |
| Prompts | `/prompts` (`Detection2DArray`), **must share image header stamp**; `prompt_input_type:=bbox` |
| Grid | `scripts/tray_grid_prompts.py` — ROI ~`(540,215)–(705,475)`, 5×5, boxes ~70×70 |
| Clouds | `scripts/masks_to_pointclouds_nms.py` → `output/clouds_nms/*.ply` |
| Bringup | `bash ${ISAAC_ROS_WS}/scripts/run_phase1.sh` (inside container) |
| GPU | RTX 3090 24GB |

**Container:**  
`cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh`  
Workspace mounts as `/workspaces/isaac_ros-dev`.

**Persistent packages:** `scripts/enable_persistent_isaac_ros.sh` + `docker/Dockerfile.perception` (optional). Bootstrap: `scripts/bootstrap_isaac_ros_packages.sh`.

**Not in git:** ONNX weights under `isaac_ros_assets/models/`, `vit_b.pth`, bags, `output/`, nested `src/isaac_ros_common`.

---

## 4. Phase 1 docs (read if needed)

| Doc | Role |
|-----|------|
| `docs/PHASE1_README.md` | Intro / which tutorial track |
| `docs/TUTORIAL_Phase1_Week1_*.md` | Docker, SAM ONNX, first masks |
| `docs/TUTORIAL_Phase1_Week2_*.md` | ROI, grid, bbox lessons |
| `docs/TUTORIAL_Phase1_Week3_*.md` | Point clouds continuation |
| `docs/TUTORIAL_Phase1_Standalone_From_GitHub.md` | Clone path for new machines |

User preference: **guide with commands**; they often run steps themselves. Prefer not to overwrite working scripts blindly; new files OK (e.g. we added `*_nms.py` instead of editing the first cloud script).

---

## 5. Phase 2 — status (complete)

**Done:** Contact-GraspNet on **full scene** RGB-D + SAM object `segmap` (`--local_regions --filter_grasps`). Opening capped at **0.085 m** via `DATA.gripper_width:0.085`. Ranked JSON via `scripts/rank_cgn_grasps.py`.

**How to run:** [`TUTORIAL_Phase2_Grasp_Generation.md`](./TUTORIAL_Phase2_Grasp_Generation.md)

**Out (achieved):**

1. Backend: **Contact-GraspNet** (AnyGrasp skipped — license).  
2. Scene `npz` from `scripts/sam_scene_to_cgn.py` (not isolated PLYs for the main path).  
3. Width ≤ **0.085 m** at generation + rank script.  
4. Open3D during inference; `output/grasps/*_preview.png` after ranking.  
5. Ranked per-object list in camera frame.  
6. **MoveIt not wired** (Phase 3).

**Front camera:** `scripts/frontcam_table_grid_prompts.py` ROI `(200,500)–(1050,720)`. Planarity: `scripts/masks_to_pointclouds_planar.py`.

**Rank env:** host `/usr/bin/python3` (numpy + matplotlib). **Not** `uois3d` — UOIS was tried and abandoned; leave `third_party/uois` alone.

---

## 6. Watch-outs for Phase 3

- [x] Contact-GraspNet vs AnyGrasp → CGN, Docker `jp_cgnet`.  
- [x] Grasp net **not** in Isaac ROS container.  
- [x] Input: scene `rgb/depth/K/segmap`.  
- [x] Rank with system Python — not `uois3d`.  
- [ ] Camera optical → `base_link` / `tool0`.  
- [ ] Front-cam tray ROI is scene-specific.

---

## 7. Suggested first actions for Phase 3

Use the dedicated handoff: [`PHASE3_HANDOFF.md`](./PHASE3_HANDOFF.md).
