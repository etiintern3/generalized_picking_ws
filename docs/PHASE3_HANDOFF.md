# Phase 3 handoff — context for a new agent

**Date:** 2026-10-08  
**Repo:** https://github.com/etiintern3/generalized_picking_ws  
**Local workspace:** `/home/satwik/generalized_picking_ws` (`ISAAC_ROS_WS`)  
**Do not modify:** `/home/satwik/ur_ws` unless the user **explicitly** asks (working MoveIt pick-place + joint remapper)

Feed this file to a new Cursor agent before starting Phase 3. Also skim:

- [`PHASE2_README.md`](./PHASE2_README.md)  
- [`TUTORIAL_Phase2_Grasp_Generation.md`](./TUTORIAL_Phase2_Grasp_Generation.md)  

User preference: **guide with commands** they run themselves unless they ask you to edit. Prefer **new files** over overwriting working scripts.

---

## 1. Project goal (big picture)

Yamaha / UR10e + Robotiq 2F-85 in Isaac Sim → generalized picking of **novel objects** (no CAD, no per-object training).

| Phase | Status |
|-------|--------|
| 0 Baseline pick-place (known poses) | **Done** (`ur_ws`) |
| 1 Class-agnostic perception → object clouds | **Done** |
| 2 Grasp pose generation (Contact-GraspNet) | **Done** — tested on varied YCB-style objects; working well |
| **3 Motion planning** | **NEXT** — TF → IK → collision → staged trajectories |
| 4 Grasp robustness | Later |
| 5 Domain randomization | Later |
| 6 Real robot | Later |

**Phase 3 target:** take a ranked grasp from Phase 2 (`T_cam` in camera optical frame), transform it into the robot / MoveIt planning frame, solve IK with collision awareness, and execute a **staged** pick (pre-grasp → grasp → retreat). Sim still **mirrors** MoveIt — not dual-arm.

---

## 2. What “mirroring” means (control — leave alone)

- MoveIt (Humble) plans; publishes joint states  
- Remapper: MoveIt gripper joint names → Isaac Sim names  
- Isaac Sim robot listens on `/joint_command_isaac`  
- This is **not** dual-arm  

**Read-only reference** (known-pose cube pick, fixed downward gripper):  
`/home/satwik/ur_ws/src/pick_place_scripts/pick_place_scripts/tutorial5_full.py`

Notes from that script (do not assume Phase 3 can reuse blindly):

- Plans `tool0` goals as `PoseStamped` with `header.frame_id = "world"`  
- Uses a **fixed** `GRIPPER_ORIENTATION` (top-down). Phase 2 grasps are **full 6-DoF** from CGN — that quaternion must **not** be reused as-is  
- Groups: `ur_manipulator`, `gripper`  
- Gripper open = named pose `"open"`; close = direct action `/robotiq_gripper_controller/gripper_cmd`  
- Collision objects added in MoveIt planning scene (floor, cube)

Phase 3 should **consume ranked grasps**, not rewrite perception or CGN.

---

## 3. What Phase 2 delivers (input to Phase 3)

### Daily perception + grasp run (working)

1. **Host:** Isaac Sim 5.1, front RGB-D on `/camera/realsense/{rgb,depth,camera_info}` — 1280×720, depth `32FC1`, frame usually `sim_camera`  
2. **Isaac ROS:** `bash ${ISAAC_ROS_WS}/scripts/run_phase1.sh` — SAM1 + front-cam bbox grid + planar object PLYs  
3. **Isaac ROS:** `bash ${ISAAC_ROS_WS}/scripts/run_phase2.sh dump` → `output/cgn_scenes/scene_*.npz`  
4. **Host:** `bash ${ISAAC_ROS_WS}/scripts/run_phase2.sh` → CGN in `jp_cgnet` Docker + rank  
5. **Outputs:** `output/grasps/scene_*_ranked.json`, `*_preview.png`; optional Open3D viz script  

Docs: [`PHASE2_README.md`](./PHASE2_README.md), [`TUTORIAL_Phase2_Grasp_Generation.md`](./TUTORIAL_Phase2_Grasp_Generation.md).

### Ranked JSON (what to read)

Path pattern: `output/grasps/scene_<stamp>_ranked.json`

Per object: `object_id`, `n_raw`, `n_kept`, `grasps[]` sorted by score. Each grasp has:

| Field | Meaning |
|-------|---------|
| `score` | CGN confidence |
| `width_m` | Gripper opening (m), capped ≤ 0.085 |
| `contact_cam` | Contact point in camera frame |
| `approach_cam` | Unit approach (= column **Z** of `T_cam`) |
| `T_cam` | 4×4 grasp pose in **camera optical** frame |

**Frame:** OpenCV optical — x right, y down, z forward.  
**CGN convention:** `T_cam` is the gripper TCP pose used by Contact-GraspNet (Panda-style), **not** necessarily identical to UR `tool0` or `robotiq_85_base_link`. Contact often sits a few cm along +Z from the TCP.

**Ranking filters (current defaults):** min score 0.10, max width 0.085 m, min contact height above fitted table 0.008 m. **`into_table` approach filter is OFF** (front-cam top grasps look “into” the table and were wrongly rejected).

No need to clear `output/cgn_scenes` / `output/grasps` between scenes — new stamps are created; grasp step uses the **latest** scene unless `SCENE=` is set.

---

## 4. Current stack snapshot

| Item | Detail |
|------|--------|
| OS / ROS | Ubuntu 22.04, ROS 2 **Humble** |
| Isaac Sim | **5.1** at `~/isaac-sim` |
| Isaac ROS | Dev Docker, `isaac_ros_common` **release-3.2** |
| SAM | Isaac ROS **SAM1**, `prompt_input_type:=bbox`, stamped prompts |
| Front-cam prompts | `scripts/frontcam_table_grid_prompts.py` — ROI ~`(220,1000)×(500,720)`, grid **6×6**, boxes 100×100 (tuned; may need retune per scene) |
| Planar PLYs | `scripts/masks_to_pointclouds_planar.py` → `output/clouds_planar/` (optional; not CGN input) |
| CGN | `third_party/cgnet` (gitignored), Docker **`jp_cgnet`**, ckpt `scene_test_2048_bs3_hor_sigma_001` |
| Rank / viz | Host **`/usr/bin/python3`** — not `uois3d` |
| GPU | RTX 3090 24GB |
| GitHub `main` | Phase 1 + Phase 2 scripts and tutorials |

**Envs (keep separate):** Isaac ROS = SAM/dump · `jp_cgnet` = CGN · host system Python = rank/viz · MoveIt lives in `ur_ws`.

---

## 5. Dead ends (do not retry unless asked)

- **AnyGrasp** — skipped (license)  
- **UOIS** — abandoned; leave `third_party/uois` / conda `uois3d` alone  
- **Isolated object PLYs as main CGN input** — worse than full scene + segmap  
- Old **top-down** `tray_grid_prompts.py` together with front-cam prompts (both publish `/prompts`)  
- Reusing Phase 0 **fixed top-down** gripper quaternion for CGN grasps  

---

## 6. Phase 3 scope and success criteria

### In scope

1. **TF:** reliable transform camera optical (`sim_camera` or whatever `camera_info.frame_id` is) → planning frame (`base_link` and/or `world` — verify both; Phase 0 uses `world`)  
2. **Pose bridge:** `T_base = T_base_cam @ T_cam` (or world), then map CGN TCP → MoveIt `tool0` / gripper frame if offsets differ  
3. **IK** for grasp (and pre-grasp) with MoveIt — collision-aware  
4. **Staged motion:** pre-grasp (offset along **−approach**) → grasp → close gripper → retreat (often +Z world / clear of table)  
5. **Try next grasp** in the ranked list if IK or planning fails  
6. Prefer **new scripts** under `generalized_picking_ws` or a clearly named new file; touch `ur_ws` only if the user asks  

### Out of scope until user asks

- Rewriting SAM / CGN / ranking  
- Real-robot bringup  
- Full robust multi-object pick-place productization (Phase 4+)  

### Done when

- Top (or next-feasible) grasp from ranked JSON → robot approaches and closes at a plausible pose in Sim without obvious table/robot collision  
- Pre-grasp → grasp → retreat sequence works for at least one YCB-style object  
- Failures are understandable (IK fail → try next grasp), not silent  

---

## 7. Suggested first actions (do not jump to execution)

1. Read this handoff + Phase 2 tutorial (how JSON is produced).  
2. With Sim + robot TF up (existing MoveIt/Sim stack), confirm frames:

   ```bash
   ros2 topic echo /camera/realsense/camera_info --once | grep frame_id
   ros2 run tf2_ros tf2_echo base_link sim_camera
   ros2 run tf2_ros tf2_echo world sim_camera
   ```

3. Take the best grasp `T_cam` from a recent `output/grasps/*_ranked.json` and compute `T_world` / `T_base` offline; sanity-check position (on table in front of UR) and approach (not through the floor).  
4. Resolve **CGN TCP ↔ `tool0`** (2F-85 vs Panda mesh offset) before trusting IK.  
5. Only then: MoveIt IK + staged plan; prefer a **new** pick script inspired by `tutorial5_full.py`, not a silent rewrite of the working cube demo.  
6. **Do not wire execution** until the user asks; start with TF + transform validation if they want a careful ramp.

---

## 8. Watch-outs

- Front-cam ROI / grid is **scene-specific**; retune if the table crop moves.  
- Planarity filter drops table patches (`plane_RMS < 0.004 m`); some flat objects can be over-filtered.  
- CGN Docker needs GPU; do not run CGN inside Isaac ROS.  
- `output/` and `third_party/cgnet` are **not** in git.  
- Host `python3` may be Miniconda without numpy — use `/usr/bin/python3` for rank/viz.  
- Phase 0 heights (`PRE_GRASP_HEIGHT`, `GRASP_HEIGHT`) are for a known cube, not CGN poses.  

---

## 9. Key paths

| Path | Role |
|------|------|
| `docs/PHASE3_HANDOFF.md` | This file |
| `docs/PHASE2_README.md` | Phase 2 intro |
| `docs/TUTORIAL_Phase2_Grasp_Generation.md` | How to run grasps |
| `scripts/run_phase1.sh` / `run_phase2.sh` | Bringup |
| `output/grasps/*_ranked.json` | Phase 3 input |
| `/home/satwik/ur_ws/.../tutorial5_full.py` | MoveIt pick reference (read-only unless asked) |

---

## 10. User working style

- Prefers commands they run themselves; ask before large edits.  
- Prefer new scripts over breaking working Phase 1/2 paths.  
- Leave `ur_ws` alone unless explicitly requested.  
- Do not restart dead ends (UOIS, AnyGrasp, isolated-PLY CGN) unless asked.  
