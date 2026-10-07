# Phase 2 — Grasp Generation (Introduction)

Read this **before** the Phase 2 how-to tutorial.

**Assumes:** you finished [Phase 1 Standalone](./TUTORIAL_Phase1_Standalone_From_GitHub.md) (or the Week 1→3 path) on this machine — Isaac Sim camera + Isaac ROS SAM1 already work.

---

## What Phase 2 is

**Goal:** From a live RGB-D view of the table, produce a **ranked list of 6-DoF grasp poses** for novel objects — suitable for a Robotiq **2F-85** (max opening **85 mm**) — with **no CAD models** and **no per-object training**.

**How:** pretrained **Contact-GraspNet (CGN)** on a **full scene** dump (table + objects in depth, plus a SAM object `segmap`), then filter/rank for the 2F-85.

**Hardware / sim context**

- Same cell: UR10e + Robotiq 2F-85 (control stays in `~/ur_ws`; Phase 2 does not change MoveIt)
- Isaac Sim 5.1 with a **front-facing** RealSense-style RGB-D camera (not the old top-down tray-only view)
- Perception still Isaac ROS **SAM1**; grasps run in a **separate** Docker (`jp_cgnet`)

**Why this approach**

Isolated object point clouds alone were a weaker CGN input. The network does better when it sees the **whole depth scene** and uses the **segmap** (`--local_regions --filter_grasps`) so contacts stay on objects. Opening is capped at **0.085 m** for the 2F-85.

**What Phase 2 is not**

- No MoveIt, IK, collision checking, or pick execution (Phase 3)
- No training or fine-tuning of CGN / SAM — pretrained weights only
- Not AnyGrasp (license) and not UOIS (tried; abandoned)

**Done when:** for objects on the table you care about, you have `output/grasps/*_ranked.json` with non-empty `grasps` lists (`T_cam`, score, width), a preview image, and (optional) Open3D viz of grasps on the scene cloud.

---

## Pipeline (mental model)

```text
Isaac Sim front RGB-D
        ↓
Isaac ROS SAM1  ←  front-cam bbox grid on the table crop
        ↓
Full scene dump  (rgb + depth + K + object segmap)
        ↓
Contact-GraspNet  (jp_cgnet Docker, 85 mm gripper cap)
        ↓
Rank / filter  (score, width, table height)
        ↓
Ranked grasps in camera frame  + preview / 3D viz
```

Poses are in the **camera optical** frame (OpenCV: x right, y down, z forward). Transform to `base_link` / `world` is Phase 3.

---

## Envs (keep separate)

| Job | Where |
|-----|--------|
| SAM + scene dump | Isaac ROS Dev container |
| Contact-GraspNet | Docker image / container `jp_cgnet` |
| Rank + Open3D viz | Host `/usr/bin/python3` |

Do **not** put CGN inside the Isaac ROS container. Do **not** use the leftover `uois3d` conda env for ranking.

---

## Front camera vs Phase 1 Standalone docs

Phase 1 Standalone was written around a **top-down tray**. The live cell now uses a **front** camera. After you pull current `main`:

| Role | Script |
|------|--------|
| SAM prompts | `frontcam_table_grid_prompts.py` |
| Optional object PLYs | `masks_to_pointclouds_planar.py` (planarity, not avg‑Z) |
| Phase 1 bringup | `run_phase1.sh` (starts the two above) |
| Scene for CGN | `sam_scene_to_cgn.py` / `run_phase2.sh dump` |

Do **not** also run old `tray_grid_prompts.py` (both publish `/prompts`).

---

## How to follow Phase 2

There is **one** tutorial track:

1. Read **this** intro  
2. Follow **[Phase 2 — Grasp generation](./TUTORIAL_Phase2_Grasp_Generation.md)**  
   - pull `main`  
   - install CGN under `third_party/cgnet` (not in git)  
   - `run_phase1.sh` → `run_phase2.sh dump` → `run_phase2.sh`  
   - inspect JSON / preview / `visualize_ranked_grasps.py`

Agent / roadmap notes (optional): [`PHASE2_HANDOFF.md`](./PHASE2_HANDOFF.md).

---

## Suggested order

```text
Phase 1 done (Standalone or Week 3)
    ↓
Read this Phase 2 intro
    ↓
TUTORIAL_Phase2_Grasp_Generation.md
    ↓
Phase 2 done (ranked grasps in camera frame)
    ↓
Phase 3 — TF → base, IK, trajectories
```

---

## Repo & workspace

- **GitHub:** https://github.com/etiintern3/generalized_picking_ws  
- **Local path:** `~/generalized_picking_ws` (`ISAAC_ROS_WS`)  
- **Not in git:** `third_party/cgnet` (clone + checkpoints), `output/`, SAM ONNX / weights, `src/isaac_ros_common`

Daily Phase 2 (after CGN install), with Sim + SAM up:

```bash
# Isaac ROS — dump one scene
bash ${ISAAC_ROS_WS}/scripts/run_phase2.sh dump

# Host — CGN + rank
bash ${ISAAC_ROS_WS}/scripts/run_phase2.sh
```

---

## After Phase 2

**Phase 3:** take top `T_cam` from the ranked JSON, get TF camera → `base_link` / `world` in Sim, then IK, collision, and staged approach. Leave `~/ur_ws` alone until you explicitly wire execution.
