# Phase 1 — Class-Agnostic Perception (Introduction)

Read this **before** the week-by-week tutorials.

---

## What Phase 1 is

**Goal:** Given a tray of objects the system has never been trained on, produce a **clean 3D point cloud per object** — no CAD models, no per-object YOLO training, no grasp or motion yet.

**Hardware / sim context**

- Robot cell: UR10e + Robotiq 2F-85 (control stays in `ur_ws`; Phase 1 does not change that)
- Sim: NVIDIA Isaac Sim 5.1 with a downward RealSense-style RGB-D camera on the tray
- Perception: Isaac ROS **SAM1** (Segment Anything) + depth back-projection

**Why this approach**

Earlier work used per-object detectors. That does not scale to novel parts. Phase 1 uses **class-agnostic** segmentation (shape in the image, not object identity), then turns masks + depth into geometry for later grasping.

**What Phase 1 is not**

- No grasp pose generation (Phase 2)
- No MoveIt / pick execution changes
- No SAM training or fine-tuning — pretrained weights only

**Done when:** objects in the tray → reliable **object-only** `.ply` point clouds (tray floor filtered out).

---

## Pipeline (mental model)

```text
RGB-D camera (Sim or bag)
        ↓
Isaac ROS SAM1  ←  bbox grid prompts over the tray (stamped with image time)
        ↓
Masks (per prompt / object)
        ↓
Filters (ROI, area, overlap NMS, tray depth)
        ↓
Back-project with depth + intrinsics
        ↓
One point cloud (.ply) per object
```

---

## How to follow the tutorials

There are **two tracks**. Pick one:

| Your situation | Start here |
|----------------|------------|
| New to this project, or new PC | **Standalone** tutorial (clone GitHub, full setup) |
| You already finished Week 1 and Week 2 on this machine | **Week 3 continuation** (no re-clone) |

The **Week 1 → Week 2 → Week 3** series is the lab path we used (learn by building step by step).  
The **Standalone** tutorial is the “get Phase 1 running from the repo” path; it still points back to Week 1–2 when you need deep install/debug detail.

---

## Tutorial map

### Step-by-step series (recommended learning path)

1. **[Week 1 — Isaac ROS SAM setup](./TUTORIAL_Phase1_Week1_Isaac_ROS_SAM.md)**  
   Docker Isaac ROS 3.2 (Humble), SAM ONNX, first masks (rosbag + Sim camera), stamped prompts.

2. **[Week 2 — Tray ROI & grid prompts](./TUTORIAL_Phase1_Week2_Grid_and_Masks.md)**  
   Flat tray, ROI overlay, bbox grid, mask quality (box size, mesh vs flat).

3. **[Week 3 — Point clouds (continuation)](./TUTORIAL_Phase1_Week3_PointClouds_Continuation.md)**  
   Depth → `.ply`, NMS / tray-Z filters, multi-terminal bringup, optional `run_phase1.sh`.  
   **For people who already have `generalized_picking_ws` from Weeks 1–2 — do not clone the repo over it.**

### Alternate: full pipeline from GitHub

4. **[Standalone — Phase 1 from GitHub](./TUTORIAL_Phase1_Standalone_From_GitHub.md)**  
   `git clone` → Isaac ROS + ONNX → Sim → one-command `run_phase1.sh`.  
   Use on a **new** folder/machine. Refers to Weeks 1–2 for SAM/Docker details when needed.

Index page: [Phase 1 complete (index)](./TUTORIAL_Phase1_Complete_PointClouds_and_Bringup.md)

---

## Suggested order

```text
Read this intro
    ↓
┌───────────────────────────────────────┐
│ Already did Week 1+2?                 │
│   Yes → Week 3 continuation           │
│   No  → Standalone  (or Week 1→2→3)   │
└───────────────────────────────────────┘
    ↓
Phase 1 done (object point clouds)
    ↓
Phase 2 — grasp generation (next project stage)
```

---

## Repo & workspace

- **GitHub:** https://github.com/etiintern3/generalized_picking_ws  
- **Typical local path:** `~/generalized_picking_ws` (`ISAAC_ROS_WS`)  
- **Not in git:** large SAM weights / ONNX, bags, generated clouds, `isaac_ros_common` clone  

Daily run (after setup), inside Isaac ROS container with Sim up:

```bash
bash ${ISAAC_ROS_WS}/scripts/run_phase1.sh
```

---

## After Phase 1

**Phase 2 (next after Standalone):** read [`PHASE2_README.md`](./PHASE2_README.md), then the how-to [`TUTORIAL_Phase2_Grasp_Generation.md`](./TUTORIAL_Phase2_Grasp_Generation.md).  
Handoff notes: [`PHASE2_HANDOFF.md`](./PHASE2_HANDOFF.md).

Motion planning is Phase 3.
