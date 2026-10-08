# Generalized Picking Workspace

UR10e + Robotiq 2F-85 · Isaac Sim 5.1 · Isaac ROS SAM1 (Humble)

**Do not put pick-and-place control here** — that stays in `~/ur_ws`. This workspace is for perception (Phase 1+) and Isaac ROS.

**GitHub:** https://github.com/etiintern3/generalized_picking_ws

## Start here — Phase 1

**[docs/PHASE1_README.md](docs/PHASE1_README.md)** — intro to Phase 1, what you will build, and which tutorial track to follow (Week 1→2→3 vs standalone from GitHub).

### Tutorial list

1. [Week 1 — Isaac ROS SAM](docs/TUTORIAL_Phase1_Week1_Isaac_ROS_SAM.md)  
2. [Week 2 — ROI & grid masks](docs/TUTORIAL_Phase1_Week2_Grid_and_Masks.md)  
3. [Week 3 — Point clouds (continuation)](docs/TUTORIAL_Phase1_Week3_PointClouds_Continuation.md)  
4. [Standalone — full pipeline from GitHub](docs/TUTORIAL_Phase1_Standalone_From_GitHub.md)  

## Phase 2 (grasps)

**Start here:** **[docs/PHASE2_README.md](docs/PHASE2_README.md)** — what Phase 2 is, pipeline, envs (read before the tutorial).

**Then:** **[docs/TUTORIAL_Phase2_Grasp_Generation.md](docs/TUTORIAL_Phase2_Grasp_Generation.md)** — install CGN, dump scene, run/rank/visualize (assumes Phase 1 Standalone done).

CGN lives under `third_party/cgnet` (clone upstream yourself; not shipped in git).

## Phase 3 (motion) — next

**Handoff for a new agent:** **[docs/PHASE3_HANDOFF.md](docs/PHASE3_HANDOFF.md)** — TF camera→base, IK, collision, staged pick (do not edit `ur_ws` unless asked).  
