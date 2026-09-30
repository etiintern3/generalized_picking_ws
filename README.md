# Generalized Picking Workspace

UR10e + Robotiq 2F-85 · Isaac Sim 5.1 · Isaac ROS SAM1 (Humble)

**Do not put pick-and-place control here** — that stays in `~/ur_ws`. This workspace is for perception (Phase 1+) and Isaac ROS.

## Tutorials (Phase 1)

1. **Week 1 — Isaac ROS SAM setup**  
   [docs/TUTORIAL_Phase1_Week1_Isaac_ROS_SAM.md](docs/TUTORIAL_Phase1_Week1_Isaac_ROS_SAM.md)

2. **Week 2 — Tray ROI, grid prompts, mask quality**  
   [docs/TUTORIAL_Phase1_Week2_Grid_and_Masks.md](docs/TUTORIAL_Phase1_Week2_Grid_and_Masks.md)

3. **Phase 1 complete — Point clouds, filters, one-command bringup**  
   [docs/TUTORIAL_Phase1_Complete_PointClouds_and_Bringup.md](docs/TUTORIAL_Phase1_Complete_PointClouds_and_Bringup.md)  
   Depth→PLY, NMS + tray-Z filter, rosbag backup, persistent Docker packages, `run_phase1.sh`.

**Quick start (container, Sim running):**  
`bash ${ISAAC_ROS_WS}/scripts/run_phase1.sh`
