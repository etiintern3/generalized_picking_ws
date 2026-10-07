# Tutorial — Phase 2: Grasp generation (after Phase 1 Standalone)

**For:** you already finished [Phase 1 Standalone from GitHub](./TUTORIAL_Phase1_Standalone_From_GitHub.md) on this machine (`~/generalized_picking_ws`, Isaac ROS SAM1, Sim camera working).

**Goal:** From the live front RGB-D view, get a **ranked list of 6-DoF grasps** for the Robotiq **2F-85** (max opening **85 mm**) using **Contact-GraspNet** on the **full scene** (not isolated object PLYs).

**Not in this tutorial:** MoveIt, IK, or execution (Phase 3). Do **not** modify `~/ur_ws`.

**Poses:** camera optical / OpenCV (`x` right, `y` down, `z` forward). Frame id is usually `sim_camera`.

---

## 0. What changed since Phase 1 Standalone

Phase 1 Standalone used a **top-down tray** mental model (`tray_grid_prompts.py`, NMS + avg‑Z clouds). The **current** cell uses a **front-facing** camera. After you pull Phase 2 scripts:

| Piece | Phase 1 Standalone (old) | Phase 2 (current) |
|--------|---------------------------|-------------------|
| Prompts | `tray_grid_prompts.py` | `frontcam_table_grid_prompts.py` |
| Object PLYs | `masks_to_pointclouds_nms.py` → `clouds_nms/` | `masks_to_pointclouds_planar.py` → `clouds_planar/` |
| Bringup | `run_phase1.sh` | same script, now starts **front-cam + planar** |
| Grasps | — | full-scene CGN + `run_phase2.sh` |

Do **not** run `tray_grid_prompts.py` at the same time as the front-cam prompt node (both publish `/prompts`).

**Pipeline:**

```text
Isaac Sim (front RGB-D)
    → Isaac ROS SAM1 + front-cam bbox grid
    → sam_scene_to_cgn.py  →  scene_*.npz  (rgb, depth, K, segmap)
    → Contact-GraspNet (jp_cgnet Docker)
    → rank_cgn_grasps.py  →  ranked JSON + preview PNG
    → (optional) visualize_ranked_grasps.py on the point cloud
```

Envs stay separate:

| Job | Where |
|-----|--------|
| SAM / scene dump | Isaac ROS Dev container |
| Contact-GraspNet | Docker `jp_cgnet` |
| Rank + Open3D viz | Host `/usr/bin/python3` (numpy, matplotlib, open3d) |

UOIS / `uois3d` are **not** part of this pipeline.

---

## 1. Update this workspace from GitHub

On the **host**, in the folder you already use for Phase 1:

```bash
export ISAAC_ROS_WS=$HOME/generalized_picking_ws
cd ${ISAAC_ROS_WS}

git fetch origin
git status
# Recommended: short-lived branch, then merge to main via PR (see §9)
git checkout -b phase2-cgn
git pull origin main   # if you already pushed; otherwise skip
```

After the Phase 2 PR is on `main`:

```bash
cd ${ISAAC_ROS_WS}
git checkout main
git pull origin main
```

You should see at least:

- `scripts/frontcam_table_grid_prompts.py`
- `scripts/masks_to_pointclouds_planar.py`
- `scripts/sam_scene_to_cgn.py`
- `scripts/run_phase2.sh`
- `scripts/rank_cgn_grasps.py`
- `scripts/visualize_ranked_grasps.py`
- `scripts/cgn_infer_once.py`

---

## 2. Install Contact-GraspNet (once, on the host)

CGN is **not** installed inside Isaac ROS. Clone upstream next to our scripts (do **not** commit checkpoints into this repo).

```bash
mkdir -p ${ISAAC_ROS_WS}/third_party
cd ${ISAAC_ROS_WS}/third_party

# Skip if you already have third_party/cgnet working
git clone --recursive https://github.com/jishnujayakumar/contact_graspnet.git cgnet

cd ${ISAAC_ROS_WS}/third_party/cgnet/docker
docker-compose up -d
# If that fails: docker compose up -d
# Wrapper on some machines: /usr/local/bin/docker-compose
```

Enter the container and finish setup **once**:

```bash
docker-compose exec jp_cgnet bash
# inside:
conda activate jp_cgnet
cd /home/$USER/generalized_picking_ws/third_party/cgnet   # adjust user/path if needed
# same path as host mount: /home/satwik/generalized_picking_ws/third_party/cgnet

python download_assets.py
# if gdown/typing errors: pip install gdown

# If PointNet ops fail later:
# sh compile_pointnet_tfops.sh
exit
```

You need checkpoint dir:

`third_party/cgnet/checkpoints/scene_test_2048_bs3_hor_sigma_001/`

Image: `irvlutd/jp_cgnet` (pulled/built via their docker setup).

---

## 3. Host deps for ranking / viz

```bash
/usr/bin/python3 -c "import numpy, matplotlib; print('ok')"
/usr/bin/python3 -c "import open3d; print(open3d.__version__)"
```

If Open3D is missing:

```bash
/usr/bin/python3 -m pip install --user 'numpy<2' matplotlib open3d
```

Use **`/usr/bin/python3`**, not Miniconda `base` (often has no numpy).

---

## 4. Daily run — perception (Isaac ROS)

**Host:** Isaac Sim 5.1 with front camera publishing:

`/camera/realsense/{rgb,depth,camera_info}` — 1280×720, depth `32FC1`.

Put **2–3 objects** on the table in view.

**Container:**

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh
```

Inside:

```bash
source /opt/ros/humble/setup.bash
bash ${ISAAC_ROS_WS}/scripts/run_phase1.sh
```

This starts SAM (`prompt_input_type:=bbox`), **front-cam** grid prompts, and planar PLYs (optional for CGN). Confirm masks look good in rqt.

Leave this terminal running.

---

## 5. Dump one CGN scene (Isaac ROS, second terminal)

Attach another shell to the same Dev container, then:

```bash
source /opt/ros/humble/setup.bash
bash ${ISAAC_ROS_WS}/scripts/run_phase2.sh dump
```

Waits until a **new** file appears:

`output/cgn_scenes/scene_<stamp>.npz` with `objects=N` and **N ≥ 1**.

(The dumper only saves every ~60 mask frames.) Ctrl+C is handled by the script when a file appears.

Keys in the `npz`: `rgb`, `depth`, `K`, `segmap` (instance ids after area / NMS / planarity).

---

## 6. Contact-GraspNet + rank (host)

```bash
bash ${ISAAC_ROS_WS}/scripts/run_phase2.sh
# or: SCENE=${ISAAC_ROS_WS}/output/cgn_scenes/scene_XXXX.npz bash ${ISAAC_ROS_WS}/scripts/run_phase2.sh
```

By default this:

1. Starts `jp_cgnet` if needed  
2. Runs CGN with **Mayavi / image windows** (close them to continue) — same as manual `inference.py`  
3. Ranks with `/usr/bin/python3` → `output/grasps/scene_*_ranked.json` + `*_preview.png`

Useful env knobs:

| Variable | Default | Meaning |
|----------|---------|---------|
| `USE_VIZ_CGN` | `1` | `0` = headless (`cgn_infer_once.py`) |
| `SKIP_CGN` | `0` | `1` = re-rank only |
| `MIN_SCORE` | `0.10` | drop low-confidence grasps |
| `MAX_WIDTH` | `0.085` | 2F-85 opening cap (m) |
| `MIN_HEIGHT` | `0.008` | contact above fitted table (m) |
| `MAX_INTO_TABLE` | `1.0` | **≥ 1.0 = off** (front-cam top grasps look “into” the table) |

Example — re-rank only, looser table height:

```bash
SKIP_CGN=1 MIN_HEIGHT=0.0 bash ${ISAAC_ROS_WS}/scripts/run_phase2.sh
```

CGN is called with `--local_regions --filter_grasps`, `--z_range=[0.3,1.8]`, and `DATA.gripper_width:0.085` (clamp at generate time; network is still the Panda-trained model).

---

## 7. Inspect results

**2D preview** (written by the ranker):

`output/grasps/scene_*_preview.png`

**3D — filtered grasps on the scene cloud** (standalone; not part of `run_phase2.sh`):

```bash
/usr/bin/python3 ${ISAAC_ROS_WS}/scripts/visualize_ranked_grasps.py \
  --ranked ${ISAAC_ROS_WS}/output/grasps/scene_XXXX_ranked.json
```

Options: `--top-k 5`, `--object-id 4 5 7`, `--show-approach`.

**JSON fields** (per grasp): `score`, `width_m`, `contact_cam`, `approach_cam`, `T_cam` (4×4).

If ranking prints `rejected: {'below_table': ...}`, loosen `MIN_HEIGHT`.  
`into_table` is off by default; only enable if you know you need it (`ENABLE_INTO_TABLE=1`).

---

## 8. Success criteria

- [ ] CGN Docker runs; checkpoint `scene_test_2048_bs3_hor_sigma_001` present  
- [ ] `scene_*.npz` with **≥ 1** object in `segmap`  
- [ ] Ranked JSON has non-empty `grasps` for the objects you care about  
- [ ] `width_m ≤ 0.085` (not `null` — if null, re-run CGN once)  
- [ ] Preview / Open3D viz: contacts on objects, not empty table  

**Phase 2 done.** Next (Phase 3): TF `sim_camera` → `base_link` / `world`, then IK and staged motion — still without rewriting `ur_ws` until you choose to.

---

## 9. Scripts reference

| Script | Role |
|--------|------|
| `run_phase1.sh` | SAM + front-cam prompts + planar PLYs |
| `run_phase2.sh dump` | One scene `npz` (Isaac ROS) |
| `run_phase2.sh` | CGN + rank (host) |
| `sam_scene_to_cgn.py` | Scene dump node |
| `rank_cgn_grasps.py` | Filter / rank / preview PNG |
| `visualize_ranked_grasps.py` | Open3D cloud + kept grasps |
| `cgn_infer_once.py` | Headless CGN (used when `USE_VIZ_CGN=0`) |

Manual CGN (optional; `run_phase2.sh` already does this):

```bash
cd ${ISAAC_ROS_WS}/third_party/cgnet/docker
docker-compose exec jp_cgnet bash
conda activate jp_cgnet
cd ${ISAAC_ROS_WS}/third_party/cgnet

python contact_graspnet/inference.py \
  --np_path=${ISAAC_ROS_WS}/output/cgn_scenes/scene_XXXX.npz \
  --ckpt_dir=checkpoints/scene_test_2048_bs3_hor_sigma_001 \
  --local_regions --filter_grasps --forward_passes=5 \
  --z_range=[0.3,1.8] \
  --arg_configs DATA.gripper_width:0.085
```

---

## 10. Dead ends (do not retry)

- **AnyGrasp** — skipped (license)  
- **UOIS** — not used; leave `third_party/uois` alone if present  
- **Isolated object PLYs as CGN input** — worse than full scene + `segmap`  
- Old **top-down** `tray_grid_prompts.py` with the front camera  

---

## 11. Where to read more

| Topic | Doc |
|--------|-----|
| Phase 1 install you already did | [Standalone from GitHub](./TUTORIAL_Phase1_Standalone_From_GitHub.md) |
| Agent handoff / roadmap notes | [PHASE2_HANDOFF.md](./PHASE2_HANDOFF.md) |
| Phase 1 intro | [PHASE1_README.md](./PHASE1_README.md) |
