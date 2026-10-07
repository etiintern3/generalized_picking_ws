#!/usr/bin/env bash
# Phase 2 bringup: dump a CGN scene (Isaac ROS) and/or run CGN + rank (host).
#
# Phase 2 spans two machines/envs — use the matching mode:
#
#   DUMP (inside Isaac ROS Dev, with Sim + run_phase1.sh already running):
#     bash ${ISAAC_ROS_WS}/scripts/run_phase2.sh dump
#
#   GRASP (on host — Contact-GraspNet Docker + system Python rank):
#     bash /home/satwik/generalized_picking_ws/scripts/run_phase2.sh
#     SCENE=/path/to/scene_XXX.npz bash .../run_phase2.sh
#
#   BOTH on host if a scene already exists (skip dump):
#     bash .../run_phase2.sh grasp
#
# Env knobs:
#   DUMP_TIMEOUT_SEC=180   FORWARD_PASSES=5
#   MAX_WIDTH=0.085  MIN_SCORE=0.10  TOP_K=10
#   MIN_HEIGHT=0.008   # contact height above table (m); lower if kept=0 but raw>0
#   MAX_INTO_TABLE=1.0   # >=1.0 = OFF (default). Front-cam top grasps look "into table".
#                        # Legacy on: MAX_INTO_TABLE=0.35 or ENABLE_INTO_TABLE=1
#   SKIP_CGN=0  SKIP_RANK=0
#   USE_VIZ_CGN=1   # default ON — Mayavi point-cloud grasps (close windows to continue)
#                   # USE_VIZ_CGN=0 for headless
#
# Ranking / kept=0 tuning is scripts/rank_cgn_grasps.py (not CGN itself).
# Does NOT start Phase 1 / SAM. Does NOT use uois3d.
set -eo pipefail

MODE="${1:-grasp}"
HOST_WS="${HOST_WS:-/home/satwik/generalized_picking_ws}"
# Inside Isaac ROS the workspace is usually mounted here:
ROS_WS="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"

DUMP_TIMEOUT_SEC="${DUMP_TIMEOUT_SEC:-180}"
FORWARD_PASSES="${FORWARD_PASSES:-5}"
MAX_WIDTH="${MAX_WIDTH:-0.085}"
MIN_SCORE="${MIN_SCORE:-0.10}"
TOP_K="${TOP_K:-10}"
MIN_HEIGHT="${MIN_HEIGHT:-0.008}"
MAX_INTO_TABLE="${MAX_INTO_TABLE:-1.0}"
ENABLE_INTO_TABLE="${ENABLE_INTO_TABLE:-0}"
SKIP_CGN="${SKIP_CGN:-0}"
SKIP_RANK="${SKIP_RANK:-0}"
USE_VIZ_CGN="${USE_VIZ_CGN:-1}"
CKPT_DIR="${CKPT_DIR:-checkpoints/scene_test_2048_bs3_hor_sigma_001}"

log() { echo "[phase2] $*"; }
die() { echo "[phase2] ERROR: $*" >&2; exit 1; }

latest_scene() {
  local dir="$1"
  ls -1t "${dir}"/scene_*.npz 2>/dev/null | head -1 || true
}

compose_up() {
  local dir="$1"
  if command -v docker-compose >/dev/null 2>&1; then
    ( cd "${dir}" && docker-compose up -d )
  elif docker compose version >/dev/null 2>&1; then
    ( cd "${dir}" && docker compose up -d )
  else
    die "neither docker-compose nor 'docker compose' found"
  fi
}

# ---------------------------------------------------------------------------
# dump — Isaac ROS container
# ---------------------------------------------------------------------------
do_dump() {
  local ws="${ROS_WS}"
  local out_dir="${ws}/output/cgn_scenes"
  local script="${ws}/scripts/sam_scene_to_cgn.py"

  [[ -f "${script}" ]] || die "missing ${script} (run inside Isaac ROS Dev?)"

  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u

  if ! ros2 topic list 2>/dev/null | grep -q '/camera/realsense/rgb'; then
    log "WARNING: /camera/realsense/rgb not seen. Is Sim + run_phase1.sh up?"
  fi
  if ! ros2 topic list 2>/dev/null | grep -q '/segment_anything/raw_segmentation_mask'; then
    log "WARNING: SAM masks not seen. Start: bash \${ISAAC_ROS_WS}/scripts/run_phase1.sh"
  fi

  mkdir -p "${out_dir}"
  local before
  before="$(latest_scene "${out_dir}")"
  log "Dumping one CGN scene (timeout ${DUMP_TIMEOUT_SEC}s)..."
  log "Waiting for a NEW file under ${out_dir}/ (saves every ~60 mask frames)"

  python3 "${script}" &
  local dump_pid=$!

  local deadline=$((SECONDS + DUMP_TIMEOUT_SEC))
  local after=""
  while (( SECONDS < deadline )); do
    after="$(latest_scene "${out_dir}")"
    if [[ -n "${after}" && "${after}" != "${before}" ]]; then
      break
    fi
    sleep 1
  done

  kill "${dump_pid}" 2>/dev/null || true
  wait "${dump_pid}" 2>/dev/null || true

  after="$(latest_scene "${out_dir}")"
  if [[ -z "${after}" || "${after}" == "${before}" ]]; then
    die "no new scene_*.npz within ${DUMP_TIMEOUT_SEC}s. Check SAM masks / ROI / objects."
  fi

  # Host path (same file via mount)
  local host_scene="${HOST_WS}/output/cgn_scenes/$(basename "${after}")"
  log "Saved: ${after}"
  log "Host path: ${host_scene}"
  log "Next (on host): bash ${HOST_WS}/scripts/run_phase2.sh"
  # Print path alone on last line for scripting
  echo "${host_scene}"
}

# ---------------------------------------------------------------------------
# grasp — host: CGN Docker + rank
# ---------------------------------------------------------------------------
do_grasp() {
  local ws="${HOST_WS}"
  local scenes="${ws}/output/cgn_scenes"
  local cgnet="${ws}/third_party/cgnet"
  local docker_dir="${cgnet}/docker"
  local rank_py="${ws}/scripts/rank_cgn_grasps.py"
  local infer_py="${ws}/scripts/cgn_infer_once.py"
  local py="${PYTHON:-/usr/bin/python3}"

  [[ -d "${cgnet}" ]] || die "missing ${cgnet}"
  [[ -f "${rank_py}" ]] || die "missing ${rank_py}"

  local scene="${SCENE:-}"
  if [[ -z "${scene}" ]]; then
    scene="$(latest_scene "${scenes}")"
  fi
  [[ -n "${scene}" && -f "${scene}" ]] || die "no scene_*.npz in ${scenes}. Run: bash run_phase2.sh dump (in Isaac ROS)"
  scene="$(realpath "${scene}")"
  local stem
  stem="$(basename "${scene}" .npz)"   # scene_XXXX
  local pred="${cgnet}/results/predictions_${stem}.npz"

  log "Scene: ${scene}"
  log "Pred will be: ${pred}"

  if [[ "${SKIP_CGN}" != "1" ]]; then
    log "Starting jp_cgnet..."
    compose_up "${docker_dir}"

    # Wait until container is running
    local i
    for i in $(seq 1 30); do
      if docker inspect -f '{{.State.Running}}' jp_cgnet 2>/dev/null | grep -q true; then
        break
      fi
      sleep 1
    done
    docker inspect -f '{{.State.Running}}' jp_cgnet 2>/dev/null | grep -q true \
      || die "jp_cgnet container not running"

    if [[ "${USE_VIZ_CGN}" == "1" ]]; then
      log "CGN inference + point-cloud grasp viz (close image/Mayavi windows to continue)..."
      # -i keeps X11/Mayavi happier; DISPLAY comes from compose env on the container
      docker exec -i -w "${cgnet}" \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e QT_X11_NO_MITSHM=1 \
        jp_cgnet bash -lc "
        set -e
        source ~/.bashrc 2>/dev/null || true
        conda activate jp_cgnet
        python contact_graspnet/inference.py \
          --np_path=${scene} \
          --ckpt_dir=${CKPT_DIR} \
          --local_regions \
          --filter_grasps \
          --forward_passes=${FORWARD_PASSES} \
          --z_range=[0.3,1.8] \
          --arg_configs DATA.gripper_width:0.085
      "
    else
      log "CGN inference (headless via cgn_infer_once.py)..."
      docker exec -w "${cgnet}" jp_cgnet bash -lc "
        set -e
        source ~/.bashrc 2>/dev/null || true
        conda activate jp_cgnet
        python ${infer_py} \
          --np_path=${scene} \
          --ckpt_dir=${CKPT_DIR} \
          --local_regions \
          --filter_grasps \
          --forward_passes=${FORWARD_PASSES} \
          --z_range=[0.3,1.8] \
          --arg_configs DATA.gripper_width:0.085
      "
    fi
    [[ -f "${pred}" ]] || die "expected predictions missing: ${pred}"
    log "Predictions: ${pred}"
  else
    [[ -f "${pred}" ]] || die "SKIP_CGN=1 but missing ${pred}"
  fi

  if [[ "${SKIP_RANK}" != "1" ]]; then
    [[ -x "${py}" || -f "${py}" ]] || die "python not found: ${py}"
    local out_grasps="${ws}/output/grasps"
    mkdir -p "${out_grasps}" 2>/dev/null || true
    if [[ ! -w "${out_grasps}" ]]; then
      die "cannot write ${out_grasps} (owned by another user, often Docker 'nobody'). Fix with:
  sudo chown -R \"\$USER:\$USER\" ${out_grasps}
then re-rank only (skip CGN):
  SKIP_CGN=1 bash ${ws}/scripts/run_phase2.sh"
    fi
    log "Ranking with ${py} (tune via MIN_SCORE / MIN_HEIGHT / MAX_WIDTH; into_table off by default)..."
    local rank_extra=()
    if [[ "${ENABLE_INTO_TABLE}" == "1" ]]; then
      rank_extra+=(--enable-into-table)
    fi
    "${py}" "${rank_py}" \
      --pred "${pred}" \
      --scene "${scene}" \
      --max-width "${MAX_WIDTH}" \
      --min-score "${MIN_SCORE}" \
      --top-k "${TOP_K}" \
      --min-height "${MIN_HEIGHT}" \
      --max-into-table "${MAX_INTO_TABLE}" \
      "${rank_extra[@]}"
    local ranked="${out_grasps}/${stem}_ranked.json"
    log "Done. Ranked: ${ranked}"
    log "Preview:  ${out_grasps}/${stem}_preview.png"
    log "If kept=0 but raw>0, read 'rejected:' lines (usually below_table)."
  fi
}

case "${MODE}" in
  dump)
    do_dump
    ;;
  grasp|"" )
    do_grasp
    ;;
  -h|--help|help)
    sed -n '2,25p' "$0"
    ;;
  *)
    die "unknown mode '${MODE}' (use: dump | grasp)"
    ;;
esac
