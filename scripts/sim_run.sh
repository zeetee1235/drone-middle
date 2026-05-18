#!/usr/bin/env bash
# sim_run.sh — Full sprint-drone simulation runner
#
# Usage:
#   scripts/sim_run.sh [OPTIONS]
#
# Modes:
#   (default)  Standalone Gazebo — camera rig embedded in world, no PX4
#   --sitl     Full PX4 SITL — PX4 + micro-XRCE-DDS + Gazebo spawned by PX4
#
# Options:
#   --sitl          PX4 SITL mode
#   --seed N        ArUco marker placement seed (default: 42)
#   --timeout N     Mission timeout seconds (default: 300)
#   --record        Record rosbag2 (default: on in SITL, off in standalone)
#   --no-record     Disable rosbag2 recording
#   --record-images Include downward camera frames in rosbag2
#   --analyze       Run log_analyzer + plot_mission after the run
#   --report-figures Generate report figures from the recorded rosbag
#   --build         colcon build before launching
#   --headless      Run Gazebo server-only and suppress camera viewer
#   --show-camera   Show rqt_image_view even when not headless
#   --no-tmux       Redirect logs to tmp/ instead of tmux panes
#   -h, --help      Show this help

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ── defaults ──────────────────────────────────────────────────────────────────
MODE="standalone"
SEED=42
TIMEOUT_SEC=300
DO_RECORD=""          # auto-set below if not given
RECORD_IMAGES=false
DO_ANALYZE=false
DO_REPORT_FIGURES=false
DO_BUILD=false
HEADLESS=false
SHOW_CAMERA=true
USE_TMUX=true
SESSION="sprint_sim"

# ── arg parsing ───────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --sitl)        MODE="sitl" ;;
    --seed)        SEED="$2"; shift ;;
    --timeout)     TIMEOUT_SEC="$2"; shift ;;
    --record)      DO_RECORD=true ;;
    --no-record)   DO_RECORD=false ;;
    --record-images) RECORD_IMAGES=true ;;
    --analyze)     DO_ANALYZE=true ;;
    --report-figures) DO_REPORT_FIGURES=true; DO_RECORD=true; RECORD_IMAGES=true ;;
    --build)       DO_BUILD=true ;;
    --headless)    HEADLESS=true; SHOW_CAMERA=false ;;
    --show-camera) SHOW_CAMERA=true ;;
    --no-tmux)     USE_TMUX=false ;;
    -h|--help)
      sed -n '2,/^set -/{ /^set -/d; s/^# \?//p }' "$0"
      exit 0
      ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
  shift
done

# Default: record in SITL, not in standalone
if [[ -z "${DO_RECORD}" ]]; then
  [[ "${MODE}" == "sitl" ]] && DO_RECORD=true || DO_RECORD=false
fi

# ── helpers ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GRN='\033[0;32m'; YLW='\033[1;33m'; BLU='\033[0;34m'; NC='\033[0m'
log()  { echo -e "${BLU}[sim_run]${NC} $*"; }
ok()   { echo -e "${GRN}[sim_run] ✓${NC} $*"; }
warn() { echo -e "${YLW}[sim_run] !${NC} $*"; }
die()  { echo -e "${RED}[sim_run] ✗${NC} $*" >&2; exit 1; }

# ── pre-flight checks ─────────────────────────────────────────────────────────
log "Mode: ${MODE} | seed: ${SEED} | timeout: ${TIMEOUT_SEC}s | record: ${DO_RECORD} | headless: ${HEADLESS} | tmux: ${USE_TMUX}"

ROS_SETUP="/opt/ros/jazzy/setup.bash"
[[ -f "${ROS_SETUP}" ]] || die "ROS2 Jazzy not found at ${ROS_SETUP}. Run: scripts/install_dev_env_ubuntu24.sh"

INSTALL_SETUP="${ROOT}/install/setup.bash"
[[ -f "${INSTALL_SETUP}" ]] || die "Workspace not built yet. Run with --build or: colcon build"

command -v gz >/dev/null || die "gz not found. Install: sudo apt install gz-harmonic"

if [[ "${MODE}" == "sitl" ]]; then
  PX4_DIR="${PX4_DIR:-${ROOT}/external/PX4-Autopilot}"
  [[ -d "${PX4_DIR}" ]] || die "PX4-Autopilot not found at ${PX4_DIR}. Run: scripts/setup_px4_sitl.sh --build-px4"
fi

if [[ "${USE_TMUX}" == "true" ]] && ! command -v tmux >/dev/null; then
  warn "tmux not found; falling back to --no-tmux"
  USE_TMUX=false
fi

# ── ros2 source helper (ROS setup files use unset vars; guard with set +u) ──
ros_source() {
  set +u
  # shellcheck source=/dev/null
  source "${ROS_SETUP}"
  # shellcheck source=/dev/null
  [[ -f "${INSTALL_SETUP}" ]] && source "${INSTALL_SETUP}"
  set -u
}

# ── optional build ────────────────────────────────────────────────────────────
if [[ "${DO_BUILD}" == "true" ]]; then
  log "Building workspace..."
  ros_source
  cd "${ROOT}"
  colcon build --symlink-install 2>&1 | grep -E 'Starting|Finished|ERROR|error'
  ok "Build done"
fi

# ── source environment ────────────────────────────────────────────────────────
ros_source

# ── marker placement ──────────────────────────────────────────────────────────
log "Placing ArUco markers (seed=${SEED})..."
python3 "${ROOT}/tools/spawn_markers.py" --seed "${SEED}" --write
# sync source → install so launch.py picks up the new marker positions
INSTALLED_SDF="${ROOT}/install/sprint_drone/share/sprint_drone/worlds/sprint_grid_world.sdf"
SRC_SDF="${ROOT}/src/sprint_drone/worlds/sprint_grid_world.sdf"
if [[ -f "${INSTALLED_SDF}" ]]; then
  if [[ "${SRC_SDF}" -ef "${INSTALLED_SDF}" ]]; then
    log "Installed world SDF already points to source; skipping copy"
  else
    cp "${SRC_SDF}" "${INSTALLED_SDF}"
  fi
fi
ok "Marker placement written to world SDF"

# ── rosbag output dir ─────────────────────────────────────────────────────────
BAG_TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="${ROOT}/bags/run_${MODE}_${SEED}_${BAG_TIMESTAMP}"

# ── process tracking ──────────────────────────────────────────────────────────
declare -a PIDS=()
LOG_DIR="${ROOT}/tmp/sim_logs/${BAG_TIMESTAMP}"
mkdir -p "${LOG_DIR}"

# Kill all tracked processes on exit
cleanup() {
  log "Shutting down all processes..."
  for pid in "${PIDS[@]}"; do
    kill "${pid}" 2>/dev/null || true
  done
  # Give tmux session a moment then kill it
  if [[ "${USE_TMUX}" == "true" ]]; then
    sleep 1
    tmux kill-session -t "${SESSION}" 2>/dev/null || true
  fi
  wait 2>/dev/null || true
  ok "Cleanup complete. Logs: ${LOG_DIR}"
}
trap cleanup EXIT INT TERM

# ── launch helpers ────────────────────────────────────────────────────────────

TMUX_WIN=0

_tmux_new_window() {
  local name="$1"; shift
  if [[ ${TMUX_WIN} -eq 0 ]]; then
    tmux new-session -d -s "${SESSION}" -n "${name}" 2>/dev/null || \
      tmux new-window -t "${SESSION}" -n "${name}"
  else
    tmux new-window -t "${SESSION}" -n "${name}"
  fi
  TMUX_WIN=$(( TMUX_WIN + 1 ))
  tmux send-keys -t "${SESSION}:${name}" "$*" Enter
}

run_bg() {
  # run_bg <name> <cmd...>
  local name="$1"; shift
  local logfile="${LOG_DIR}/${name}.log"
  if [[ "${USE_TMUX}" == "true" ]]; then
    _tmux_new_window "${name}" "set +u; source ${ROS_SETUP}; source ${INSTALL_SETUP}; set -u; $* 2>&1 | tee ${logfile}"
    # PID tracking for tmux is approximate; we rely on kill-session
  else
    eval "$*" >"${logfile}" 2>&1 &
    PIDS+=($!)
    log "  ${name} PID=$!"
  fi
}

# ── wait helpers ──────────────────────────────────────────────────────────────

_topic_recv() {
  # Print first message from topic, return non-zero if nothing arrives in 3s.
  timeout 3 ros2 topic echo --once "$1" 2>/dev/null || true
}

_topic_list_has() {
  ros2 topic list 2>/dev/null | grep -qF "$1"
}

wait_topic() {
  # wait_topic <description> <topic> <timeout_sec>
  local desc="$1" topic="$2" max_sec="$3"
  log "Waiting for ${desc} (${topic})..."
  local elapsed=0
  while ! _topic_list_has "${topic}" || ! _topic_recv "${topic}" | grep -q .; do
    sleep 2; elapsed=$(( elapsed + 2 ))
    [[ ${elapsed} -ge ${max_sec} ]] && die "Timeout (${max_sec}s) waiting for ${desc}"
    printf "."
  done
  echo ""; ok "${desc} ready"
}

wait_service() {
  local desc="$1" svc="$2" max_sec="$3"
  log "Waiting for ${desc} (${svc})..."
  local elapsed=0
  while ! ros2 service list 2>/dev/null | grep -qF "${svc}"; do
    sleep 2; elapsed=$(( elapsed + 2 ))
    [[ ${elapsed} -ge ${max_sec} ]] && die "Timeout (${max_sec}s) waiting for ${desc}"
    printf "."
  done
  echo ""; ok "${desc} ready"
}

wait_mission_state() {
  log "Waiting for mission_manager to initialize..."
  local elapsed=0 state=""
  while [[ -z "${state}" ]]; do
    state="$(_topic_recv /mission/state | grep 'data:' | awk '{print $2}' || true)"
    [[ -n "${state}" ]] && break
    sleep 2; elapsed=$(( elapsed + 2 ))
    [[ ${elapsed} -ge 60 ]] && die "mission_manager did not come up within 60s"
    printf "."
  done
  echo ""; ok "mission_manager online (state=${state})"
}

# ── STANDALONE MODE ───────────────────────────────────────────────────────────

start_standalone() {
  log "=== Standalone Gazebo mode ==="

  run_bg "gazebo" \
    "ros2 launch sprint_drone sim_gazebo.launch.py headless:=${HEADLESS} show_camera:=${SHOW_CAMERA}"

  log "Waiting for Gazebo camera topic..."
  sleep 6  # Gazebo + bridge take a moment to negotiate
  local elapsed=0
  while ! ros2 topic list 2>/dev/null | grep -q "/drone/camera/down/image_raw"; do
    sleep 2; elapsed=$(( elapsed + 2 ))
    [[ ${elapsed} -ge 40 ]] && die "Gazebo camera bridge did not come up in 40s"
    echo -n "."
  done
  echo ""; ok "Gazebo + bridge ready"

  # use_sim_pose=true: skip visual_odometry; gz_drone_sim publishes exact pose_grid
  run_bg "perception" \
    "ros2 launch sprint_drone sim_perception.launch.py use_sim_pose:=true"
  sleep 2

  run_bg "planner" \
    "ros2 launch sprint_drone sim_planner.launch.py"

  run_bg "mission" \
    "ros2 launch sprint_drone sim_mission.launch.py"

  # In standalone mode there is no real PX4. The control node still publishes
  # /fmu/in/* setpoints, and gz_drone_sim consumes trajectory_setpoint to move
  # the camera rig through the Gazebo world.
  run_bg "control" \
    "ros2 launch sprint_drone sim_control.launch.py"

  # Move the sprint_camera_rig in Gazebo based on trajectory setpoints so
  # the Gazebo GUI shows the drone actually flying and the camera sees the
  # ground moving (enabling real optical-flow feedback).
  run_bg "gz_drone_sim" \
    "python3 ${ROOT}/scripts/gz_drone_sim.py --world sprint_grid_world"
}

# ── SITL MODE ─────────────────────────────────────────────────────────────────

start_sitl() {
  log "=== PX4 SITL mode ==="

  run_bg "xrce_agent" \
    "bash ${ROOT}/scripts/run_micro_xrce_agent.sh"
  sleep 2

  run_bg "px4_sitl" \
    "bash ${ROOT}/scripts/run_px4_sitl.sh"

  # Wait for PX4 SITL to publish vehicle status via micro-XRCE-DDS
  wait_topic "PX4 vehicle_status" "/fmu/out/vehicle_status" 120

  run_bg "gz_bridge" \
    "ros2 launch sprint_drone sim_gazebo.launch.py use_px4_sitl:=true show_camera:=${SHOW_CAMERA}"

  wait_topic "camera bridge" "/drone/camera/down/image_raw" 30

  run_bg "perception" \
    "ros2 launch sprint_drone sim_perception.launch.py"
  sleep 2

  run_bg "planner" \
    "ros2 launch sprint_drone sim_planner.launch.py"

  run_bg "mission" \
    "ros2 launch sprint_drone sim_mission.launch.py"

  run_bg "control" \
    "ros2 launch sprint_drone sim_control.launch.py"
}

# ── ROSBAG RECORDING ──────────────────────────────────────────────────────────

start_recording() {
  log "Starting rosbag2 recording → ${BAG_DIR}"
  local exclude_regex="/parameter_events|/rosout|/events/write_split"
  if [[ "${RECORD_IMAGES}" != "true" ]]; then
    exclude_regex="/drone/camera/down/image_raw|${exclude_regex}"
  fi
  run_bg "rosbag" \
    "ros2 bag record --all-topics --exclude-regex '${exclude_regex}' -o ${BAG_DIR}"
  sleep 1
}

ensure_bag_metadata() {
  if [[ ! -d "${BAG_DIR}" ]]; then
    return 1
  fi
  if [[ -f "${BAG_DIR}/metadata.yaml" ]]; then
    return 0
  fi

  warn "rosbag metadata missing; attempting ros2 bag reindex"
  if ros2 bag reindex "${BAG_DIR}"; then
    return 0
  fi
  return 1
}

finish_recording() {
  if [[ "${DO_RECORD}" != "true" || "${USE_TMUX}" == "true" ]]; then
    return
  fi

  log "Stopping rosbag2 recording cleanly..."
  pkill -INT -f "ros2.*bag record.*${BAG_DIR}" 2>/dev/null || true
  sleep 3
  ensure_bag_metadata || warn "rosbag metadata still unavailable"
}

# ── MISSION LOOP ──────────────────────────────────────────────────────────────

run_mission() {
  wait_service "/mission/start service" "/mission/start" 60

  if [[ "${DO_RECORD}" == "true" ]]; then
    start_recording
  fi

  log "Triggering mission start..."
  ros2 service call /mission/start std_srvs/srv/Trigger "{}" | grep -E "success|message" || true
  ok "Mission started"

  log "Monitoring mission state (timeout: ${TIMEOUT_SEC}s)..."
  local elapsed=0
  local last_state="" state=""
  while true; do
    state="$(_topic_recv /mission/state | grep 'data:' | awk '{print $2}' || true)"
    if [[ -n "${state}" && "${state}" != "${last_state}" ]]; then
      log "  state → ${state}"
      last_state="${state}"
    fi
    if [[ "${state}" == "LANDED" || "${state}" == "ABORT" ]]; then
      break
    fi
    sleep 2; elapsed=$(( elapsed + 2 ))
    if [[ ${elapsed} -ge ${TIMEOUT_SEC} ]]; then
      warn "Mission timeout after ${TIMEOUT_SEC}s (last state: ${last_state:-UNKNOWN})"
      break
    fi
  done

  if [[ "${state}" == "LANDED" ]]; then
    ok "Mission complete! Final state: LANDED"
  else
    warn "Mission ended with state: ${state:-UNKNOWN}"
  fi
}

# ── POST-RUN ANALYSIS ─────────────────────────────────────────────────────────

run_analysis() {
  if [[ "${DO_RECORD}" != "true" ]]; then
    log "Skipping analysis (no rosbag recorded)"
    return
  fi
  if [[ ! -d "${BAG_DIR}" ]]; then
    warn "Bag dir not found: ${BAG_DIR}"
    return
  fi
  ensure_bag_metadata || return

  log "Running log_analyzer..."
  if python3 "${ROOT}/tools/log_analyzer.py" "${BAG_DIR}"; then
    ok "log_analyzer done"
  else
    warn "log_analyzer failed (check pip install rosbags numpy)"
  fi

  REPORT_DIR="${ROOT}/reports"
  mkdir -p "${REPORT_DIR}"
  PLOT_OUT="${REPORT_DIR}/$(basename "${BAG_DIR}").png"
  log "Running plot_mission → ${PLOT_OUT}"
  if python3 "${ROOT}/tools/plot_mission.py" "${BAG_DIR}" --output "${PLOT_OUT}"; then
    ok "Plot saved: ${PLOT_OUT}"
  else
    warn "plot_mission failed (check pip install matplotlib)"
  fi
}

run_report_figures() {
  if [[ "${DO_RECORD}" != "true" ]]; then
    log "Skipping report figures (no rosbag recorded)"
    return
  fi
  if [[ ! -d "${BAG_DIR}" ]]; then
    warn "Bag dir not found: ${BAG_DIR}"
    return
  fi
  ensure_bag_metadata || return

  log "Generating report figures from Gazebo rosbag..."
  if python3 "${ROOT}/reports/generate_report_figures.py" "${BAG_DIR}"; then
    ok "Report figures updated in ${ROOT}/reports"
    if python3 "${ROOT}/scripts/plot_phase1_search.py"; then
      ok "Phase 1 search figure refreshed from clean marker-layout plotter"
    fi
    # The current Gazebo smoke bag may end during GRID_SEARCH, so generate the
    # full mission path figure with the mission-sequence plotter used for
    # mission_path_real-style report figures.
    if python3 "${ROOT}/scripts/plot_mission_path.py"; then
      cp "${ROOT}/reports/mission_path.png" \
         "${ROOT}/reports/fig_06_gazebo_mission_path.png"
      ok "Mission sequence figure refreshed from plot_mission_path"
    fi
    if python3 "${ROOT}/scripts/generate_report_extra_figures.py"; then
      ok "Design/sweep report figures refreshed"
    fi
  else
    warn "Report figure generation failed"
  fi
}

# ── TMUX SESSION SETUP ────────────────────────────────────────────────────────

if [[ "${USE_TMUX}" == "true" ]]; then
  tmux kill-session -t "${SESSION}" 2>/dev/null || true
  log "Launching in tmux session '${SESSION}'"
  log "  Attach with:  tmux attach -t ${SESSION}"
fi

# ── MAIN ──────────────────────────────────────────────────────────────────────

echo ""
log "Log dir: ${LOG_DIR}"
log "Bag dir: ${BAG_DIR} (record=${DO_RECORD})"
echo ""

case "${MODE}" in
  standalone) start_standalone ;;
  sitl)       start_sitl ;;
esac

sleep 3
wait_mission_state
run_mission

log "Stopping processes..."
finish_recording
# Signal all children; cleanup trap will do the rest.
for pid in "${PIDS[@]}"; do
  kill "${pid}" 2>/dev/null || true
done
[[ "${USE_TMUX}" == "true" ]] && sleep 2 && tmux kill-session -t "${SESSION}" 2>/dev/null || true

if [[ "${DO_ANALYZE}" == "true" ]]; then
  run_analysis
fi
if [[ "${DO_REPORT_FIGURES}" == "true" ]]; then
  run_report_figures
fi

echo ""
ok "Done."
[[ "${DO_RECORD}" == "true" ]] && echo "  Bag:    ${BAG_DIR}"
[[ "${DO_ANALYZE}" == "true" && -d "${ROOT}/reports" ]] && echo "  Report: ${ROOT}/reports/"
[[ "${DO_REPORT_FIGURES}" == "true" && -d "${ROOT}/reports" ]] && echo "  Figures: ${ROOT}/reports/fig_*.png"
