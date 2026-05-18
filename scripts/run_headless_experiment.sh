#!/usr/bin/env bash
# run_headless_experiment.sh — ROS2 노드만으로 미션 전체를 실행하고 bag 기록
# Gazebo / PX4 SITL 없이 동작.  mock_sensors.py가 센서 신호를 대체한다.
#
# Usage: bash scripts/run_headless_experiment.sh [--timeout 180]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TIMEOUT_SEC=180
BAG_DIR="${ROOT}/bags/headless_$(date +%Y%m%d_%H%M%S)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --timeout) TIMEOUT_SEC="$2"; shift ;;
    *) echo "Unknown: $1" >&2; exit 1 ;;
  esac
  shift
done

set +u
source /opt/ros/jazzy/setup.bash
source "${ROOT}/install/setup.bash"
set -u

echo "[headless] bag → ${BAG_DIR}"
echo "[headless] timeout = ${TIMEOUT_SEC}s"
mkdir -p "${BAG_DIR}"

PIDS=()
cleanup() {
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

run_bg() {
  local name="$1"; shift
  "$@" > "${BAG_DIR}/${name}.log" 2>&1 &
  PIDS+=($!)
  echo "[headless] ${name} PID=$!"
}

# ── 노드 기동 ─────────────────────────────────────────────────────────────────
run_bg mission_manager  ros2 launch sprint_drone sim_mission.launch.py
sleep 1
run_bg planner          ros2 launch sprint_drone sim_planner.launch.py
sleep 1
# gz_drone_sim: Gazebo 없이도 topic integration loop만 돌아감
run_bg gz_drone_sim     python3 "${ROOT}/scripts/gz_drone_sim.py" \
                          --world sprint_grid_world
sleep 1
run_bg mock_sensors     python3 "${ROOT}/scripts/mock_sensors.py"
sleep 2

# ── mission/start 서비스 대기 → 호출 ─────────────────────────────────────────
echo "[headless] waiting for /mission/start service..."
elapsed=0
while ! ros2 service list 2>/dev/null | grep -q "/mission/start"; do
  sleep 1; elapsed=$((elapsed+1))
  [[ $elapsed -ge 30 ]] && { echo "timeout waiting for service"; exit 1; }
done
echo "[headless] triggering mission start..."
ros2 service call /mission/start std_srvs/srv/Trigger "{}" > /dev/null 2>&1 || true

# ── rosbag 기록 시작 ──────────────────────────────────────────────────────────
run_bg rosbag ros2 bag record \
  /localization/pose_grid \
  /mission/state \
  /planner/velocity_profile \
  /fmu/out/vehicle_local_position \
  --output "${BAG_DIR}/run"

# ── LANDED 또는 타임아웃까지 대기 ────────────────────────────────────────────
echo "[headless] monitoring mission state..."
elapsed=0
last_state=""
while true; do
  state=$(ros2 topic echo --once /mission/state 2>/dev/null \
           | grep 'data:' | awk '{print $2}' || true)
  if [[ -n "$state" && "$state" != "$last_state" ]]; then
    echo "[headless]   state → $state"
    last_state="$state"
  fi
  if [[ "$state" == "LANDED" || "$state" == "ABORT" ]]; then
    echo "[headless] mission ended: $state"
    break
  fi
  sleep 2; elapsed=$((elapsed+2))
  if [[ $elapsed -ge $TIMEOUT_SEC ]]; then
    echo "[headless] timeout after ${TIMEOUT_SEC}s (state=${last_state:-UNKNOWN})"
    break
  fi
done

sleep 2  # bag flush
echo "[headless] done. bag: ${BAG_DIR}/run"
echo "${BAG_DIR}/run"
