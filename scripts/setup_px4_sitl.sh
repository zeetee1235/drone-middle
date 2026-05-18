#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXTERNAL_DIR="${ROOT_DIR}/external"
PX4_DIR="${PX4_DIR:-${EXTERNAL_DIR}/PX4-Autopilot}"
AGENT_DIR="${AGENT_DIR:-${EXTERNAL_DIR}/Micro-XRCE-DDS-Agent}"
AGENT_PREFIX="${AGENT_PREFIX:-${EXTERNAL_DIR}/install/micro-xrce-dds-agent}"
PX4_REF="${PX4_REF:-main}"

BUILD_PX4=false
INSTALL_APT_DEPS=false

usage() {
  cat <<EOF
Usage: scripts/setup_px4_sitl.sh [options]

Options:
  --build-px4         Build PX4 SITL target after cloning.
  --install-apt-deps  Install native packages for Micro XRCE-DDS Agent.
  -h, --help          Show this help.

Environment:
  PX4_DIR       Default: ${PX4_DIR}
  AGENT_DIR     Default: ${AGENT_DIR}
  AGENT_PREFIX  Default: ${AGENT_PREFIX}
  PX4_REF       Default: ${PX4_REF}
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-px4)
      BUILD_PX4=true
      shift
      ;;
    --install-apt-deps)
      INSTALL_APT_DEPS=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

mkdir -p "${EXTERNAL_DIR}"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # Reuse ROS2's Fast-CDR/Fast-DDS CMake packages when native dev packages
  # are not installed globally.
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi

if [[ "${INSTALL_APT_DEPS}" == "true" ]]; then
  sudo apt update
  sudo apt install -y \
    git cmake build-essential ninja-build python3-pip \
    libfastcdr-dev libfastrtps-dev
fi

echo "[1/5] Preparing Micro XRCE-DDS Agent"
if [[ ! -d "${AGENT_DIR}/.git" ]]; then
  git clone --recursive --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "${AGENT_DIR}"
else
  git -C "${AGENT_DIR}" fetch --depth 1 origin
  git -C "${AGENT_DIR}" pull --ff-only || true
  git -C "${AGENT_DIR}" submodule update --init --recursive --depth 1
fi

echo "[2/5] Building Micro XRCE-DDS Agent"
cmake -S "${AGENT_DIR}" -B "${AGENT_DIR}/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${AGENT_PREFIX}"
cmake --build "${AGENT_DIR}/build" -j"$(nproc)"
cmake --install "${AGENT_DIR}/build"

echo "[3/5] Preparing PX4-Autopilot"
if [[ ! -d "${PX4_DIR}/.git" ]]; then
  git clone --recursive --depth 1 --branch "${PX4_REF}" https://github.com/PX4/PX4-Autopilot.git "${PX4_DIR}"
else
  git -C "${PX4_DIR}" fetch --depth 1 origin "${PX4_REF}"
  git -C "${PX4_DIR}" checkout "${PX4_REF}"
  git -C "${PX4_DIR}" pull --ff-only || true
  git -C "${PX4_DIR}" submodule update --init --recursive --depth 1
fi

echo "[4/5] Installing PX4 Python requirements for current user"
python3 -m pip install --user --break-system-packages -r "${PX4_DIR}/Tools/setup/requirements.txt"

echo "[5/5] Generating sprint-grid PX4 world"
python3 "${ROOT_DIR}/tools/make_px4_world.py" \
  --output "${PX4_DIR}/Tools/simulation/gz/worlds/sprint_grid_world_px4.sdf" \
  --world-name sprint_grid_world_px4

if [[ "${BUILD_PX4}" == "true" ]]; then
  echo "[extra] Building PX4 SITL x500 down-facing camera target"
  (
    cd "${PX4_DIR}"
    make px4_sitl gz_x500_mono_cam_down
  )
fi

cat <<EOF

PX4 SITL assets are ready.

Next terminals:
  scripts/run_micro_xrce_agent.sh
  scripts/run_px4_sitl.sh
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  ros2 launch sprint_drone sim_px4_full.launch.py
EOF
