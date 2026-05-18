#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-${ROOT_DIR}/external/PX4-Autopilot}"
PX4_WORLD_NAME="${PX4_WORLD_NAME:-sprint_grid_world_px4}"
PX4_MODEL_TARGET="${PX4_MODEL_TARGET:-gz_x500_mono_cam_down}"
PX4_MODEL_POSE="${PX4_MODEL_POSE:-2,19,0.72,0,0,0}"
PX4_RELAXED_PREFLIGHT="${PX4_RELAXED_PREFLIGHT:-1}"

if [[ ! -d "${PX4_DIR}" ]]; then
  cat >&2 <<EOF
PX4-Autopilot was not found at:
  ${PX4_DIR}

Prepare it first:
  scripts/setup_px4_sitl.sh --build-px4
EOF
  exit 1
fi

WORLD_OUT="${PX4_DIR}/Tools/simulation/gz/worlds/${PX4_WORLD_NAME}.sdf"
python3 "${ROOT_DIR}/tools/make_px4_world.py" \
  --output "${WORLD_OUT}" \
  --world-name "${PX4_WORLD_NAME}"

export PX4_GZ_WORLD="${PX4_WORLD_NAME}"
export PX4_GZ_MODEL_POSE="${PX4_MODEL_POSE}"
export GZ_SIM_RESOURCE_PATH="${ROOT_DIR}/src/sprint_drone/models:${PX4_DIR}/Tools/simulation/gz/models:${PX4_DIR}/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# The x500 Gazebo model publishes these simulated sensors. Force-enable them
# because PX4 persists SYS_HAS_* values across SITL runs.
export PX4_PARAM_SYS_HAS_BARO="${PX4_PARAM_SYS_HAS_BARO:-1}"
export PX4_PARAM_SYS_HAS_MAG="${PX4_PARAM_SYS_HAS_MAG:-1}"

if [[ "${PX4_RELAXED_PREFLIGHT}" == "1" ]]; then
  # Prototype-only SITL defaults. PX4's POSIX rcS applies PX4_PARAM_* after
  # the airframe file, so these override the x500 defaults without patching PX4.
  export PX4_PARAM_CBRK_SUPPLY_CHK="${PX4_PARAM_CBRK_SUPPLY_CHK:-894281}"
  export PX4_PARAM_COM_ARM_WO_GPS="${PX4_PARAM_COM_ARM_WO_GPS:-2}"
  export PX4_PARAM_NAV_DLL_ACT="${PX4_PARAM_NAV_DLL_ACT:-0}"
fi

cat <<EOF
Starting PX4 SITL:
  PX4_GZ_WORLD=${PX4_GZ_WORLD}
  PX4_GZ_MODEL_POSE=${PX4_GZ_MODEL_POSE}
  PX4_RELAXED_PREFLIGHT=${PX4_RELAXED_PREFLIGHT}
  PX4_PARAM_SYS_HAS_BARO=${PX4_PARAM_SYS_HAS_BARO}
  PX4_PARAM_SYS_HAS_MAG=${PX4_PARAM_SYS_HAS_MAG}
  make px4_sitl ${PX4_MODEL_TARGET}
EOF

cd "${PX4_DIR}"
exec make px4_sitl "${PX4_MODEL_TARGET}"
