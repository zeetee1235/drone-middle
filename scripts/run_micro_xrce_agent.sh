#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCAL_AGENT="${ROOT_DIR}/external/install/micro-xrce-dds-agent/bin/MicroXRCEAgent"
PORT="${PORT:-8888}"

if [[ -x "${LOCAL_AGENT}" ]]; then
  export LD_LIBRARY_PATH="${ROOT_DIR}/external/install/micro-xrce-dds-agent/lib:${LD_LIBRARY_PATH:-}"
  exec "${LOCAL_AGENT}" udp4 -p "${PORT}"
fi

if command -v MicroXRCEAgent >/dev/null 2>&1; then
  exec MicroXRCEAgent udp4 -p "${PORT}"
fi

cat >&2 <<EOF
MicroXRCEAgent was not found.

Build it first:
  scripts/setup_px4_sitl.sh --install-apt-deps

Or set PATH so MicroXRCEAgent is visible.
EOF
exit 1
