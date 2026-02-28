#!/usr/bin/env bash
set -euo pipefail

if [[ ! -x ".venv/bin/lidar-pc" ]]; then
  echo "Missing .venv/bin/lidar-pc. Run scripts/setup_wsl.sh first."
  exit 1
fi

CAMERA_INDEX="${1:-0}"
ALLOW_BIND_FLAG="${2:-}"

CMD=(doctor --camera-index "${CAMERA_INDEX}" --auto-fix-wsl-camera)
if [[ "${ALLOW_BIND_FLAG}" == "--allow-wsl-bind" ]]; then
  CMD+=(--allow-wsl-bind)
fi

.venv/bin/lidar-pc "${CMD[@]}"
