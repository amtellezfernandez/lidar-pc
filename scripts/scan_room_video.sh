#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: scripts/scan_room_video.sh <video_path> [session_id] [video_frame_step]"
  exit 1
fi

if [[ ! -x ".venv/bin/lidar-pc" ]]; then
  echo "Missing .venv/bin/lidar-pc. Run scripts/setup_wsl.sh first."
  exit 1
fi

VIDEO_PATH="$1"
SESSION_ID="${2:-room_$(date +%Y%m%d_%H%M%S)}"
FRAME_STEP="${3:-3}"

.venv/bin/lidar-pc run \
  --session-id "${SESSION_ID}" \
  --input-video "${VIDEO_PATH}" \
  --video-frame-step "${FRAME_STEP}" \
  --mode keyframes \
  --quality medium \
  --min-inliers 8 \
  --step-scale-m 0.1 \
  --out outputs

echo "Scan completed: outputs/${SESSION_ID}"
