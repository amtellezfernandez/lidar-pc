#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f "pyproject.toml" ]]; then
  echo "Run this script from the lidar-pc repository root (where pyproject.toml exists)." >&2
  exit 1
fi

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .[dev]

echo "WSL setup complete. Activate with: source .venv/bin/activate"
echo "Then run camera diagnostics: .venv/bin/lidar-pc doctor --camera-index 0 --auto-fix-wsl-camera"
echo "If camera attach fails, install usbipd-win on Windows and retry."
