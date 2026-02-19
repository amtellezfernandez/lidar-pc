#!/usr/bin/env bash
set -euo pipefail

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .[dev]

echo "WSL setup complete. Activate with: source .venv/bin/activate"
echo "Then run camera diagnostics: .venv/bin/lidar-pc doctor --camera-index 0 --auto-fix-wsl-camera"
echo "If camera attach fails, install usbipd-win on Windows and retry."
