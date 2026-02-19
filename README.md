# lidar-pc

`lidar-pc` turns a regular PC camera into a LiDAR-like scanner pipeline:
capture RGB keyframes, estimate poses with SLAM-style tracking, reconstruct a point cloud, and export `capture_packet` metadata compatible with `lidar-studio`.

## What v1 does
- Cross-platform Python CLI (WSL/Linux, Windows, macOS)
- Built-in webcam or pre-recorded images as source
- Offline reconstruction-first workflow
- Local outputs first (`PLY`, optional `GLB`, packet JSON, manifest)

## Setup (pick one)
Clone once:
```bash
gh repo clone amtellezfernandez/lidar-pc
```

### Windows PowerShell (copy/paste exactly)
```powershell
cd lidar-pc
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
```

### Windows Command Prompt (`cmd.exe`)
```bat
cd lidar-pc
python -m venv .venv
.\.venv\Scripts\activate.bat
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
```

### Linux/macOS (bash or zsh)
```bash
cd lidar-pc
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e '.[dev]'
```

### WSL (Ubuntu, Debian, etc.)
```bash
cd lidar-pc
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e '.[dev]'
```

Optional reconstruction extras:
```bash
python -m pip install -e '.[dev,reconstruction]'
```

If you get `does not appear to be a Python project`, run setup from the repo root (`cd lidar-pc`).

## Quickstart
Run everything in one command:
```bash
lidar-pc run --session-id demo --mode keyframes --camera-index 0 --out outputs
```

Input images instead of camera:
```bash
lidar-pc run --session-id demo --input-glob "data/*.jpg" --out outputs
```

Optional pre-check:
```bash
lidar-pc doctor --skip-camera
```

## Command reference
```bash
lidar-pc calibrate --camera-id laptop_cam --board 6x9 --square-mm 25 --samples 20
lidar-pc capture --session-id run01 --input-glob "data/*.jpg" --out outputs
lidar-pc capture --session-id room01 --camera-index 0 --auto-fix-wsl-camera
lidar-pc reconstruct --session outputs/run01 --min-inliers 30 --step-scale-m 0.1
lidar-pc export --session outputs/run01
lidar-pc run --session-id room02 --camera-index 0 --out outputs
```
Tip: if a session folder already exists, `capture` writes to a new folder like `outputs/<session_id>_run02`.
Always pass that exact folder to `reconstruct` and `export`.

## Output layout
```text
outputs/<session_id>/
  rgb/
  meta/
    session.json
    intrinsics.json
    frames.jsonl
    trajectory.json
  reconstruction/
    pointcloud.ply
    mesh.glb              # optional
    reconstruction.json
  exports/
    capture_packets/
      pkt_00000.json
      ...
    manifest.json
```

## Platform setup scripts
- WSL/Linux: `scripts/setup_wsl.sh`
- macOS: `scripts/setup_macos.sh`
- Windows PowerShell: `scripts/setup_windows.ps1`
- WSL camera helper: `scripts/fix_wsl_camera.sh`

## WSL Camera Notes
- `lidar-pc doctor` and `lidar-pc capture` now try WSL camera passthrough automatically by default.
- You can pin a specific USB camera with `--wsl-busid <BUSID>`.
- If auto-attach fails, run this on Windows PowerShell as Administrator:
```powershell
wsl --shutdown
usbipd list
usbipd bind --busid 2-8
usbipd attach --wsl "Ubuntu-24.04" --busid 2-8 --auto-attach
usbipd list
```
- Replace `2-8` with your actual camera BUSID from `usbipd list`. Do not type `<BUSID>` literally in PowerShell.
- If `usbipd` is missing, install it from PowerShell:
```powershell
winget install dorssel.usbipd-win
```
- If you see `UtilBindVsockAnyPort: socket failed 1`, the WSL-to-Windows bridge is unavailable. Run:
```powershell
wsl --shutdown
```
  then reopen WSL and repeat the `usbipd` commands above.
- Verify camera exposure inside WSL before capture:
```bash
ls -l /dev/video*
lidar-pc doctor --camera-index 0 --auto-fix-wsl-camera
```
- Integrated laptop cameras may not expose an attachable USB bus ID; run `lidar-pc` natively on Windows/macOS in that case.

## Limitations
- Built-in webcams are not true LiDAR sensors; depth is estimated and scale can drift.
- Reconstruction quality depends on lighting, texture, and camera motion.
- Real-time dense mapping is out of scope for v1.
