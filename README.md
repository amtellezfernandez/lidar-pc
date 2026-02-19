# lidar-pc

`lidar-pc` turns a regular PC camera into a LiDAR-like scanner pipeline:
capture RGB keyframes, estimate poses with SLAM-style tracking, reconstruct a point cloud, and export `capture_packet` metadata compatible with `lidar-studio`.

## What v1 does
- Cross-platform Python CLI (WSL/Linux, Windows, macOS)
- Built-in webcam or pre-recorded images as source
- Offline reconstruction-first workflow
- Local outputs first (`PLY`, optional `GLB`, packet JSON, manifest)

## Install
```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .[dev]
```

Optional extras:
```bash
python -m pip install -e .[dev,reconstruction]
```

## Quickstart
1. Environment check:
```bash
lidar-pc doctor --skip-camera
```

2. Capture a session:
```bash
lidar-pc capture --session-id demo --mode keyframes --camera-index 0 --out outputs
```

3. Reconstruct:
```bash
lidar-pc reconstruct --session outputs/demo --quality high
```

4. Export packet metadata + manifest:
```bash
lidar-pc export --session outputs/demo --mode keyframes
```

## Command reference
```bash
lidar-pc calibrate --camera-id laptop_cam --board 6x9 --square-mm 25 --samples 20
lidar-pc capture --session-id run01 --input-glob "data/*.jpg" --out outputs
lidar-pc reconstruct --session outputs/run01 --min-inliers 30 --step-scale-m 0.1
lidar-pc export --session outputs/run01
```

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

## Limitations
- Built-in webcams are not true LiDAR sensors; depth is estimated and scale can drift.
- Reconstruction quality depends on lighting, texture, and camera motion.
- Real-time dense mapping is out of scope for v1.

