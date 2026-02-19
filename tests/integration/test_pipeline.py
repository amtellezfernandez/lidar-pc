from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

from lidar_pc.capture import capture_session
from lidar_pc.cli import main
from lidar_pc.exporter import generate_capture_packets
from lidar_pc.reconstruction import run_reconstruction
from lidar_pc.tracking import run_tracking


def _generate_input_images(path: Path, count: int = 8) -> None:
    path.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(1234)
    base = rng.integers(0, 255, size=(240, 320, 3), dtype=np.uint8)
    for index in range(count):
        transform = np.float32([[1, 0, index * 2], [0, 1, index]])
        shifted = cv2.warpAffine(base, transform, (320, 240))
        cv2.imwrite(str(path / f"img_{index:02d}.jpg"), shifted)


def test_end_to_end_pipeline_from_images(tmp_path: Path) -> None:
    inputs = tmp_path / "inputs"
    _generate_input_images(inputs)

    capture = capture_session(
        session_id="demo",
        output_root=tmp_path / "outputs",
        capture_mode="keyframes",
        frame_interval=1,
        blur_threshold=0.0,
        pixel_delta_threshold=0.0,
        max_frames=20,
        input_glob=str(inputs / "*.jpg"),
    )
    assert capture.keyframes >= 4

    tracking = run_tracking(session_dir=capture.session_dir, min_inliers=8, step_scale_m=0.1)
    assert tracking.pose_count >= 4

    reconstruction = run_reconstruction(session_dir=capture.session_dir, quality="medium")
    assert reconstruction.pointcloud_path.exists()
    assert reconstruction.point_count > 0

    exported = generate_capture_packets(session_dir=capture.session_dir, capture_mode="keyframes")
    assert exported.packet_count == capture.keyframes

    manifest = json.loads(exported.manifest_path.read_text(encoding="utf-8"))
    assert manifest["packet_count"] == capture.keyframes


def test_cli_run_chains_pipeline_from_images(tmp_path: Path) -> None:
    inputs = tmp_path / "inputs"
    _generate_input_images(inputs)

    exit_code = main(
        [
            "run",
            "--session-id",
            "demo",
            "--input-glob",
            str(inputs / "*.jpg"),
            "--out",
            str(tmp_path / "outputs"),
            "--max-frames",
            "20",
            "--mode",
            "keyframes",
            "--quality",
            "medium",
            "--min-inliers",
            "8",
            "--step-scale-m",
            "0.1",
        ]
    )
    assert exit_code == 0

    session_dir = tmp_path / "outputs" / "demo"
    assert (session_dir / "meta" / "frames.jsonl").exists()
    assert (session_dir / "meta" / "trajectory.json").exists()
    assert (session_dir / "reconstruction" / "pointcloud.ply").exists()
    assert (session_dir / "exports" / "manifest.json").exists()
