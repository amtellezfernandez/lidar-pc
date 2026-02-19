from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from lidar_pc.capture import capture_session
from lidar_pc.reconstruction import run_reconstruction
from lidar_pc.tracking import run_tracking


def _generate_input_images(path: Path, count: int = 8) -> None:
    path.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(2026)
    base = rng.integers(0, 255, size=(240, 320, 3), dtype=np.uint8)
    for index in range(count):
        transform = np.float32([[1, 0, index * 2], [0, 1, index]])
        shifted = cv2.warpAffine(base, transform, (320, 240))
        cv2.imwrite(str(path / f"img_{index:02d}.jpg"), shifted)


def test_reconstruction_falls_back_to_ascii_ply_when_open3d_write_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
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
    run_tracking(session_dir=capture.session_dir, min_inliers=8, step_scale_m=0.1)

    class FakePointCloud:
        def __init__(self) -> None:
            self.points: np.ndarray = np.empty((0, 3), dtype=np.float64)
            self.colors: np.ndarray = np.empty((0, 3), dtype=np.float64)

        def remove_statistical_outlier(
            self,
            nb_neighbors: int,
            std_ratio: float,
        ) -> tuple["FakePointCloud", None]:
            return self, None

    fake_o3d = SimpleNamespace(
        geometry=SimpleNamespace(PointCloud=FakePointCloud),
        utility=SimpleNamespace(Vector3dVector=lambda values: np.asarray(values)),
        io=SimpleNamespace(write_point_cloud=lambda path, cloud, write_ascii=True: False),
    )
    monkeypatch.setitem(sys.modules, "open3d", fake_o3d)

    reconstruction = run_reconstruction(session_dir=capture.session_dir, quality="high")
    assert reconstruction.pointcloud_path.exists()
    assert reconstruction.point_count > 0
    assert reconstruction.pointcloud_path.read_text(encoding="utf-8").startswith("ply\n")
