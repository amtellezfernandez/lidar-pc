from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import pytest

from lidar_pc.capture import _capture_from_files, calibrate_camera


def test_capture_from_files_error_includes_pattern_and_cwd(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.chdir(tmp_path)
    with pytest.raises(RuntimeError) as excinfo:
        _capture_from_files("data/*.jpg", max_frames=10)

    message = str(excinfo.value)
    assert "No images matched pattern: data/*.jpg" in message
    assert f"cwd={tmp_path}" in message


def test_calibrate_camera_error_reports_found_count(tmp_path: Path) -> None:
    # Create valid image files that intentionally do not contain a checkerboard.
    for index in range(3):
        image = np.full((240, 320, 3), 127, dtype=np.uint8)
        cv2.imwrite(str(tmp_path / f"frame_{index:02d}.jpg"), image)

    with pytest.raises(RuntimeError) as excinfo:
        calibrate_camera(
            output_path=tmp_path / "camera.json",
            board_rows=6,
            board_cols=9,
            square_mm=25.0,
            samples=5,
            input_glob=str(tmp_path / "*.jpg"),
            camera_id="laptop_cam",
        )

    message = str(excinfo.value)
    assert "Not enough checkerboard detections" in message
    assert "found 0, need >= 5 samples" in message
    assert "pattern" in message
