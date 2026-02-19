from __future__ import annotations

import numpy as np

from lidar_pc.capture import should_keep_keyframe


def test_first_frame_is_kept() -> None:
    frame = np.zeros((32, 32), dtype=np.uint8)
    assert should_keep_keyframe(
        prev_gray=None,
        current_gray=frame,
        frame_index=0,
        frame_interval=4,
        pixel_delta_threshold=10.0,
        blur_score=100.0,
        blur_threshold=20.0,
    )


def test_rejects_blurry_frame() -> None:
    prev = np.full((32, 32), 20, dtype=np.uint8)
    curr = np.full((32, 32), 200, dtype=np.uint8)
    assert not should_keep_keyframe(
        prev_gray=prev,
        current_gray=curr,
        frame_index=4,
        frame_interval=4,
        pixel_delta_threshold=5.0,
        blur_score=2.0,
        blur_threshold=20.0,
    )


def test_accepts_motion_on_interval() -> None:
    prev = np.zeros((32, 32), dtype=np.uint8)
    curr = np.full((32, 32), 255, dtype=np.uint8)
    assert should_keep_keyframe(
        prev_gray=prev,
        current_gray=curr,
        frame_index=4,
        frame_interval=4,
        pixel_delta_threshold=5.0,
        blur_score=80.0,
        blur_threshold=20.0,
    )

