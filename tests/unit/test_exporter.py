from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

from lidar_pc.exporter import generate_capture_packets
from lidar_pc.utils import write_json, write_jsonl


def _make_session(tmp_path: Path) -> Path:
    session = tmp_path / "sess_001"
    (session / "rgb").mkdir(parents=True)
    (session / "meta").mkdir(parents=True)

    image = np.zeros((64, 64, 3), dtype=np.uint8)
    image[20:40, 20:40] = (255, 120, 10)
    cv2.imwrite(str(session / "rgb" / "frame_000000.jpg"), image)

    write_json(
        session / "meta" / "session.json",
        {
            "schema_version": "v1",
            "session_id": "sess_001",
            "capture_mode": "keyframes",
        },
    )
    write_json(
        session / "meta" / "intrinsics.json",
        {
            "camera_id": "pc_rgb",
            "version": 1,
            "fx": 700.0,
            "fy": 700.0,
            "cx": 32.0,
            "cy": 32.0,
        },
    )
    write_jsonl(
        session / "meta" / "frames.jsonl",
        [
            {
                "frame_index": 0,
                "keyframe_index": 0,
                "relative_rgb_path": "rgb/frame_000000.jpg",
                "t_capture_ns": 1,
                "t_wall_ms": 2,
                "width": 64,
                "height": 64,
                "blur_score": 100.0,
            }
        ],
    )
    write_json(
        session / "meta" / "trajectory.json",
        {
            "schema_version": "v1",
            "pose_source": "slam",
            "poses": [
                {
                    "frame_index": 0,
                    "keyframe_index": 0,
                    "translation_m": [0.0, 0.0, 0.0],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                    "tracking_state": "good",
                    "pose_source": "slam",
                }
            ],
        },
    )
    return session


def test_generate_capture_packets(tmp_path: Path) -> None:
    session = _make_session(tmp_path)
    summary = generate_capture_packets(session_dir=session, capture_mode="keyframes")
    assert summary.packet_count == 1
    assert summary.manifest_path.exists()

    packet = json.loads((summary.packets_dir / "pkt_00000.json").read_text(encoding="utf-8"))
    assert packet["schema_version"] == "v1"
    assert packet["payloads"]["rgb"]["content_type"] == "image/jpeg"
    assert len(packet["payloads"]["rgb"]["checksum_sha256"]) == 64

