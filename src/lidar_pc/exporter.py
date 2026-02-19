from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from jsonschema import validate

from lidar_pc.capture import load_frame_records, load_intrinsics
from lidar_pc.models import TrajectoryPose
from lidar_pc.utils import now_wall_ms, read_json, sha256_file, write_json


@dataclass(slots=True)
class ExportSummary:
    packets_dir: Path
    manifest_path: Path
    packet_count: int


def _content_type(path: Path) -> str:
    suffix = path.suffix.lower()
    if suffix in {".jpg", ".jpeg"}:
        return "image/jpeg"
    if suffix == ".png":
        return "image/png"
    return "application/octet-stream"


def _load_poses(session_dir: Path) -> list[TrajectoryPose]:
    payload = read_json(session_dir / "meta" / "trajectory.json")
    return [TrajectoryPose.from_dict(item) for item in payload["poses"]]  # type: ignore[index]


def _load_schema() -> dict[str, object]:
    schema_path = Path(__file__).parent / "schemas" / "capture_packet.v1.schema.json"
    return json.loads(schema_path.read_text(encoding="utf-8"))


def _build_packet(
    *,
    session_id: str,
    frame_record,
    pose: TrajectoryPose,
    intrinsics,
    rgb_path: Path,
    capture_mode: str,
) -> dict[str, object]:
    return {
        "schema_version": "v1",
        "session_id": session_id,
        "packet_id": f"pkt_{frame_record.keyframe_index:05d}",
        "capture_mode": capture_mode,
        "keyframe": True,
        "keyframe_reason": "interval_or_motion",
        "chunk_index": frame_record.keyframe_index,
        "t_capture_ns": frame_record.t_capture_ns,
        "t_wall_ms": frame_record.t_wall_ms,
        "frame_ids": {
            "world": "world",
            "session": "session",
            "device": "pc_camera_0",
            "camera_rgb": intrinsics.camera_id,
            "imu": "none",
        },
        "pose": {
            "translation_m": list(pose.translation_m),
            "quaternion_xyzw": list(pose.quaternion_xyzw),
            "tracking_state": pose.tracking_state,
            "pose_source": "slam",
        },
        "intrinsics": {
            "camera_id": intrinsics.camera_id,
            "version": intrinsics.version,
            "fx": intrinsics.fx,
            "fy": intrinsics.fy,
            "cx": intrinsics.cx,
            "cy": intrinsics.cy,
        },
        "payloads": {
            "rgb": {
                "uri": frame_record.relative_rgb_path,
                "content_type": _content_type(rgb_path),
                "checksum_sha256": sha256_file(rgb_path),
                "size_bytes": rgb_path.stat().st_size,
            }
        },
    }


def generate_capture_packets(
    *,
    session_dir: Path,
    capture_mode: str = "keyframes",
) -> ExportSummary:
    schema = _load_schema()
    frame_records = load_frame_records(session_dir)
    intrinsics = load_intrinsics(session_dir)
    poses = _load_poses(session_dir)
    pose_by_keyframe = {pose.keyframe_index: pose for pose in poses}

    meta = read_json(session_dir / "meta" / "session.json")
    session_id = str(meta.get("session_id", session_dir.name))

    packets_dir = session_dir / "exports" / "capture_packets"
    packets_dir.mkdir(parents=True, exist_ok=True)

    packet_checksums: list[dict[str, object]] = []
    for frame in frame_records:
        rgb_path = session_dir / frame.relative_rgb_path
        pose = pose_by_keyframe.get(frame.keyframe_index)
        if pose is None:
            pose = TrajectoryPose(
                frame_index=frame.frame_index,
                keyframe_index=frame.keyframe_index,
                translation_m=(0.0, 0.0, 0.0),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                tracking_state="limited",
            )
        packet = _build_packet(
            session_id=session_id,
            frame_record=frame,
            pose=pose,
            intrinsics=intrinsics,
            rgb_path=rgb_path,
            capture_mode=capture_mode,
        )
        validate(instance=packet, schema=schema)

        packet_path = packets_dir / f"{packet['packet_id']}.json"
        write_json(packet_path, packet)
        packet_checksums.append(
            {
                "packet_id": packet["packet_id"],
                "file": str(packet_path.relative_to(session_dir)),
                "checksum_sha256": sha256_file(packet_path),
            }
        )

    artifacts: dict[str, str] = {}
    pointcloud = session_dir / "reconstruction" / "pointcloud.ply"
    mesh = session_dir / "reconstruction" / "mesh.glb"
    if pointcloud.exists():
        artifacts["pointcloud_ply"] = str(pointcloud.relative_to(session_dir))
    if mesh.exists():
        artifacts["mesh_glb"] = str(mesh.relative_to(session_dir))

    manifest_path = session_dir / "exports" / "manifest.json"
    write_json(
        manifest_path,
        {
            "schema_version": "v1",
            "session_id": session_id,
            "created_at_ms": now_wall_ms(),
            "packet_count": len(packet_checksums),
            "packets": packet_checksums,
            "artifacts": artifacts,
        },
    )
    return ExportSummary(
        packets_dir=packets_dir,
        manifest_path=manifest_path,
        packet_count=len(packet_checksums),
    )
