from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np


@dataclass(slots=True)
class Intrinsics:
    camera_id: str
    version: int
    fx: float
    fy: float
    cx: float
    cy: float

    def matrix(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    def to_dict(self) -> dict[str, float | int | str]:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict[str, object]) -> Intrinsics:
        return cls(
            camera_id=str(value["camera_id"]),
            version=int(value["version"]),
            fx=float(value["fx"]),
            fy=float(value["fy"]),
            cx=float(value["cx"]),
            cy=float(value["cy"]),
        )


@dataclass(slots=True)
class FrameRecord:
    frame_index: int
    keyframe_index: int
    relative_rgb_path: str
    t_capture_ns: int
    t_wall_ms: int
    width: int
    height: int
    blur_score: float

    def to_dict(self) -> dict[str, int | float | str]:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: dict[str, object]) -> FrameRecord:
        return cls(
            frame_index=int(value["frame_index"]),
            keyframe_index=int(value["keyframe_index"]),
            relative_rgb_path=str(value["relative_rgb_path"]),
            t_capture_ns=int(value["t_capture_ns"]),
            t_wall_ms=int(value["t_wall_ms"]),
            width=int(value["width"]),
            height=int(value["height"]),
            blur_score=float(value["blur_score"]),
        )


@dataclass(slots=True)
class TrajectoryPose:
    frame_index: int
    keyframe_index: int
    translation_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    tracking_state: str
    pose_source: str = "slam"

    def to_dict(self) -> dict[str, object]:
        return {
            "frame_index": self.frame_index,
            "keyframe_index": self.keyframe_index,
            "translation_m": list(self.translation_m),
            "quaternion_xyzw": list(self.quaternion_xyzw),
            "tracking_state": self.tracking_state,
            "pose_source": self.pose_source,
        }

    @classmethod
    def from_dict(cls, value: dict[str, object]) -> TrajectoryPose:
        translation = tuple(float(v) for v in value["translation_m"])
        quaternion = tuple(float(v) for v in value["quaternion_xyzw"])
        return cls(
            frame_index=int(value["frame_index"]),
            keyframe_index=int(value["keyframe_index"]),
            translation_m=(translation[0], translation[1], translation[2]),
            quaternion_xyzw=(quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
            tracking_state=str(value["tracking_state"]),
            pose_source=str(value.get("pose_source", "slam")),
        )
