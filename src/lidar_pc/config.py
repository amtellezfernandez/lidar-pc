from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover
    import tomli as tomllib  # type: ignore[no-redef]


@dataclass(slots=True)
class CaptureConfig:
    fps_target: int = 15
    resolution_width: int = 1280
    resolution_height: int = 720
    keyframe_interval: int = 4
    keyframe_blur_threshold: float = 40.0
    keyframe_pixel_delta_threshold: float = 10.0
    max_frames: int = 240


@dataclass(slots=True)
class TrackingConfig:
    min_inliers: int = 30
    step_scale_m: float = 0.1


@dataclass(slots=True)
class ReconstructionConfig:
    quality_profile: str = "high"


@dataclass(slots=True)
class PathConfig:
    output_root: str = "outputs"


@dataclass(slots=True)
class AppConfig:
    capture: CaptureConfig
    tracking: TrackingConfig
    reconstruction: ReconstructionConfig
    paths: PathConfig


def _section(raw: dict[str, object], key: str) -> dict[str, object]:
    section = raw.get(key, {})
    return section if isinstance(section, dict) else {}


def load_config(path: Path | None) -> AppConfig:
    capture = CaptureConfig()
    tracking = TrackingConfig()
    reconstruction = ReconstructionConfig()
    paths = PathConfig()

    if path is None or not path.exists():
        return AppConfig(
            capture=capture,
            tracking=tracking,
            reconstruction=reconstruction,
            paths=paths,
        )

    raw = tomllib.loads(path.read_text(encoding="utf-8"))
    cap_raw = _section(raw, "capture")
    trk_raw = _section(raw, "tracking")
    rec_raw = _section(raw, "reconstruction")
    paths_raw = _section(raw, "paths")

    capture.fps_target = int(cap_raw.get("fps_target", capture.fps_target))
    capture.resolution_width = int(cap_raw.get("resolution_width", capture.resolution_width))
    capture.resolution_height = int(cap_raw.get("resolution_height", capture.resolution_height))
    capture.keyframe_interval = int(cap_raw.get("keyframe_interval", capture.keyframe_interval))
    capture.keyframe_blur_threshold = float(
        cap_raw.get("keyframe_blur_threshold", capture.keyframe_blur_threshold)
    )
    capture.keyframe_pixel_delta_threshold = float(
        cap_raw.get("keyframe_pixel_delta_threshold", capture.keyframe_pixel_delta_threshold)
    )
    capture.max_frames = int(cap_raw.get("max_frames", capture.max_frames))

    tracking.min_inliers = int(trk_raw.get("min_inliers", tracking.min_inliers))
    tracking.step_scale_m = float(trk_raw.get("step_scale_m", tracking.step_scale_m))

    reconstruction.quality_profile = str(
        rec_raw.get("quality_profile", reconstruction.quality_profile)
    )
    paths.output_root = str(paths_raw.get("output_root", paths.output_root))

    return AppConfig(capture=capture, tracking=tracking, reconstruction=reconstruction, paths=paths)
