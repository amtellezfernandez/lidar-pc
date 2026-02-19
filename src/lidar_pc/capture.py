from __future__ import annotations

import glob
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from lidar_pc.models import FrameRecord, Intrinsics
from lidar_pc.utils import (
    allocate_session_dir,
    ensure_dir,
    now_capture_ns,
    now_wall_ms,
    read_json,
    read_jsonl,
    write_json,
    write_jsonl,
)


@dataclass(slots=True)
class CaptureSummary:
    session_dir: Path
    total_source_frames: int
    keyframes: int


def variance_of_laplacian(gray: np.ndarray) -> float:
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def should_keep_keyframe(
    prev_gray: np.ndarray | None,
    current_gray: np.ndarray,
    frame_index: int,
    frame_interval: int,
    pixel_delta_threshold: float,
    blur_score: float,
    blur_threshold: float,
) -> bool:
    if frame_index == 0:
        return True
    if blur_score < blur_threshold:
        return False
    if prev_gray is None:
        return frame_index % max(frame_interval, 1) == 0
    delta = float(np.mean(cv2.absdiff(prev_gray, current_gray)))
    return frame_index % max(frame_interval, 1) == 0 and delta >= pixel_delta_threshold


def _infer_intrinsics(width: int, height: int, camera_id: str) -> Intrinsics:
    focal = float(max(width, height))
    return Intrinsics(
        camera_id=camera_id,
        version=1,
        fx=focal,
        fy=focal,
        cx=width / 2.0,
        cy=height / 2.0,
    )


def _load_intrinsics(path: Path | None, width: int, height: int, camera_id: str) -> Intrinsics:
    if path is not None and path.exists():
        payload = read_json(path)
        return Intrinsics.from_dict(payload)
    return _infer_intrinsics(width=width, height=height, camera_id=camera_id)


def load_frame_records(session_dir: Path) -> list[FrameRecord]:
    rows = read_jsonl(session_dir / "meta" / "frames.jsonl")
    return [FrameRecord.from_dict(row) for row in rows]


def load_intrinsics(session_dir: Path) -> Intrinsics:
    return Intrinsics.from_dict(read_json(session_dir / "meta" / "intrinsics.json"))


def _capture_from_camera(
    camera_index: int,
    max_frames: int,
    duration_s: float | None,
    width: int,
    height: int,
    fps_target: int,
) -> list[np.ndarray]:
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Unable to open camera index {camera_index}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps_target)

    frames: list[np.ndarray] = []
    started = time.time()
    frame_period = 1.0 / max(fps_target, 1)
    try:
        while len(frames) < max_frames:
            if duration_s is not None and (time.time() - started) >= duration_s:
                break
            ok, frame = cap.read()
            if not ok:
                continue
            frames.append(frame)
            time.sleep(frame_period)
    finally:
        cap.release()
    return frames


def _capture_from_files(pattern: str, max_frames: int) -> list[np.ndarray]:
    paths = sorted(Path(p) for p in glob.glob(pattern))
    if not paths:
        raise RuntimeError(f"No images matched pattern: {pattern}")

    frames: list[np.ndarray] = []
    for path in paths[:max_frames]:
        frame = cv2.imread(str(path))
        if frame is None:
            continue
        frames.append(frame)

    if not frames:
        raise RuntimeError("No readable input images were found.")
    return frames


def capture_session(
    *,
    session_id: str,
    output_root: Path,
    camera_index: int = 0,
    capture_mode: str = "keyframes",
    max_frames: int = 240,
    duration_s: float | None = None,
    fps_target: int = 15,
    resolution_width: int = 1280,
    resolution_height: int = 720,
    frame_interval: int = 4,
    blur_threshold: float = 40.0,
    pixel_delta_threshold: float = 10.0,
    input_glob: str | None = None,
    intrinsics_path: Path | None = None,
    camera_id: str = "pc_rgb",
) -> CaptureSummary:
    session_dir = allocate_session_dir(output_root=output_root, session_id=session_id)
    rgb_dir = ensure_dir(session_dir / "rgb")
    meta_dir = ensure_dir(session_dir / "meta")

    if input_glob:
        source_frames = _capture_from_files(pattern=input_glob, max_frames=max_frames)
    else:
        source_frames = _capture_from_camera(
            camera_index=camera_index,
            max_frames=max_frames,
            duration_s=duration_s,
            width=resolution_width,
            height=resolution_height,
            fps_target=fps_target,
        )

    records: list[FrameRecord] = []
    previous_keyframe_gray: np.ndarray | None = None
    for frame_index, frame in enumerate(source_frames):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = variance_of_laplacian(gray)
        keep = capture_mode == "full_stream" or should_keep_keyframe(
            prev_gray=previous_keyframe_gray,
            current_gray=gray,
            frame_index=frame_index,
            frame_interval=frame_interval,
            pixel_delta_threshold=pixel_delta_threshold,
            blur_score=blur,
            blur_threshold=blur_threshold,
        )
        if not keep:
            continue

        keyframe_index = len(records)
        filename = f"frame_{keyframe_index:06d}.jpg"
        frame_path = rgb_dir / filename
        cv2.imwrite(str(frame_path), frame)
        records.append(
            FrameRecord(
                frame_index=frame_index,
                keyframe_index=keyframe_index,
                relative_rgb_path=f"rgb/{filename}",
                t_capture_ns=now_capture_ns(),
                t_wall_ms=now_wall_ms(),
                width=int(frame.shape[1]),
                height=int(frame.shape[0]),
                blur_score=blur,
            )
        )
        previous_keyframe_gray = gray

    if not records:
        raise RuntimeError(
            "No keyframes were kept. Lower blur/pixel thresholds or use --mode full_stream."
        )

    intrinsics = _load_intrinsics(
        path=intrinsics_path,
        width=records[0].width,
        height=records[0].height,
        camera_id=camera_id,
    )
    write_json(meta_dir / "intrinsics.json", intrinsics.to_dict())
    write_jsonl(meta_dir / "frames.jsonl", [record.to_dict() for record in records])
    write_json(
        meta_dir / "session.json",
        {
            "schema_version": "v1",
            "session_id": session_dir.name,
            "capture_mode": capture_mode,
            "source": "images" if input_glob else "camera",
            "camera_index": camera_index,
            "fps_target": fps_target,
            "max_frames": max_frames,
            "keyframe_count": len(records),
            "created_at_ms": now_wall_ms(),
        },
    )
    return CaptureSummary(
        session_dir=session_dir,
        total_source_frames=len(source_frames),
        keyframes=len(records),
    )


def calibrate_camera(
    *,
    output_path: Path,
    board_rows: int,
    board_cols: int,
    square_mm: float,
    camera_index: int = 0,
    samples: int = 20,
    input_glob: str | None = None,
    camera_id: str = "pc_rgb",
) -> Intrinsics:
    objpoints: list[np.ndarray] = []
    imgpoints: list[np.ndarray] = []
    pattern = (board_cols, board_rows)
    objp = np.zeros((board_rows * board_cols, 3), np.float32)
    grid = np.mgrid[0:board_cols, 0:board_rows].T.reshape(-1, 2)
    objp[:, :2] = grid * (square_mm / 1000.0)

    source_frames: list[np.ndarray]
    if input_glob:
        source_frames = _capture_from_files(pattern=input_glob, max_frames=samples * 5)
    else:
        source_frames = _capture_from_camera(
            camera_index=camera_index,
            max_frames=samples * 10,
            duration_s=None,
            width=1280,
            height=720,
            fps_target=20,
        )

    image_shape: tuple[int, int] | None = None
    for frame in source_frames:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern, None)
        if not found:
            continue
        image_shape = (gray.shape[1], gray.shape[0])
        refined = cv2.cornerSubPix(
            gray,
            corners,
            winSize=(11, 11),
            zeroZone=(-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        objpoints.append(objp)
        imgpoints.append(refined)
        if len(objpoints) >= samples:
            break

    if len(objpoints) < 5 or image_shape is None:
        raise RuntimeError(
            "Not enough checkerboard detections for calibration (need >= 5 samples)."
        )

    _, camera_matrix, _, _, _ = cv2.calibrateCamera(objpoints, imgpoints, image_shape, None, None)
    intrinsics = Intrinsics(
        camera_id=camera_id,
        version=1,
        fx=float(camera_matrix[0, 0]),
        fy=float(camera_matrix[1, 1]),
        cx=float(camera_matrix[0, 2]),
        cy=float(camera_matrix[1, 2]),
    )
    write_json(output_path, intrinsics.to_dict())
    return intrinsics
