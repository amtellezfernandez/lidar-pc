from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from lidar_pc.capture import load_frame_records, load_intrinsics
from lidar_pc.models import TrajectoryPose
from lidar_pc.utils import rotation_matrix_to_quaternion_xyzw, write_json


@dataclass(slots=True)
class TrackingSummary:
    trajectory_path: Path
    pose_count: int
    good_ratio: float


def _tracking_state(inliers: int, min_inliers: int) -> str:
    if inliers >= min_inliers * 2:
        return "good"
    if inliers >= min_inliers:
        return "limited"
    return "lost"


def run_tracking(
    *,
    session_dir: Path,
    min_inliers: int = 30,
    step_scale_m: float = 0.1,
) -> TrackingSummary:
    frame_records = load_frame_records(session_dir)
    intrinsics = load_intrinsics(session_dir)
    k = intrinsics.matrix()

    orb = cv2.ORB_create(2000)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    world_rotation = np.eye(3, dtype=np.float64)
    world_translation = np.zeros(3, dtype=np.float64)
    poses: list[TrajectoryPose] = [
        TrajectoryPose(
            frame_index=frame_records[0].frame_index,
            keyframe_index=frame_records[0].keyframe_index,
            translation_m=(0.0, 0.0, 0.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            tracking_state="good",
        )
    ]

    for idx in range(1, len(frame_records)):
        previous = frame_records[idx - 1]
        current = frame_records[idx]

        prev_img = cv2.imread(str(session_dir / previous.relative_rgb_path), cv2.IMREAD_GRAYSCALE)
        curr_img = cv2.imread(str(session_dir / current.relative_rgb_path), cv2.IMREAD_GRAYSCALE)
        if prev_img is None or curr_img is None:
            poses.append(
                TrajectoryPose(
                    frame_index=current.frame_index,
                    keyframe_index=current.keyframe_index,
                    translation_m=tuple(world_translation.tolist()),
                    quaternion_xyzw=rotation_matrix_to_quaternion_xyzw(world_rotation),
                    tracking_state="lost",
                )
            )
            continue

        kp1, des1 = orb.detectAndCompute(prev_img, None)
        kp2, des2 = orb.detectAndCompute(curr_img, None)
        if des1 is None or des2 is None or len(kp1) < 8 or len(kp2) < 8:
            poses.append(
                TrajectoryPose(
                    frame_index=current.frame_index,
                    keyframe_index=current.keyframe_index,
                    translation_m=tuple(world_translation.tolist()),
                    quaternion_xyzw=rotation_matrix_to_quaternion_xyzw(world_rotation),
                    tracking_state="lost",
                )
            )
            continue

        matches = sorted(matcher.match(des1, des2), key=lambda match: match.distance)
        if len(matches) < 8:
            poses.append(
                TrajectoryPose(
                    frame_index=current.frame_index,
                    keyframe_index=current.keyframe_index,
                    translation_m=tuple(world_translation.tolist()),
                    quaternion_xyzw=rotation_matrix_to_quaternion_xyzw(world_rotation),
                    tracking_state="lost",
                )
            )
            continue

        points_1 = np.float32([kp1[match.queryIdx].pt for match in matches])
        points_2 = np.float32([kp2[match.trainIdx].pt for match in matches])
        essential, mask = cv2.findEssentialMat(
            points_1,
            points_2,
            k,
            method=cv2.RANSAC,
            threshold=1.0,
        )
        if essential is None:
            poses.append(
                TrajectoryPose(
                    frame_index=current.frame_index,
                    keyframe_index=current.keyframe_index,
                    translation_m=tuple(world_translation.tolist()),
                    quaternion_xyzw=rotation_matrix_to_quaternion_xyzw(world_rotation),
                    tracking_state="lost",
                )
            )
            continue

        inliers, rel_rotation, rel_translation, _ = cv2.recoverPose(
            essential,
            points_1,
            points_2,
            k,
            mask=mask,
        )
        inliers_count = int(inliers)
        state = _tracking_state(inliers=inliers_count, min_inliers=min_inliers)
        if state != "lost":
            rel_t = rel_translation.reshape(3)
            norm = float(np.linalg.norm(rel_t))
            if norm > 1e-8:
                rel_t = rel_t / norm * step_scale_m
            world_translation = world_translation + world_rotation @ rel_t
            world_rotation = world_rotation @ rel_rotation

        poses.append(
            TrajectoryPose(
                frame_index=current.frame_index,
                keyframe_index=current.keyframe_index,
                translation_m=tuple(float(v) for v in world_translation.tolist()),
                quaternion_xyzw=rotation_matrix_to_quaternion_xyzw(world_rotation),
                tracking_state=state,
            )
        )

    good_count = sum(1 for pose in poses if pose.tracking_state == "good")
    trajectory_path = session_dir / "meta" / "trajectory.json"
    write_json(
        trajectory_path,
        {
            "schema_version": "v1",
            "pose_source": "slam",
            "poses": [pose.to_dict() for pose in poses],
            "metrics": {
                "pose_count": len(poses),
                "good_ratio": good_count / max(len(poses), 1),
            },
        },
    )
    return TrackingSummary(
        trajectory_path=trajectory_path,
        pose_count=len(poses),
        good_ratio=good_count / max(len(poses), 1),
    )
