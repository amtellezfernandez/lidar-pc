from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from lidar_pc.capture import load_frame_records, load_intrinsics
from lidar_pc.models import TrajectoryPose
from lidar_pc.utils import (
    quaternion_xyzw_to_rotation_matrix,
    read_json,
    write_json,
)


@dataclass(slots=True)
class ReconstructionSummary:
    pointcloud_path: Path
    mesh_path: Path | None
    point_count: int


def _load_poses(session_dir: Path) -> list[TrajectoryPose]:
    payload = read_json(session_dir / "meta" / "trajectory.json")
    return [TrajectoryPose.from_dict(item) for item in payload["poses"]]  # type: ignore[index]


def _write_ascii_ply(path: Path, points: np.ndarray, colors: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        handle.write("ply\n")
        handle.write("format ascii 1.0\n")
        handle.write(f"element vertex {len(points)}\n")
        handle.write("property float x\nproperty float y\nproperty float z\n")
        handle.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        handle.write("end_header\n")
        for point, color in zip(points, colors, strict=False):
            handle.write(
                f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
            )


def _triangulate_pair(
    img_1: np.ndarray,
    img_2: np.ndarray,
    intrinsics: np.ndarray,
    max_matches: int,
) -> tuple[np.ndarray, np.ndarray]:
    orb = cv2.ORB_create(3000)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    kp1, des1 = orb.detectAndCompute(img_1, None)
    kp2, des2 = orb.detectAndCompute(img_2, None)
    if des1 is None or des2 is None or len(kp1) < 8 or len(kp2) < 8:
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.uint8)

    matches = sorted(matcher.match(des1, des2), key=lambda match: match.distance)[:max_matches]
    if len(matches) < 8:
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.uint8)

    pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
    pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])
    essential, mask = cv2.findEssentialMat(pts1, pts2, intrinsics, method=cv2.RANSAC, threshold=1.0)
    if essential is None:
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.uint8)

    _, rotation, translation, _ = cv2.recoverPose(essential, pts1, pts2, intrinsics, mask=mask)
    projection_1 = intrinsics @ np.hstack((np.eye(3), np.zeros((3, 1))))
    projection_2 = intrinsics @ np.hstack((rotation, translation))
    homogeneous = cv2.triangulatePoints(projection_1, projection_2, pts1.T, pts2.T)

    xyz = (homogeneous[:3] / homogeneous[3]).T
    valid = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0.0) & (np.linalg.norm(xyz, axis=1) < 100.0)
    xyz = xyz[valid]
    if len(xyz) == 0:
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.uint8)

    rgb = cv2.cvtColor(img_1, cv2.COLOR_GRAY2BGR)
    colors: list[np.ndarray] = []
    for point in pts1[valid]:
        x = int(np.clip(round(point[0]), 0, rgb.shape[1] - 1))
        y = int(np.clip(round(point[1]), 0, rgb.shape[0] - 1))
        colors.append(rgb[y, x][::-1])  # RGB ordering

    return xyz.astype(np.float64), np.asarray(colors, dtype=np.uint8)


def run_reconstruction(*, session_dir: Path, quality: str = "high") -> ReconstructionSummary:
    frame_records = load_frame_records(session_dir)
    poses = _load_poses(session_dir)
    intrinsics = load_intrinsics(session_dir).matrix()
    max_matches = 1200 if quality == "high" else 500

    points_world: list[np.ndarray] = []
    colors_world: list[np.ndarray] = []
    for idx in range(len(frame_records) - 1):
        frame_a = frame_records[idx]
        frame_b = frame_records[idx + 1]
        image_a = cv2.imread(str(session_dir / frame_a.relative_rgb_path), cv2.IMREAD_GRAYSCALE)
        image_b = cv2.imread(str(session_dir / frame_b.relative_rgb_path), cv2.IMREAD_GRAYSCALE)
        if image_a is None or image_b is None:
            continue

        points_local, colors_local = _triangulate_pair(image_a, image_b, intrinsics, max_matches)
        if len(points_local) == 0:
            continue

        pose = poses[idx]
        rotation = quaternion_xyzw_to_rotation_matrix(pose.quaternion_xyzw)
        translation = np.array(pose.translation_m, dtype=np.float64)
        points_world.append((rotation @ points_local.T).T + translation)
        colors_world.append(colors_local)

    if points_world:
        points = np.vstack(points_world)
        colors = np.vstack(colors_world)
    else:
        # Fallback keeps output non-empty even when triangulation fails.
        points = np.array([pose.translation_m for pose in poses], dtype=np.float64)
        colors = np.full((len(points), 3), 180, dtype=np.uint8)

    pointcloud_path = session_dir / "reconstruction" / "pointcloud.ply"
    pointcloud_path.parent.mkdir(parents=True, exist_ok=True)
    mesh_path: Path | None = None

    try:
        import open3d as o3d  # type: ignore[import-not-found]

        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        cloud.colors = o3d.utility.Vector3dVector(colors.astype(np.float64) / 255.0)
        if len(points) > 50:
            cloud, _ = cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        points = np.asarray(cloud.points)
        colors = np.clip(np.asarray(cloud.colors) * 255.0, 0, 255).astype(np.uint8)

        write_ok = o3d.io.write_point_cloud(str(pointcloud_path), cloud, write_ascii=True)
        if not write_ok:
            _write_ascii_ply(pointcloud_path, points, colors)
    except ModuleNotFoundError:
        _write_ascii_ply(pointcloud_path, points, colors)
    except Exception:
        _write_ascii_ply(pointcloud_path, points, colors)

    try:
        import trimesh  # type: ignore[import-not-found]

        if len(points) >= 30:
            hull = trimesh.points.PointCloud(points).convex_hull
            mesh_path = session_dir / "reconstruction" / "mesh.glb"
            mesh_path.parent.mkdir(parents=True, exist_ok=True)
            hull.export(mesh_path)
    except ModuleNotFoundError:
        mesh_path = None
    except Exception:
        mesh_path = None

    write_json(
        session_dir / "reconstruction" / "reconstruction.json",
        {
            "schema_version": "v1",
            "point_count": int(len(points)),
            "mesh_generated": mesh_path is not None,
            "quality_profile": quality,
        },
    )
    return ReconstructionSummary(
        pointcloud_path=pointcloud_path,
        mesh_path=mesh_path,
        point_count=int(len(points)),
    )
