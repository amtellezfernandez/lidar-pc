from __future__ import annotations

import argparse
import sys
from pathlib import Path

from lidar_pc.capture import calibrate_camera, capture_session
from lidar_pc.config import load_config
from lidar_pc.doctor import run_doctor
from lidar_pc.exporter import generate_capture_packets
from lidar_pc.reconstruction import run_reconstruction
from lidar_pc.tracking import run_tracking


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="lidar-pc", description="PC scanning pipeline.")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("lidar_pc.toml"),
        help="Path to configuration file.",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    doctor = sub.add_parser("doctor", help="Validate local runtime and camera access.")
    doctor.add_argument("--camera-index", type=int, default=0)
    doctor.add_argument("--skip-camera", action="store_true")

    calibrate = sub.add_parser("calibrate", help="Estimate camera intrinsics from checkerboard.")
    calibrate.add_argument("--camera-id", default="pc_rgb")
    calibrate.add_argument("--camera-index", type=int, default=0)
    calibrate.add_argument("--board", default="6x9", help="Checkerboard inner corners rowsxcols.")
    calibrate.add_argument("--square-mm", type=float, default=25.0)
    calibrate.add_argument("--samples", type=int, default=20)
    calibrate.add_argument(
        "--input-glob",
        default=None,
        help="Use calibration images instead of camera.",
    )
    calibrate.add_argument("--out", type=Path, default=None)

    capture = sub.add_parser("capture", help="Capture keyframes from camera or image set.")
    capture.add_argument("--session-id", required=True)
    capture.add_argument("--camera-index", type=int, default=0)
    capture.add_argument("--mode", choices=["keyframes", "full_stream"], default="keyframes")
    capture.add_argument("--out", type=Path, default=None, help="Session output root.")
    capture.add_argument("--max-frames", type=int, default=None)
    capture.add_argument("--duration-s", type=float, default=None)
    capture.add_argument("--input-glob", default=None)
    capture.add_argument("--intrinsics-file", type=Path, default=None)
    capture.add_argument("--camera-id", default="pc_rgb")

    reconstruct = sub.add_parser("reconstruct", help="Track poses and reconstruct point cloud.")
    reconstruct.add_argument("--session", type=Path, required=True)
    reconstruct.add_argument("--quality", choices=["high", "medium"], default=None)
    reconstruct.add_argument("--min-inliers", type=int, default=None)
    reconstruct.add_argument("--step-scale-m", type=float, default=None)

    export = sub.add_parser("export", help="Generate capture packets and manifest.")
    export.add_argument("--session", type=Path, required=True)
    export.add_argument("--mode", choices=["keyframes", "full_stream"], default="keyframes")

    return parser


def _parse_board(value: str) -> tuple[int, int]:
    try:
        left, right = value.lower().split("x", 1)
        return int(left), int(right)
    except ValueError as exc:
        raise SystemExit(
            f"Invalid --board value '{value}', expected rowsxcols (example: 6x9)."
        ) from exc


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    config = load_config(args.config if args.config and args.config.exists() else None)

    if args.command == "doctor":
        checks, healthy = run_doctor(camera_index=args.camera_index, skip_camera=args.skip_camera)
        for check in checks:
            print(f"{check.name:10} {check.status:5} {check.details}")
        return 0 if healthy else 1

    if args.command == "calibrate":
        rows, cols = _parse_board(args.board)
        output = args.out or Path("calibration") / f"{args.camera_id}.json"
        intrinsics = calibrate_camera(
            output_path=output,
            board_rows=rows,
            board_cols=cols,
            square_mm=args.square_mm,
            camera_index=args.camera_index,
            samples=args.samples,
            input_glob=args.input_glob,
            camera_id=args.camera_id,
        )
        print(
            f"calibrated {intrinsics.camera_id} fx={intrinsics.fx:.2f} fy={intrinsics.fy:.2f} "
            f"cx={intrinsics.cx:.2f} cy={intrinsics.cy:.2f} -> {output}"
        )
        return 0

    if args.command == "capture":
        output_root = args.out or Path(config.paths.output_root)
        summary = capture_session(
            session_id=args.session_id,
            output_root=output_root,
            camera_index=args.camera_index,
            capture_mode=args.mode,
            max_frames=args.max_frames or config.capture.max_frames,
            duration_s=args.duration_s,
            fps_target=config.capture.fps_target,
            resolution_width=config.capture.resolution_width,
            resolution_height=config.capture.resolution_height,
            frame_interval=config.capture.keyframe_interval,
            blur_threshold=config.capture.keyframe_blur_threshold,
            pixel_delta_threshold=config.capture.keyframe_pixel_delta_threshold,
            input_glob=args.input_glob,
            intrinsics_path=args.intrinsics_file,
            camera_id=args.camera_id,
        )
        print(
            f"captured {summary.keyframes} keyframes from "
            f"{summary.total_source_frames} source frames "
            f"-> {summary.session_dir}"
        )
        return 0

    if args.command == "reconstruct":
        session_dir = args.session
        tracking = run_tracking(
            session_dir=session_dir,
            min_inliers=args.min_inliers or config.tracking.min_inliers,
            step_scale_m=args.step_scale_m or config.tracking.step_scale_m,
        )
        recon = run_reconstruction(
            session_dir=session_dir,
            quality=args.quality or config.reconstruction.quality_profile,
        )
        print(
            f"trajectory poses={tracking.pose_count} good_ratio={tracking.good_ratio:.2f}; "
            f"pointcloud={recon.pointcloud_path} points={recon.point_count}"
        )
        if recon.mesh_path:
            print(f"mesh={recon.mesh_path}")
        return 0

    if args.command == "export":
        summary = generate_capture_packets(session_dir=args.session, capture_mode=args.mode)
        print(f"packets={summary.packet_count} manifest={summary.manifest_path}")
        return 0

    parser.print_help()
    return 2


if __name__ == "__main__":
    sys.exit(main())
