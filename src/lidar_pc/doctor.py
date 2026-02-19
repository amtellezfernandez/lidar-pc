from __future__ import annotations

import importlib
import platform
import sys
from dataclasses import dataclass

import cv2

from lidar_pc.wsl_camera import attempt_wsl_camera_fix, is_wsl_environment, list_linux_video_devices


@dataclass(slots=True)
class DoctorCheck:
    name: str
    status: str
    details: str


def _check_module(module_name: str, required: bool) -> DoctorCheck:
    try:
        importlib.import_module(module_name)
    except ModuleNotFoundError:
        state = "fail" if required else "warn"
        return DoctorCheck(module_name, state, "not installed")
    return DoctorCheck(module_name, "ok", "installed")


def run_doctor(
    camera_index: int = 0,
    skip_camera: bool = False,
    auto_fix_wsl_camera: bool = False,
    wsl_busid: str | None = None,
) -> tuple[list[DoctorCheck], bool]:
    checks: list[DoctorCheck] = [
        DoctorCheck(
            "python",
            "ok",
            f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}",
        ),
        DoctorCheck("platform", "ok", f"{platform.system()} {platform.release()}"),
        _check_module("numpy", required=True),
        _check_module("cv2", required=True),
        _check_module("jsonschema", required=True),
        _check_module("open3d", required=False),
        _check_module("trimesh", required=False),
    ]
    if is_wsl_environment():
        linux_devices = ", ".join(list_linux_video_devices()) or "none"
        checks.append(DoctorCheck("wsl_video", "ok", linux_devices))

    if skip_camera:
        checks.append(DoctorCheck("camera", "warn", "skipped"))
    else:
        camera = cv2.VideoCapture(camera_index)
        if not camera.isOpened():
            checks.append(DoctorCheck("camera", "fail", f"unable to open index {camera_index}"))
            if auto_fix_wsl_camera and is_wsl_environment():
                fix = attempt_wsl_camera_fix(requested_busid=wsl_busid)
                status = "ok" if fix.success else ("warn" if fix.attempted else "fail")
                details = "; ".join(fix.messages) if fix.messages else "no diagnostic output"
                checks.append(DoctorCheck("wsl_fix", status, details))
                if fix.success:
                    camera.release()
                    camera = cv2.VideoCapture(camera_index)
                    if camera.isOpened():
                        ok, _ = camera.read()
                        if ok:
                            checks.append(
                                DoctorCheck("camera_retry", "ok", f"index {camera_index} available")
                            )
                        else:
                            checks.append(
                                DoctorCheck(
                                    "camera_retry",
                                    "fail",
                                    f"index {camera_index} opened but no frame",
                                )
                            )
                    else:
                        checks.append(
                            DoctorCheck(
                                "camera_retry",
                                "fail",
                                f"unable to open index {camera_index}",
                            )
                        )
        else:
            ok, _ = camera.read()
            if ok:
                checks.append(DoctorCheck("camera", "ok", f"index {camera_index} available"))
            else:
                checks.append(
                    DoctorCheck(
                        "camera",
                        "fail",
                        f"index {camera_index} opened but no frame",
                    )
                )
        camera.release()

    healthy = True
    for check in checks:
        if check.status == "ok":
            continue
        if check.status == "warn" and check.name in {"open3d", "trimesh", "camera", "wsl_fix"}:
            continue
        healthy = False
        break
    return checks, healthy
