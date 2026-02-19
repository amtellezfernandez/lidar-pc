from __future__ import annotations

import importlib
import platform
import sys
from dataclasses import dataclass

import cv2


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


def run_doctor(camera_index: int = 0, skip_camera: bool = False) -> tuple[list[DoctorCheck], bool]:
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

    if skip_camera:
        checks.append(DoctorCheck("camera", "warn", "skipped"))
    else:
        camera = cv2.VideoCapture(camera_index)
        if not camera.isOpened():
            checks.append(DoctorCheck("camera", "fail", f"unable to open index {camera_index}"))
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
        if check.status == "warn" and check.name in {"open3d", "trimesh", "camera"}:
            continue
        healthy = False
        break
    return checks, healthy
