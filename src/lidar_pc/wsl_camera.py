from __future__ import annotations

import os
import platform
import re
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(slots=True)
class UsbIpDevice:
    busid: str
    description: str
    state: str


@dataclass(slots=True)
class WslCameraFixResult:
    attempted: bool
    success: bool
    messages: list[str]
    linux_video_devices: list[str]
    matched_busids: list[str]


_CAMERA_KEYWORDS = (
    "camera",
    "webcam",
    "imaging",
    "realsense",
    "depth",
    "lidar",
)


def is_wsl_environment() -> bool:
    release = platform.release().lower()
    if "microsoft" in release or "wsl" in release:
        return True
    try:
        return "microsoft" in Path("/proc/version").read_text(encoding="utf-8").lower()
    except OSError:
        return False


def list_linux_video_devices() -> list[str]:
    if not Path("/dev").exists():
        return []
    return sorted(str(path) for path in Path("/dev").glob("video*"))


def parse_usbipd_list(output: str) -> list[UsbIpDevice]:
    devices: list[UsbIpDevice] = []
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if line.lower().startswith("busid") or line.startswith("-"):
            continue
        if not re.match(r"^[0-9-]+", line):
            continue

        first_split = re.split(r"\s+", line, maxsplit=2)
        if len(first_split) < 3:
            continue

        busid = first_split[0]
        rest = first_split[2].strip()
        parts = re.split(r"\s{2,}", rest)
        if len(parts) < 2:
            continue

        description = " ".join(parts[:-1]).strip()
        state = parts[-1].strip()
        if description:
            devices.append(UsbIpDevice(busid=busid, description=description, state=state))
    return devices


def _run_windows_powershell(command: str, timeout_s: int = 60) -> tuple[int, str, str]:
    try:
        completed = subprocess.run(
            ["powershell.exe", "-NoProfile", "-Command", command],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except FileNotFoundError:
        return (127, "", "powershell.exe not found")
    except subprocess.TimeoutExpired:
        return (124, "", f"command timed out: {command}")
    return (completed.returncode, completed.stdout.strip(), completed.stderr.strip())


def _is_wsl_vsock_bridge_error(text: str) -> bool:
    lowered = text.lower()
    return "utilbindvsockanyport" in lowered or (
        "wsl (" in lowered and "socket failed" in lowered
    )


def _current_wsl_distro_name() -> str | None:
    value = os.environ.get("WSL_DISTRO_NAME", "").strip()
    return value or None


def _candidate_attach_commands(busid: str) -> list[str]:
    commands = [f"usbipd attach --wsl --busid {busid} --auto-attach"]
    distro_name = _current_wsl_distro_name()
    if distro_name:
        commands.append(f'usbipd attach --wsl "{distro_name}" --busid {busid} --auto-attach')
    return commands


def _find_candidate_busids(devices: list[UsbIpDevice], requested_busid: str | None) -> list[str]:
    if requested_busid:
        return [requested_busid]

    candidates: list[str] = []
    for device in devices:
        desc = device.description.lower()
        if any(keyword in desc for keyword in _CAMERA_KEYWORDS):
            candidates.append(device.busid)
    return candidates


def attempt_wsl_camera_fix(
    *,
    requested_busid: str | None = None,
    max_wait_s: float = 6.0,
) -> WslCameraFixResult:
    messages: list[str] = []
    initial_devices = list_linux_video_devices()
    if initial_devices:
        return WslCameraFixResult(
            attempted=False,
            success=True,
            messages=["linux camera device already present; no passthrough needed"],
            linux_video_devices=initial_devices,
            matched_busids=[],
        )

    if not is_wsl_environment():
        return WslCameraFixResult(
            attempted=False,
            success=False,
            messages=["not running in WSL; passthrough helper is skipped"],
            linux_video_devices=[],
            matched_busids=[],
        )

    rc, _, err = _run_windows_powershell("Get-Command usbipd -ErrorAction SilentlyContinue")
    if rc != 0:
        details = err or "Get-Command usbipd failed"
        bridge_recovery = None
        if _is_wsl_vsock_bridge_error(details):
            bridge_recovery = (
                "WSL-to-Windows bridge is unavailable (vsock error). "
                "Run 'wsl --shutdown' in Windows PowerShell, reopen WSL, and retry."
            )
        messages = [
            "usbipd is not available on Windows host.",
            "Install usbipd-win on Windows and retry.",
        ]
        if bridge_recovery:
            messages.append(bridge_recovery)
        messages.append(f"details: {details}")
        return WslCameraFixResult(
            attempted=False,
            success=False,
            messages=messages,
            linux_video_devices=[],
            matched_busids=[],
        )

    rc, out, err = _run_windows_powershell("usbipd list")
    if rc != 0:
        return WslCameraFixResult(
            attempted=True,
            success=False,
            messages=[
                "unable to enumerate usbipd devices",
                f"details: {err or out or 'usbipd list failed'}",
            ],
            linux_video_devices=[],
            matched_busids=[],
        )

    devices = parse_usbipd_list(out)
    if not devices:
        return WslCameraFixResult(
            attempted=True,
            success=False,
            messages=[
                "usbipd returned no attachable USB devices.",
                "If you use an integrated laptop camera, WSL passthrough may not be available.",
            ],
            linux_video_devices=[],
            matched_busids=[],
        )

    busids = _find_candidate_busids(devices, requested_busid=requested_busid)
    if not busids:
        return WslCameraFixResult(
            attempted=True,
            success=False,
            messages=[
                "no camera-like USB device found in usbipd list",
                "plug in a USB camera/depth camera and retry",
            ],
            linux_video_devices=[],
            matched_busids=[],
        )

    for busid in busids:
        messages.append(f"attempting usbipd bind/attach for {busid}")
        bind_rc, bind_out, bind_err = _run_windows_powershell(f"usbipd bind --busid {busid}")
        if bind_rc != 0:
            messages.append(f"bind failed for {busid}: {bind_err or bind_out or 'unknown error'}")

        attach_commands = _candidate_attach_commands(busid)
        for attach_idx, attach_command in enumerate(attach_commands):
            attach_rc, attach_out, attach_err = _run_windows_powershell(attach_command)
            if attach_rc == 0:
                if attach_idx > 0:
                    messages.append(
                        "attach succeeded after retrying with explicit WSL distro name"
                    )
                break

            details = attach_err or attach_out or "unknown error"
            messages.append(f"attach failed for {busid}: {details}")
            if attach_idx + 1 < len(attach_commands):
                messages.append(
                    "retrying attach with explicit distro: "
                    f"{attach_commands[attach_idx + 1]}"
                )

    deadline = time.time() + max_wait_s
    while time.time() < deadline:
        present = list_linux_video_devices()
        if present:
            return WslCameraFixResult(
                attempted=True,
                success=True,
                messages=messages + ["camera device detected after usbipd attach"],
                linux_video_devices=present,
                matched_busids=busids,
            )
        time.sleep(0.5)

    messages.append(
        "camera still unavailable after attach attempts. "
        "You may need to run PowerShell as Administrator and execute: "
        "usbipd bind --busid <BUSID> && usbipd attach --wsl <DISTRO> --busid <BUSID> --auto-attach"
    )
    return WslCameraFixResult(
        attempted=True,
        success=False,
        messages=messages,
        linux_video_devices=[],
        matched_busids=busids,
    )
