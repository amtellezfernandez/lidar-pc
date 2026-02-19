from __future__ import annotations

from lidar_pc import wsl_camera


def test_parse_usbipd_list() -> None:
    sample = """
BUSID  VID:PID    DEVICE                                                        STATE
2-7    046d:0825  Logitech Webcam C270                                          Not shared
3-1    8087:0033  Intel Bluetooth                                                Not shared
1-4    8086:0b3a  Intel RealSense D435i                                          Shared
"""
    devices = wsl_camera.parse_usbipd_list(sample)
    assert len(devices) == 3
    assert devices[0].busid == "2-7"
    assert "Webcam" in devices[0].description
    assert devices[0].state == "Not shared"


def test_attempt_fix_skips_when_not_wsl(monkeypatch) -> None:
    monkeypatch.setattr(wsl_camera, "is_wsl_environment", lambda: False)
    monkeypatch.setattr(wsl_camera, "list_linux_video_devices", lambda: [])
    result = wsl_camera.attempt_wsl_camera_fix()
    assert not result.attempted
    assert not result.success


def test_attempt_fix_reports_missing_usbipd(monkeypatch) -> None:
    monkeypatch.setattr(wsl_camera, "is_wsl_environment", lambda: True)
    monkeypatch.setattr(wsl_camera, "list_linux_video_devices", lambda: [])
    monkeypatch.setattr(
        wsl_camera,
        "_run_windows_powershell",
        lambda command, timeout_s=60: (1, "", "usbipd not found"),
    )
    result = wsl_camera.attempt_wsl_camera_fix()
    assert not result.success
    assert any("usbipd is not available" in message for message in result.messages)


def test_attempt_fix_reports_vsock_bridge_recovery(monkeypatch) -> None:
    monkeypatch.setattr(wsl_camera, "is_wsl_environment", lambda: True)
    monkeypatch.setattr(wsl_camera, "list_linux_video_devices", lambda: [])
    monkeypatch.setattr(
        wsl_camera,
        "_run_windows_powershell",
        lambda command, timeout_s=60: (
            1,
            "",
            "<3>WSL (2903 - ) ERROR: UtilBindVsockAnyPort:307: socket failed 1",
        ),
    )
    result = wsl_camera.attempt_wsl_camera_fix()
    assert not result.success
    assert any("wsl --shutdown" in message for message in result.messages)


def test_attempt_fix_retries_attach_with_distro_name(monkeypatch) -> None:
    monkeypatch.setattr(wsl_camera, "is_wsl_environment", lambda: True)
    device_states = [[], ["/dev/video0"]]
    monkeypatch.setattr(
        wsl_camera,
        "list_linux_video_devices",
        lambda: device_states.pop(0) if device_states else ["/dev/video0"],
    )
    monkeypatch.setenv("WSL_DISTRO_NAME", "Ubuntu-24.04")

    calls: list[str] = []
    sample = """
BUSID  VID:PID    DEVICE                                                        STATE
2-7    046d:0825  Logitech Webcam C270                                          Not shared
"""

    def fake_run(command: str, timeout_s: int = 60) -> tuple[int, str, str]:
        calls.append(command)
        if command == "Get-Command usbipd -ErrorAction SilentlyContinue":
            return (0, "usbipd", "")
        if command == "usbipd list":
            return (0, sample, "")
        if command == "usbipd bind --busid 2-7":
            return (0, "", "")
        if command == "usbipd attach --wsl --busid 2-7 --auto-attach":
            return (1, "", "option '--wsl' requires an argument")
        if command == 'usbipd attach --wsl "Ubuntu-24.04" --busid 2-7 --auto-attach':
            return (0, "", "")
        return (1, "", f"unexpected command: {command}")

    monkeypatch.setattr(wsl_camera, "_run_windows_powershell", fake_run)
    result = wsl_camera.attempt_wsl_camera_fix(max_wait_s=0.01)
    assert result.success
    assert any("retrying attach with explicit distro" in message for message in result.messages)
    assert 'usbipd attach --wsl "Ubuntu-24.04" --busid 2-7 --auto-attach' in calls
