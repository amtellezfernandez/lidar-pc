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

