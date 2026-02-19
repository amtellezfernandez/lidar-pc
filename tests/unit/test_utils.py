from __future__ import annotations

from pathlib import Path

from lidar_pc.utils import allocate_session_dir, sha256_file


def test_sha256_file(tmp_path: Path) -> None:
    file_path = tmp_path / "a.txt"
    file_path.write_text("hello", encoding="utf-8")
    assert (
        sha256_file(file_path)
        == "2cf24dba5fb0a30e26e83b2ac5b9e29e1b161e5c1fa7425e73043362938b9824"
    )


def test_allocate_session_dir_creates_suffix(tmp_path: Path) -> None:
    first = tmp_path / "run"
    first.mkdir()
    (first / "x.txt").write_text("x", encoding="utf-8")
    second = allocate_session_dir(tmp_path, "run")
    assert second.name == "run_run02"
