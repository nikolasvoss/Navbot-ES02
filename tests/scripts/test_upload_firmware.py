from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts/upload_firmware.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("upload_firmware", SCRIPT)
UPLOAD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(UPLOAD)

PORT = SimpleNamespace(vid=0x1A86, pid=0x7523, device="/dev/ttyUSB9", description="stub CH340")


@pytest.fixture
def build_dir(tmp_path):
    for name, _ in UPLOAD.FLASH_LAYOUT:
        (tmp_path / name).write_bytes(b"\x00")
    return tmp_path


def run_main(monkeypatch, build_dir, tools, results=None):
    """Run main() with only `tools` in PATH; return its exit status and the commands it ran."""
    commands = []
    results = iter(results or [])

    def call(command):
        commands.append(command)
        return next(results, 0)

    monkeypatch.setattr(UPLOAD.shutil, "which", lambda name: f"/stub/{name}" if name in tools else None)
    monkeypatch.setattr(UPLOAD.list_ports, "comports", lambda: [PORT])
    monkeypatch.setattr(UPLOAD.subprocess, "call", call)
    monkeypatch.setattr(sys, "argv", ["upload_firmware.py", str(build_dir)])
    return UPLOAD.main(), commands


def test_arduino_cli_is_preferred_when_both_tools_exist(monkeypatch, build_dir):
    status, commands = run_main(monkeypatch, build_dir, {"arduino-cli", "esptool"})
    assert status == 0
    assert [command[:2] for command in commands] == [["arduino-cli", "upload"]]
    assert commands[0][commands[0].index("--port") + 1] == PORT.device


def test_esptool_fallback_erases_otadata_without_reset_then_writes_all_images(monkeypatch, build_dir):
    status, commands = run_main(monkeypatch, build_dir, {"esptool"})
    assert status == 0
    base = ["esptool", "--chip", "esp32s3", "--port", PORT.device]
    assert commands == [
        base + ["--after", "no_reset", "erase_region", "0xe000", "0x2000"],
        base + [
            "write_flash",
            "0x0", str(build_dir / "OllieFOCdrive.ino.bootloader.bin"),
            "0x8000", str(build_dir / "OllieFOCdrive.ino.partitions.bin"),
            "0x10000", str(build_dir / "OllieFOCdrive.ino.bin"),
        ],
    ]


def test_failed_erase_skips_the_write_and_returns_its_status(monkeypatch, build_dir, capsys):
    status, commands = run_main(monkeypatch, build_dir, {"esptool"}, results=[2])
    assert status == 2
    assert len(commands) == 1
    assert "pipx install esptool" in capsys.readouterr().err


def test_missing_tools_are_reported_before_touching_the_port(monkeypatch, build_dir):
    with pytest.raises(SystemExit) as exit_info:
        run_main(monkeypatch, build_dir, set())
    assert exit_info.value.code == 2


def test_missing_build_files_are_reported(monkeypatch, build_dir):
    (build_dir / "OllieFOCdrive.ino.bin").unlink()
    with pytest.raises(SystemExit) as exit_info:
        run_main(monkeypatch, build_dir, {"esptool"})
    assert exit_info.value.code == 2
