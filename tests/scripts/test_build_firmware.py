from __future__ import annotations

import importlib.util
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts/build_firmware.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = importlib.util.spec_from_file_location("build_firmware", SCRIPT)
BUILD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILD)

NOW = datetime(2026, 10, 30, 14, 5)


def test_clean_revision_has_no_dirty_marker():
    assert BUILD.build_id("d9802212", False, NOW) == "d9802212 2026-10-30 14:05"


def test_dirty_revision_gets_dirty_marker():
    assert BUILD.build_id("d9802212", True, NOW) == "d9802212-dirty 2026-10-30 14:05"


def test_missing_revision_says_nogit_and_ignores_dirty():
    assert BUILD.build_id(None, True, NOW) == "nogit 2026-10-30 14:05"


def test_thirty_two_characters_is_accepted():
    text = BUILD.build_id("abcdef123", True, NOW)
    assert text == "abcdef123-dirty 2026-10-30 14:05"
    assert len(text) == 32


def test_longer_than_thirty_two_characters_is_refused():
    with pytest.raises(ValueError, match="limit is 32"):
        BUILD.build_id("abcdef1234", True, NOW)


class FakeGit:
    """Stands in for `git -C <sketch> ...`; `answers` maps the git subcommand to stdout, or None for failure."""

    def __init__(self, answers):
        self.answers = answers
        self.calls = []

    def __call__(self, command, **kwargs):
        self.calls.append(command)
        subcommand = command[3]
        output = self.answers.get(subcommand)
        if output is None:
            return subprocess.CompletedProcess(command, 128, stdout="", stderr="fatal")
        return subprocess.CompletedProcess(command, 0, stdout=output, stderr="")


def test_header_has_revision_and_dirty_flag(monkeypatch, tmp_path):
    fake = FakeGit({"rev-parse": "d9802212\n", "status": " M OllieFOCdrive.ino\n"})
    monkeypatch.setattr(BUILD.subprocess, "run", fake)

    text = BUILD.write_build_id_header(tmp_path, NOW)

    assert text == "d9802212-dirty 2026-10-30 14:05"
    assert (tmp_path / "build_id.h").read_text(encoding="utf-8") == (
        "// Written by scripts/build_firmware.py before each build. Not in git.\n"
        '#define NAVBOT_BUILD_ID "d9802212-dirty 2026-10-30 14:05"\n'
    )
    assert fake.calls[0] == ["git", "-C", str(tmp_path), "rev-parse", "--short=8", "HEAD"]
    assert fake.calls[1] == ["git", "-C", str(tmp_path), "status", "--porcelain", "--untracked-files=no"]


def test_header_of_clean_checkout_has_no_dirty_flag(monkeypatch, tmp_path):
    fake = FakeGit({"rev-parse": "d9802212\n", "status": ""})
    monkeypatch.setattr(BUILD.subprocess, "run", fake)

    assert BUILD.write_build_id_header(tmp_path, NOW) == "d9802212 2026-10-30 14:05"
    assert '#define NAVBOT_BUILD_ID "d9802212 2026-10-30 14:05"' in (tmp_path / "build_id.h").read_text(encoding="utf-8")


def test_header_without_git_says_nogit(monkeypatch, tmp_path):
    fake = FakeGit({})
    monkeypatch.setattr(BUILD.subprocess, "run", fake)

    assert BUILD.write_build_id_header(tmp_path, NOW) == "nogit 2026-10-30 14:05"
    assert '#define NAVBOT_BUILD_ID "nogit 2026-10-30 14:05"' in (tmp_path / "build_id.h").read_text(encoding="utf-8")
    assert len(fake.calls) == 1


def test_header_without_git_executable_says_nogit(monkeypatch, tmp_path):
    def missing_git(command, **kwargs):
        raise FileNotFoundError("git")

    monkeypatch.setattr(BUILD.subprocess, "run", missing_git)

    assert BUILD.write_build_id_header(tmp_path, NOW) == "nogit 2026-10-30 14:05"
