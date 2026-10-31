"""devtools: paths, task state, evidence, ssh argv, shipped trees, host probe, port holders, gate."""

import socket
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import devtools  # noqa: E402
from devtools import Acceptance, State  # noqa: E402

STATE_TEXT = """---
branch: feat/example
worktree: /w/feat-example
baseline: 0123456789abcdef0123456789abcdef01234567
status: implementing
acceptance:
  - id: tests/hil/test_example.py::test_new_signal
    tier: unattended
    status: pending
  - id: tier_b
    tier: guided
    status: skipped
    skip: "the user's words, verbatim (2026-11-02)"
---

## Contract

Body line.
"""


def git(cwd, *args):
    return subprocess.run(
        ["git", "-c", "user.name=t", "-c", "user.email=t@example.com", *args],
        cwd=cwd, check=True, capture_output=True, text=True,
    ).stdout.strip()


@pytest.fixture
def repo(tmp_path):
    path = tmp_path / "repo"
    path.mkdir()
    git(path, "init", "-q", "-b", "main", str(path))
    (path / "src").mkdir()
    (path / "src" / "a.txt").write_text("a")
    (path / "scripts").mkdir()
    (path / "scripts" / "b.py").write_text("b = 1\n")
    (path / "tests").mkdir()
    (path / "tests" / "c.py").write_text("c = 1\n")
    (path / "firmware-build-environment.json").write_text("{}")
    (path / "README.md").write_text("not shipped\n")
    git(path, "add", "-A")
    git(path, "commit", "-q", "-m", "files")
    return path


def test_slug_turns_branch_names_into_folder_names():
    assert devtools.slug("feat/Link firmware") == "feat-link-firmware"
    assert devtools.slug("feat/x_y.z") == "feat-x-y.z"


def test_paths_come_from_the_main_checkout_even_inside_a_worktree(repo):
    worktree = repo / ".claude" / "worktrees" / "feat-link-firmware"
    git(repo, "worktree", "add", "-q", "-b", "feat/link-firmware", str(worktree))
    assert devtools.main_checkout(worktree) == repo
    assert devtools.current_branch(worktree) == "feat/link-firmware"
    assert devtools.worktree_dir("feat/link firmware", main=repo) == worktree
    assert devtools.task_dir("feat/link firmware", main=repo) == repo / "work" / "feat-link-firmware"


def test_state_round_trip_keeps_the_body(tmp_path):
    path = tmp_path / "state.md"
    path.write_text(STATE_TEXT, encoding="utf-8")

    state = devtools.read_state(path)
    assert state == State(
        branch="feat/example",
        worktree="/w/feat-example",
        baseline="0123456789abcdef0123456789abcdef01234567",
        status="implementing",
        acceptance=[
            Acceptance(id="tests/hil/test_example.py::test_new_signal", tier="unattended", status="pending"),
            Acceptance(id="tier_b", tier="guided", status="skipped", skip="the user's words, verbatim (2026-11-02)"),
        ],
        body="\n## Contract\n\nBody line.\n",
    )

    state.status = "ready"
    devtools.write_state(path, state)
    again = devtools.read_state(path)
    assert again.status == "ready"
    assert again.acceptance == state.acceptance
    assert again.body == "\n## Contract\n\nBody line.\n"


def test_update_evidence_replaces_only_the_given_stages(tmp_path):
    sha = "a" * 40
    devtools.update_evidence(tmp_path, sha, "feat/x", stages={"host": {"ok": True}, "robot": {"ok": False}})
    devtools.update_evidence(tmp_path, sha, "feat/x", stages={"robot": {"ok": True}}, verify={"result": "pass"})
    assert devtools.read_evidence(tmp_path, sha) == {
        "sha": sha,
        "branch": "feat/x",
        "stages": {"host": {"ok": True}, "robot": {"ok": True}},
        "verify": {"result": "pass"},
    }
    assert devtools.read_evidence(tmp_path, "b" * 40) is None


def test_run_on_a_peer_builds_the_ssh_command(monkeypatch):
    calls = []

    def fake(args, **kwargs):
        calls.append(args)
        return subprocess.CompletedProcess(args, 0, b"", b"")

    monkeypatch.setattr(devtools.subprocess, "run", fake)
    result = devtools.run("cm5", ["python3", "-V"], cwd="/tmp/t")
    assert calls == [
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "cm5", "bash -lc 'cd /tmp/t && python3 -V'"]
    ]
    assert (result.returncode, result.stdout) == (0, "")


def test_run_locally_returns_output_and_exit_code_without_raising():
    result = devtools.run(None, ["sh", "-c", "printf hi; exit 3"])
    assert (result.returncode, result.stdout) == (3, "hi")


def test_ship_tree_extracts_once_and_keeps_five_trees(repo, tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    monkeypatch.chdir(repo)
    shas = []
    for n in range(6):
        git(repo, "commit", "-q", "--allow-empty", "-m", f"c{n}")
        shas.append(git(repo, "rev-parse", "HEAD"))
    base = tmp_path / "home" / ".cache" / "navbot-deploy"

    first = devtools.ship_tree(None, shas[0])
    assert Path(first) == base / shas[0]
    assert (base / shas[0] / "src" / "a.txt").read_text() == "a"
    assert not (base / shas[0] / "README.md").exists()

    (base / shas[0] / "keep.txt").write_text("kept")
    assert devtools.ship_tree(None, shas[0]) == first
    assert (base / shas[0] / "keep.txt").read_text() == "kept"

    for sha in shas[1:]:
        devtools.ship_tree(None, sha)
    assert sorted(p.name for p in base.iterdir()) == sorted(shas[1:])


def test_probe_reports_the_robot_port_and_the_link(tmp_path, monkeypatch):
    tty = tmp_path / "sys" / "class" / "tty"
    robot = tmp_path / "sys" / "devices" / "usb1" / "1-2"
    (robot / "1-2:1.0").mkdir(parents=True)
    (robot / "idVendor").write_text("1a86\n")
    (robot / "idProduct").write_text("7523\n")
    other = tmp_path / "sys" / "devices" / "usb2" / "2-1"
    (other / "2-1:1.0").mkdir(parents=True)
    (other / "idVendor").write_text("2341\n")
    (other / "idProduct").write_text("0043\n")
    for name, interface in [("ttyUSB0", robot / "1-2:1.0"), ("ttyACM0", other / "2-1:1.0")]:
        (tty / name).mkdir(parents=True)
        (tty / name / "device").symlink_to(interface)

    def refused(*args, **kwargs):
        raise OSError("connection refused")

    monkeypatch.setattr(devtools, "SYS_TTY", tty)
    monkeypatch.setattr(devtools, "ROS_SETUP", tmp_path / "setup.bash")
    monkeypatch.setattr(devtools, "_toolchain", lambda: "/usr/bin/arduino-cli")
    monkeypatch.setattr(socket, "gethostname", lambda: "niko-desktop")
    monkeypatch.setattr(socket, "create_connection", refused)
    assert devtools.probe(None) == {
        "hostname": "niko-desktop",
        "toolchain": "/usr/bin/arduino-cli",
        "robot_usb": "/dev/ttyUSB0",
        "mux": False,
        "ros": False,
    }


def test_port_holders_lists_processes_with_the_port_open(tmp_path, monkeypatch):
    proc = tmp_path / "proc"
    (proc / "123" / "fd").mkdir(parents=True)
    (proc / "123" / "comm").write_text("navlink\n")
    (proc / "123" / "fd" / "3").symlink_to("/dev/ttyUSB0")
    (proc / "456" / "fd").mkdir(parents=True)
    (proc / "456" / "comm").write_text("bash\n")
    (proc / "456" / "fd" / "0").symlink_to("/dev/pts/1")
    monkeypatch.setattr(devtools, "PROC", proc)
    assert devtools.port_holders("/dev/ttyUSB0") == ["123 navlink"]


def test_gate_refuses_while_balancing_testing_or_driven_by_rc():
    clear = {"balance_state": 0, "motor_test_active": 0, "rc_valid": 0, "ch5_position": 1}
    assert devtools.gate_ok(clear) is None
    assert "balance_state=1" in devtools.gate_ok({**clear, "balance_state": 1})
    assert "motor_test_active=1" in devtools.gate_ok({**clear, "motor_test_active": 1})
    assert devtools.gate_ok({**clear, "rc_valid": 1, "ch5_position": 1}) is not None
    assert devtools.gate_ok({k: v for k, v in clear.items() if k != "ch5_position"}) is not None


def test_peer_maps_known_hosts_and_refuses_others(monkeypatch):
    monkeypatch.setattr(socket, "gethostname", lambda: "cm5")
    assert devtools.peer() == "desktop"
    monkeypatch.setattr(socket, "gethostname", lambda: "laptop")
    with pytest.raises(ValueError, match="unknown host laptop; add it to PEERS in scripts/devtools.py"):
        devtools.peer()
