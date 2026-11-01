"""devtools: paths, task state, evidence, ssh argv, shipped trees, host probe, port holders, gate, sim gate."""

import re
import shutil
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


def test_probe_refuses_two_robot_ports(tmp_path, monkeypatch):
    tty = tmp_path / "sys" / "class" / "tty"
    for name, vendor in [("ttyUSB0", "1a86"), ("ttyACM0", "1a86")]:
        usb = tmp_path / "sys" / "devices" / name
        (usb / "interface").mkdir(parents=True)
        (usb / "idVendor").write_text(f"{vendor}\n")
        (usb / "idProduct").write_text("7523\n")
        (tty / name).mkdir(parents=True)
        (tty / name / "device").symlink_to(usb / "interface")

    monkeypatch.setattr(devtools, "SYS_TTY", tty)
    monkeypatch.setattr(socket, "gethostname", lambda: "niko-desktop")
    with pytest.raises(RuntimeError, match=r"^more than one robot USB port: /dev/ttyACM0, /dev/ttyUSB0$"):
        devtools.probe(None)


def test_probe_reports_the_mux_when_the_tcp_connect_succeeds(tmp_path, monkeypatch):
    class Connected:
        def close(self):
            pass

    monkeypatch.setattr(devtools, "SYS_TTY", tmp_path / "no-tty")
    monkeypatch.setattr(devtools, "ROS_SETUP", tmp_path / "setup.bash")
    monkeypatch.setattr(devtools, "_toolchain", lambda: None)
    monkeypatch.setattr(socket, "gethostname", lambda: "niko-desktop")
    monkeypatch.setattr(socket, "create_connection", lambda *args, **kwargs: Connected())
    assert devtools.probe(None) == {
        "hostname": "niko-desktop",
        "toolchain": None,
        "robot_usb": None,
        "mux": True,
        "ros": False,
    }


def test_run_passes_input_to_subprocess(monkeypatch):
    calls = []

    def fake(args, **kwargs):
        calls.append((args, kwargs))
        return subprocess.CompletedProcess(args, 0, b"", b"")

    monkeypatch.setattr(devtools.subprocess, "run", fake)
    devtools.run(None, ["cat"], input=b"payload")
    assert calls[0][1]["input"] == b"payload"


def test_run_interactive_over_ssh_allocates_a_tty_and_captures_nothing(monkeypatch):
    calls = []

    def fake(args, **kwargs):
        calls.append((args, kwargs))
        return subprocess.CompletedProcess(args, 0, b"", b"")

    monkeypatch.setattr(devtools.subprocess, "run", fake)
    devtools.run("cm5", ["python3", "-V"], interactive=True)
    args, kwargs = calls[0]
    assert args == ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", "-t", "cm5", "bash -lc 'python3 -V'"]
    assert (kwargs["stdout"], kwargs["stderr"]) == (None, None)


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


def test_gate_passes_when_idle_and_rc_is_not_driving():
    clear = {"balance_state": 0, "motor_test_active": 0, "rc_valid": 1, "ch5_position": 0}
    assert devtools.gate_ok(clear) is None


def test_gate_refuses_with_the_reason_for_each_condition():
    clear = {"balance_state": 0, "motor_test_active": 0, "rc_valid": 0, "ch5_position": 1}
    assert devtools.gate_ok({**clear, "balance_state": 1}) == "balance_state=1; the balance controller is active"
    assert devtools.gate_ok({**clear, "balance_state": 2}) == "balance_state=2; the balance controller is active"
    assert devtools.gate_ok({**clear, "motor_test_active": 1}) == "motor_test_active=1; a motor test is running"
    assert devtools.gate_ok({**clear, "rc_valid": 1}) == (
        "rc_valid=1 and ch5_position=1; the transmitter may drive the robot"
    )
    assert devtools.gate_ok({k: v for k, v in clear.items() if k != "ch5_position"}) == (
        "missing signal(s): ch5_position"
    )
    assert devtools.gate_ok({**clear, "ch5_position": None}) == "missing signal(s): ch5_position"


def test_peer_maps_known_hosts_and_refuses_others(monkeypatch):
    monkeypatch.setattr(socket, "gethostname", lambda: "cm5")
    assert devtools.peer() == "desktop"
    monkeypatch.setattr(socket, "gethostname", lambda: "laptop")
    with pytest.raises(ValueError, match="unknown host laptop; add it to PEERS in scripts/devtools.py"):
        devtools.peer()


def test_is_sim_input_accepts_the_simulator_sources_and_rejects_the_rest():
    assert devtools.is_sim_input("src/sim/plant.cpp")
    assert devtools.is_sim_input("src/ES-02/OllieFOCdrive/ControlTick.cpp")
    assert devtools.is_sim_input("src/ES-02/OllieFOCdrive/src/navbot_link/pb.h")
    assert not devtools.is_sim_input("src/ES-02/OllieFOCdrive/OllieFOCdrive.ino")
    assert not devtools.is_sim_input("docs/robot/software/simulation.md")
    assert not devtools.is_sim_input("src/sim2/x.cpp")
    assert not devtools.is_sim_input("scripts/verify.py")


def test_sim_key_ignores_commits_outside_the_inputs_and_follows_input_content(repo):
    (repo / "src" / "sim").mkdir()
    (repo / "src" / "sim" / "a.cpp").write_text("int a;\n")
    git(repo, "add", "-A")
    git(repo, "commit", "-q", "-m", "sim")
    key = devtools.sim_key(git(repo, "rev-parse", "HEAD"), cwd=repo)

    (repo / "README.md").write_text("changed\n")
    git(repo, "commit", "-qam", "readme")
    assert devtools.sim_key(git(repo, "rev-parse", "HEAD"), cwd=repo) == key

    (repo / "src" / "sim" / "a.cpp").write_text("int b;\n")
    git(repo, "commit", "-qam", "sim change")
    changed = devtools.sim_key(git(repo, "rev-parse", "HEAD"), cwd=repo)
    assert changed != key
    assert len(changed) == 64


def test_sim_check_runs_once_per_input_key_and_records_only_passes(repo, tmp_path, monkeypatch):
    monkeypatch.setattr(devtools, "SIM_CACHE", tmp_path / "cache")
    (repo / "src" / "sim").mkdir()
    (repo / "src" / "sim" / "a.cpp").write_text("int a;\n")
    git(repo, "add", "-A")
    git(repo, "commit", "-q", "-m", "sim")
    passing = git(repo, "rev-parse", "HEAD")

    calls = []
    outcome = {"code": 0, "out": "3 passed in 0.10s\n"}

    def fake(host, argv, **kwargs):
        calls.append((host, argv, kwargs))
        return subprocess.CompletedProcess(argv, outcome["code"], outcome["out"], "")

    monkeypatch.setattr(devtools, "run", fake)
    assert devtools.sim_check(repo, passing) == (True, "3 passed in 0.10s")
    assert calls == [
        (None, ["python3", "-m", "pytest", "tests/simulation", "-q"], {"cwd": str(repo), "timeout": 900})
    ]
    assert (tmp_path / "cache" / devtools.sim_key(passing, cwd=repo)).exists()

    passed, detail = devtools.sim_check(repo, passing)
    assert passed is True
    assert detail.startswith("inputs unchanged")
    assert len(calls) == 1

    (repo / "src" / "sim" / "a.cpp").write_text("int b;\n")
    git(repo, "commit", "-qam", "break")
    failing = git(repo, "rev-parse", "HEAD")
    outcome.update(code=1, out="F\n1 failed in 0.20s\n")
    assert devtools.sim_check(repo, failing) == (False, "1 failed in 0.20s")
    assert not (tmp_path / "cache" / devtools.sim_key(failing, cwd=repo)).exists()

    outcome.update(code=0, out="3 passed in 0.10s\n")
    assert devtools.sim_check(repo, failing) == (True, "3 passed in 0.10s")
    assert len(calls) == 3


def test_sim_check_refuses_a_checkout_that_is_not_the_committed_sha(repo, tmp_path, monkeypatch):
    monkeypatch.setattr(devtools, "SIM_CACHE", tmp_path / "cache")
    (repo / "src" / "sim").mkdir()
    (repo / "src" / "sim" / "a.cpp").write_text("int a;\n")
    git(repo, "add", "-A")
    git(repo, "commit", "-q", "-m", "sim")
    head = git(repo, "rev-parse", "HEAD")
    calls = []
    monkeypatch.setattr(devtools, "run", lambda *args, **kwargs: calls.append(args))

    (repo / "src" / "sim" / "a.cpp").write_text("int fixed;\n")
    assert devtools.sim_check(repo, head) == (False, "uncommitted changes to simulator inputs: src/sim/a.cpp")
    git(repo, "checkout", "-q", "--", "src/sim/a.cpp")
    (repo / "tests" / "simulation").mkdir(parents=True)
    (repo / "tests" / "simulation" / "test_new.py").write_text("")
    assert devtools.sim_check(repo, head) == (False, "uncommitted changes to simulator inputs: tests/simulation/test_new.py")
    assert devtools.sim_check(repo, "0" * 40) == (False, f"{repo} is not at 00000000")
    assert calls == []
    assert not (tmp_path / "cache").exists()


def test_sim_inputs_cover_every_source_build_sh_compiles():
    if shutil.which("c++") is None:
        pytest.skip("c++ is not on PATH")
    build = (ROOT / "src" / "sim" / "build.sh").read_text(encoding="utf-8")
    sketch = "src/ES-02/OllieFOCdrive"
    sources = [p.relative_to(ROOT).as_posix() for p in (ROOT / "src" / "sim").glob("*.cpp")]
    sources += [f"{sketch}/{name}" for name in re.findall(r'"\$sketch_dir/([^"]+\.cpp)"', build)]
    nanopb = re.search(r"for source in ([^;]+);", build).group(1).split()
    sources += [f"{sketch}/src/navbot_link/{name}.c" for name in nanopb]

    deps = set()
    for source in sources:
        flags = ["-MM", "-Isrc/sim", f"-I{sketch}", source]
        argv = ["cc", *flags] if source.endswith(".c") else ["c++", "-std=c++17", *flags]
        out = subprocess.run(argv, cwd=ROOT, check=True, capture_output=True, text=True).stdout
        tokens = out.replace("\\", " ").split()[1:]  # the first token is "target:"
        deps.update((ROOT / token).resolve().relative_to(ROOT).as_posix() for token in tokens)

    missing = sorted(dep for dep in deps if not devtools.is_sim_input(dep))
    assert missing == []
    assert devtools.is_sim_input("src/sim/build.sh")
