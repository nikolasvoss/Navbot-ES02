"""deploy: host selection, the steps, the gate, the boot check, tier A and the exit codes, with fake hosts.

The hosts are fakes: devtools.run, probe, peer and ship_tree are replaced, so no ssh, rsync, serial port or robot is used.
"""

import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import deploy  # noqa: E402
import devtools  # noqa: E402

BASE_LOCAL = {"hostname": "niko-desktop", "toolchain": None, "robot_usb": None, "mux": False, "ros": False}
BASE_PEER = {"hostname": "cm5", "toolchain": None, "robot_usb": None, "mux": False, "ros": True}
LOCAL_PROBE = {**BASE_LOCAL, "toolchain": "/usr/bin/arduino-cli"}
PEER_PROBE = {**BASE_PEER, "robot_usb": "/dev/ttyUSB0", "mux": True}
GATE_OK = {"balance_state": 0, "motor_test_active": 0, "rc_valid": 0, "ch5_position": 0}
PASSING_XML = (
    '<testsuites><testsuite>'
    '<testcase classname="tests.hil.test_tier_a" name="test_reboot"/>'
    '</testsuite></testsuites>'
)
FAILING_XML = (
    '<testsuites><testsuite>'
    '<testcase classname="tests.hil.test_tier_a" name="test_reboot"><failure message="link did not return"/></testcase>'
    '<testcase classname="tests.hil.test_tier_a" name="test_health_within_limits"/>'
    '</testsuite></testsuites>'
)


def git(cwd, *args):
    return subprocess.run(
        ["git", "-c", "user.name=t", "-c", "user.email=t@example.com", *args],
        cwd=cwd, check=True, capture_output=True, text=True,
    ).stdout.strip()


def local(**info):
    return deploy.Host(None, {**BASE_LOCAL, **info})


def peer(**info):
    return deploy.Host("cm5", {**BASE_PEER, **info})


class Fleet:
    """Two fake hosts. A command gets the first scripted reply whose text it contains; every command is recorded."""

    def __init__(self, sha: str):
        self.sha = sha
        self.probes = {None: dict(LOCAL_PROBE), "cm5": dict(PEER_PROBE)}
        self.calls: list[tuple[str | None, str]] = []
        self.shipped: list[tuple[str | None, str]] = []
        self.copies: list[tuple] = []
        sha8 = sha[:8]
        self.replies = [
            ("-m pytest", (0, "", "")),
            (f"cat /shipped/cm5/{sha8}/hil.xml", (0, PASSING_XML, "")),
            ("navlink.py get balance_state", (0, json.dumps(GATE_OK), "")),
            ("navlink.py info", (0, json.dumps({"build": f"{sha8} 2026-11-02 10:00"}), "")),
            ("navlink.py get reset_reason", (0, json.dumps({"reset_reason": 1}), "")),
            ("port-holder", (0, "", "")),
            ("raw-serial", (0, "boot log\n", "")),
            ("upload_firmware.py", (0, "", "")),
            ("build_firmware.py", (0, "", "")),
            ("firmware-build-environment.json", (0, "{}\n", "")),
        ]

    def reply(self, needle: str, code: int = 0, out: str = "", err: str = "") -> None:
        self.replies.insert(0, (needle, (code, out, err)))

    def probe(self, host):
        if host not in self.probes:
            raise RuntimeError(f"{host} does not answer")
        return self.probes[host]

    def ship(self, host, sha):
        self.shipped.append((host, sha))
        return f"/shipped/{host or 'local'}/{sha[:8]}"

    def run(self, host, argv, *, cwd=None, timeout=600, interactive=False, input=None):
        command = " ".join(argv)
        self.calls.append((host, command))
        for needle, (code, out, err) in self.replies:
            if needle in command:
                return subprocess.CompletedProcess(argv, code, out, err)
        return subprocess.CompletedProcess(argv, 0, "", "")

    def copy(self, src_host, src, dst_host, dst):
        self.copies.append((src_host, src, dst_host, dst))


def commands(fleet: Fleet, needle: str) -> list[str]:
    return [command for _, command in fleet.calls if needle in command]


@pytest.fixture
def repo(tmp_path):
    path = tmp_path / "repo"
    path.mkdir()
    git(path, "init", "-q", "-b", "main")
    (path / "README.md").write_text("docs\n")
    (path / "firmware-build-environment.json").write_text("{}\n")
    git(path, "add", "-A")
    git(path, "commit", "-q", "-m", "tree")
    return path


@pytest.fixture
def fleet(monkeypatch, repo):
    fake = Fleet(git(repo, "rev-parse", "HEAD"))
    monkeypatch.setattr(devtools, "run", fake.run)
    monkeypatch.setattr(devtools, "probe", fake.probe)
    monkeypatch.setattr(devtools, "peer", lambda: "cm5")
    monkeypatch.setattr(devtools, "ship_tree", fake.ship)
    monkeypatch.setattr(deploy, "copy_output", fake.copy)
    return fake


def run_main(capsys, repo, *args):
    code = deploy.main(["--json", *args], root=repo)
    return code, json.loads(capsys.readouterr().out)


@pytest.mark.parametrize(
    "hosts, expected",
    [
        ([local(toolchain="arduino-cli"), peer(robot_usb="/dev/ttyUSB0", mux=True)], ("niko-desktop", "cm5", "cm5")),
        ([local(robot_usb="/dev/ttyUSB1"), peer(toolchain="arduino-cli")], ("cm5", "niko-desktop", "niko-desktop")),
        (
            [local(toolchain="arduino-cli", robot_usb="/dev/ttyUSB1"), peer(toolchain="arduino-cli", mux=True)],
            ("niko-desktop", "niko-desktop", "cm5"),
        ),
        ([local(toolchain="arduino-cli", mux=True), peer(robot_usb="/dev/ttyUSB0")], ("niko-desktop", "cm5", "niko-desktop")),
    ],
)
def test_selection_takes_the_first_host_with_each_part(hosts, expected):
    build, usb, link = deploy.select_hosts(hosts, build_only=False)
    assert (build.name, usb.name, link.name) == expected


def test_no_toolchain_is_blocked_at_the_build_stage():
    with pytest.raises(deploy.Stop) as error:
        deploy.select_hosts([local(), peer(robot_usb="/dev/ttyUSB0")], build_only=False)
    assert (error.value.code, error.value.stage, error.value.detail) == (3, "firmware_build", "no toolchain on either host")


def test_no_robot_usb_is_blocked_at_the_deploy_stage():
    with pytest.raises(deploy.Stop) as error:
        deploy.select_hosts([local(toolchain="a"), peer()], build_only=False)
    assert (error.value.code, error.value.stage, error.value.detail) == (3, "deploy", "robot not connected")


def test_robot_usb_on_both_hosts_is_an_error():
    hosts = [local(toolchain="a", robot_usb="/dev/ttyUSB1"), peer(robot_usb="/dev/ttyUSB0")]
    with pytest.raises(deploy.Stop) as error:
        deploy.select_hosts(hosts, build_only=False)
    assert (error.value.code, error.value.stage, error.value.detail) == (1, "deploy", "robot USB on both hosts")


def test_build_only_needs_no_robot_host():
    build, usb, link = deploy.select_hosts([local(toolchain="a"), peer()], build_only=True)
    assert (build.name, usb, link) == ("niko-desktop", None, None)


def test_uncommitted_change_is_refused_before_any_host_is_touched(repo, fleet, capsys):
    (repo / "README.md").write_text("changed\n")
    code, result = run_main(capsys, repo)
    assert code == 2
    assert result["sha"] is None
    assert fleet.calls == [] and fleet.shipped == []


def test_full_run_builds_gates_flashes_and_passes_tier_a(repo, fleet, capsys):
    code, result = run_main(capsys, repo)
    sha8 = fleet.sha[:8]
    assert code == 0
    assert (result["build_host"], result["usb_host"], result["link_host"]) == ("niko-desktop", "cm5", "cm5")
    assert {name: data["status"] for name, data in result["stages"].items()} == {
        "firmware_build": "pass", "deploy": "pass", "tier_a": "pass",
    }
    assert result["stages"]["tier_a"]["tests"] == {"tests/hil/test_tier_a.py::test_reboot": "passed"}
    assert commands(fleet, "build_firmware.py") == [
        f"python3 scripts/build_firmware.py --revision {fleet.sha} --output-dir /shipped/local/{sha8}/build/flash"
    ]
    assert fleet.copies == [(None, f"/shipped/local/{sha8}/build/flash", "cm5", f"/shipped/cm5/{sha8}/build/flash")]
    assert commands(fleet, "port-holder") == ["python3 scripts/devtools.py port-holder /dev/ttyUSB0"]
    assert commands(fleet, "navlink.py get balance_state") == [
        "python3 scripts/navlink.py get balance_state motor_test_active rc_valid ch5_position --json --port tcp:127.0.0.1:7650"
    ]
    assert commands(fleet, "upload_firmware.py") == [
        f"python3 scripts/upload_firmware.py /shipped/cm5/{sha8}/build/flash --port /dev/ttyUSB0"
    ]
    assert commands(fleet, "-m pytest") == [
        f"python3 -m pytest tests/hil -m not guided --robot tcp:127.0.0.1:7650 -q --junitxml /shipped/cm5/{sha8}/hil.xml"
    ]
    order = [command for _, command in fleet.calls]
    assert order.index(commands(fleet, "navlink.py get balance_state")[0]) < order.index(commands(fleet, "upload_firmware.py")[0])


@pytest.mark.parametrize(
    "values",
    [
        {**GATE_OK, "balance_state": 1},
        {**GATE_OK, "motor_test_active": 1},
        {**GATE_OK, "rc_valid": 1, "ch5_position": 1},
        {key: value for key, value in GATE_OK.items() if key != "ch5_position"},
    ],
)
def test_gate_refusal_flashes_nothing(repo, fleet, capsys, values):
    fleet.reply("navlink.py get balance_state", out=json.dumps(values))
    code, result = run_main(capsys, repo)
    assert code == 4
    assert result["stages"]["deploy"]["status"] == "fail"
    assert commands(fleet, "upload_firmware.py") == []
    assert "tier_a" not in result["stages"]


def test_gate_with_no_answer_from_the_robot_flashes_nothing(repo, fleet, capsys):
    fleet.reply("navlink.py get balance_state", code=1, err="no answer from the robot\n")
    code, result = run_main(capsys, repo)
    assert code == 4
    assert result["stages"]["deploy"]["detail"] == "navlink could not read the gate signals: no answer from the robot"
    assert commands(fleet, "upload_firmware.py") == []


def test_transmitter_alone_does_not_refuse(repo, fleet, capsys):
    fleet.reply("navlink.py get balance_state", out=json.dumps({**GATE_OK, "rc_valid": 1}))
    code, _ = run_main(capsys, repo)
    assert code == 0


def test_held_port_stops_before_the_gate(repo, fleet, capsys):
    fleet.reply("port-holder", out="4321 python3\n")
    code, result = run_main(capsys, repo)
    assert code == 9
    assert result["stages"]["deploy"]["detail"] == "/dev/ttyUSB0 is held by 4321 python3"
    assert commands(fleet, "navlink.py get") == []
    assert commands(fleet, "upload_firmware.py") == []


def test_build_failure_stops_before_the_robot(repo, fleet, capsys):
    fleet.reply("build_firmware.py", code=1, err="error: compile failed\n")
    code, result = run_main(capsys, repo)
    assert code == 5
    assert result["stages"]["firmware_build"]["status"] == "fail"
    assert result["stages"]["firmware_build"]["detail"] == "control build failed: error: compile failed"
    assert "deploy" not in result["stages"]
    assert fleet.copies == []


def test_upload_failure_is_exit_6(repo, fleet, capsys):
    fleet.reply("upload_firmware.py", code=1, err="A fatal error occurred\n")
    code, result = run_main(capsys, repo)
    assert code == 6
    assert result["stages"]["deploy"]["detail"] == "upload failed: A fatal error occurred"
    assert "tier_a" not in result["stages"]


def test_build_id_mismatch_after_flash_is_exit_7_with_the_usb_log(repo, fleet, capsys):
    fleet.reply("navlink.py info", out=json.dumps({"build": "deadbeef 2026-11-02 10:00"}))
    code, result = run_main(capsys, repo)
    assert code == 7
    assert result["stages"]["deploy"]["detail"] == f"build ID 'deadbeef 2026-11-02 10:00' does not start with {fleet.sha[:8]}"
    assert result["raw_usb"] == "boot log\n"
    assert commands(fleet, "raw-serial") == ["python3 scripts/devtools.py raw-serial /dev/ttyUSB0 10"]
    assert len(commands(fleet, "upload_firmware.py")) == 1
    assert "tier_a" not in result["stages"]


def test_reset_reason_other_than_power_on_is_exit_7(repo, fleet, capsys):
    fleet.reply("navlink.py get reset_reason", out=json.dumps({"reset_reason": 3}))
    code, result = run_main(capsys, repo)
    assert code == 7
    assert result["stages"]["deploy"]["detail"] == "reset_reason 3 is not 1 (power-on)"


def test_tier_a_failure_is_exit_8_with_the_junit_outcomes(repo, fleet, capsys):
    sha8 = fleet.sha[:8]
    fleet.reply(f"cat /shipped/cm5/{sha8}/hil.xml", out=FAILING_XML)
    fleet.reply("-m pytest", code=1, err="1 failed\n")
    code, result = run_main(capsys, repo)
    assert code == 8
    assert result["stages"]["tier_a"]["status"] == "fail"
    assert result["stages"]["tier_a"]["detail"] == "failed: tests/hil/test_tier_a.py::test_reboot"
    assert result["stages"]["tier_a"]["tests"] == {
        "tests/hil/test_tier_a.py::test_reboot": "failed",
        "tests/hil/test_tier_a.py::test_health_within_limits": "passed",
    }


def test_diagnostic_image_builds_both_and_flashes_its_own_output(repo, fleet, capsys):
    code, result = run_main(capsys, repo, "--image", "diagnostic")
    sha8 = fleet.sha[:8]
    assert code == 0
    assert set(result["stages"]) == {"firmware_build", "deploy"}
    assert commands(fleet, "build_firmware.py") == [
        f"python3 scripts/build_firmware.py --revision {fleet.sha} --output-dir /shipped/local/{sha8}/build/flash",
        f"python3 scripts/build_firmware.py --revision {fleet.sha} --output-dir /shipped/local/{sha8}/build/flash-diagnostic"
        " --build-property compiler.cpp.extra_flags=-DSENSOR_DIAGNOSTIC_MODE=1",
    ]
    assert commands(fleet, "upload_firmware.py") == [
        f"python3 scripts/upload_firmware.py /shipped/cm5/{sha8}/build/flash-diagnostic --port /dev/ttyUSB0"
    ]


def test_ship_failure_is_recorded_as_a_failed_stage_not_a_traceback(repo, fleet, capsys, monkeypatch):
    def ship(host, sha):
        if host == "cm5":
            raise RuntimeError("shipping abc to cm5 failed: no space left on device")
        return fleet.ship(host, sha)

    monkeypatch.setattr(devtools, "ship_tree", ship)
    code, result = run_main(capsys, repo)
    assert code == 1
    assert result["stages"]["firmware_build"]["status"] == "pass"
    assert result["stages"]["deploy"]["status"] == "fail"
    assert result["stages"]["deploy"]["detail"] == "shipping abc to cm5 failed: no space left on device"
    assert commands(fleet, "upload_firmware.py") == []


def test_timeout_is_recorded_as_a_failed_stage_not_a_traceback(repo, fleet, capsys, monkeypatch):
    run_fake = fleet.run

    def run(host, argv, **kwargs):
        if "build_firmware.py" in " ".join(argv):
            raise subprocess.TimeoutExpired(argv, 1800)
        return run_fake(host, argv, **kwargs)

    monkeypatch.setattr(devtools, "run", run)
    code, result = run_main(capsys, repo)
    assert code == 1
    assert result["stages"]["firmware_build"]["status"] == "fail"
    assert "timed out after 1800 seconds" in result["stages"]["firmware_build"]["detail"]
    assert "deploy" not in result["stages"]


def test_build_only_builds_and_stops(repo, fleet, capsys):
    code, result = run_main(capsys, repo, "--build-only")
    sha8 = fleet.sha[:8]
    assert code == 0
    assert set(result["stages"]) == {"firmware_build"}
    assert commands(fleet, "build_firmware.py") == [
        f"python3 scripts/build_firmware.py --revision {fleet.sha} --output-dir /shipped/local/{sha8}/build/flash"
    ]
    assert commands(fleet, "upload_firmware.py") == []
    assert commands(fleet, "navlink.py") == []


def test_changed_build_environment_is_reported_without_failing(repo, fleet, capsys):
    fleet.reply("firmware-build-environment.json", out='{"fqbn": "other"}\n')
    code, result = run_main(capsys, repo, "--build-only")
    assert code == 0
    assert result["stages"]["firmware_build"]["detail"] == "firmware-build-environment.json differs from the committed file"


def test_no_toolchain_is_exit_3_at_the_build_stage(repo, fleet, capsys):
    fleet.probes[None]["toolchain"] = None
    code, result = run_main(capsys, repo)
    assert code == 3
    assert set(result["stages"]) == {"firmware_build"}
    assert result["stages"]["firmware_build"]["status"] == "blocked"
    assert result["stages"]["firmware_build"]["detail"] == "no toolchain on either host"
    assert commands(fleet, "build_firmware.py") == []


def test_robot_not_connected_is_exit_3_at_the_deploy_stage(repo, fleet, capsys):
    fleet.probes["cm5"]["robot_usb"] = None
    code, result = run_main(capsys, repo)
    assert code == 3
    assert result["stages"]["deploy"]["status"] == "blocked"
    assert result["stages"]["deploy"]["detail"] == "robot not connected"
    assert "firmware_build" not in result["stages"]
    assert commands(fleet, "build_firmware.py") == []


def test_evidence_records_the_stages_for_the_branch(repo, fleet, capsys):
    (repo / "work" / "main").mkdir(parents=True)
    code, _ = run_main(capsys, repo, "--build-only")
    evidence = devtools.read_evidence(repo / "work" / "main", fleet.sha)
    assert code == 0
    assert evidence["branch"] == "main"
    assert set(evidence["stages"]) == {"firmware_build"}
    assert evidence["stages"]["firmware_build"]["status"] == "pass"


def fake_rsync(calls):
    def rsync(argv, **kwargs):
        calls.append(argv)
        return subprocess.CompletedProcess(argv, 0, "", "")

    return rsync


def test_copy_pushes_from_this_host_and_pulls_to_it(monkeypatch):
    calls = []
    monkeypatch.setattr(devtools, "run", lambda host, argv, **kwargs: subprocess.CompletedProcess(argv, 0, "", ""))
    monkeypatch.setattr(deploy.subprocess, "run", fake_rsync(calls))
    deploy.copy_output(None, "/a/build/flash", "cm5", "/b/build/flash")
    deploy.copy_output("cm5", "/b/build/flash", None, "/a/build/flash")
    assert calls == [
        ["rsync", "-a", "/a/build/flash/", "cm5:/b/build/flash/"],
        ["rsync", "-a", "cm5:/b/build/flash/", "/a/build/flash/"],
    ]


def test_junit_outcomes_map_to_node_ids():
    xml = (
        '<testsuites><testsuite>'
        '<testcase classname="tests.hil.test_tier_a" name="test_ok"/>'
        '<testcase classname="tests.hil.test_tier_a" name="test_bad"><failure/></testcase>'
        '<testcase classname="tests.hil.test_tier_a" name="test_later"><skipped/></testcase>'
        '</testsuite></testsuites>'
    )
    assert deploy.read_junit(xml) == {
        "tests/hil/test_tier_a.py::test_ok": "passed",
        "tests/hil/test_tier_a.py::test_bad": "failed",
        "tests/hil/test_tier_a.py::test_later": "skipped",
    }
    assert deploy.read_junit("") == {}
