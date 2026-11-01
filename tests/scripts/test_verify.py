"""verify: stage selection, skips, tier C, result rules, the summary, and whole runs in a temporary repository.

The robot, ssh and deploy.py never run: devtools.run, devtools.probe, devtools.peer and devtools.ship_tree are replaced.
"""

import argparse
import contextlib
import io
import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import deploy  # noqa: E402
import devtools  # noqa: E402
import verify  # noqa: E402
from devtools import Acceptance, State  # noqa: E402

JUNIT_TIER_A = """<testsuite>
  <testcase classname="tests.hil.test_tier_a" name="test_reboot"/>
  <testcase classname="tests.hil.test_tier_a" name="test_param_set_restore"><failure message="x"/></testcase>
  <testcase classname="tests.hil.test_guided.TestWheels" name="test_spin"><skipped/></testcase>
</testsuite>"""

JUNIT_GUIDED = """<testsuite>
  <testcase classname="tests.hil.test_guided" name="test_spin"/>
</testsuite>"""

ACCEPTANCE_TIER_A = [Acceptance("tests/hil/test_tier_a.py::test_reboot", "unattended", "pending")]
ACCEPTANCE_BOTH = [
    Acceptance("tests/hil/test_tier_a.py::test_reboot", "unattended", "pending"),
    Acceptance("tests/hil/test_guided.py::test_spin", "guided", "pending"),
]
DEPLOY_STAGES = {
    "firmware_build": {"status": "pass", "at": "t", "detail": "built"},
    "deploy": {"status": "pass", "at": "t", "detail": "flashed"},
    "tier_a": {"status": "pass", "at": "t", "detail": "2 passed", "tests": {"tests/hil/test_tier_a.py::test_reboot": "passed"}},
}


@pytest.fixture(autouse=True)
def sim_check_passes(monkeypatch):
    """The real devtools.sim_check would run tests/simulation; every test here gets a passing stand-in unless it sets its own."""
    monkeypatch.setattr(devtools, "sim_check", lambda root, sha: (True, "3 passed"))


def git(repo: Path, *args: str) -> str:
    return subprocess.run(
        ["git", "-c", "user.name=test", "-c", "user.email=test@example.com", *args],
        cwd=repo, check=True, capture_output=True, text=True,
    ).stdout.strip()


def make_repo(tmp_path: Path, monkeypatch, changed: dict[str, str], acceptance: list[Acceptance]) -> tuple[Path, str]:
    """A repository on branch feat/x with one commit on top of main's base commit and a task folder for it."""
    repo = tmp_path / "repo"
    repo.mkdir()
    git(repo, "init", "-q", "-b", "main")
    (repo / "README.md").write_text("base\n", encoding="utf-8")
    git(repo, "add", "-A")
    git(repo, "commit", "-q", "-m", "base")
    baseline = git(repo, "rev-parse", "HEAD")
    git(repo, "checkout", "-q", "-b", "feat/x")
    for relative, text in changed.items():
        (repo / relative).parent.mkdir(parents=True, exist_ok=True)
        (repo / relative).write_text(text, encoding="utf-8")
    git(repo, "add", "-A")
    git(repo, "commit", "-q", "-m", "change")
    task = repo / "work" / "feat-x"
    task.mkdir(parents=True)
    devtools.write_state(
        task / "state.md",
        State(branch="feat/x", worktree=str(repo), baseline=baseline, status="implementing", acceptance=acceptance, body="## Contract\n"),
    )
    monkeypatch.chdir(repo)
    return repo, baseline


def evidence_of(repo: Path) -> dict:
    sha = git(repo, "rev-parse", "HEAD")
    return json.loads((repo / "work" / "feat-x" / "evidence" / f"{sha}.json").read_text(encoding="utf-8"))


def deploy_json(stages: dict, code: int) -> str:
    """deploy.py's own --json report for these stages, so a renamed key in deploy.py fails these tests."""
    run = deploy.Run("control")
    run.stages = dict(stages)
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        deploy.report(argparse.Namespace(json=True), run, code)
    return out.getvalue()


class FakeRobotRun:
    """Stands in for devtools.run: pytest passes, deploy.py answers with the given stages, navlink reports build, guided tests read the junit text."""

    def __init__(self, deploy_code: int = 0, stages: dict | None = None, junit: str = JUNIT_GUIDED, build: str = ""):
        self.deploy_code = deploy_code
        self.stages = DEPLOY_STAGES if stages is None else stages
        self.junit = junit
        self.build = build
        self.calls: list[list[str]] = []

    def __call__(self, host, argv, *, cwd=None, timeout=600, interactive=False, input=None):
        argv = list(argv)
        self.calls.append(argv)
        if argv[:3] == ["python3", "-m", "pytest"] and "--guided" in argv:
            return subprocess.CompletedProcess(argv, 0, "", "")
        if argv[:3] == ["python3", "-m", "pytest"]:
            return subprocess.CompletedProcess(argv, 0, "1 passed in 0.1s\n", "")
        if argv[:2] == ["python3", "scripts/deploy.py"]:
            return subprocess.CompletedProcess(argv, self.deploy_code, deploy_json(self.stages, self.deploy_code), "")
        if argv[:3] == ["python3", "scripts/navlink.py", "info"]:
            return subprocess.CompletedProcess(argv, 0, json.dumps({"build": self.build}), "")
        if argv[:1] == ["cat"]:
            return subprocess.CompletedProcess(argv, 0, self.junit, "")
        raise AssertionError(f"unexpected command: {argv}")


def test_required_stages_follow_the_changed_paths():
    assert verify.required_stages(["README.md"]) == ["host"]
    assert verify.required_stages(["src/ES-02/OllieFOCdrive/x.cpp"]) == ["host", "firmware_build", "deploy", "tier_a", "tier_c"]
    assert verify.required_stages(["src/navbot_link/link.py"]) == ["host", "sim", "deploy", "tier_a"]
    assert verify.required_stages(["src/ros2_pc/ros2/src/node.py"]) == ["host", "ros2_pc"]


def test_sim_is_required_when_a_changed_path_is_a_simulator_input():
    assert verify.required_stages(["src/ES-02/OllieFOCdrive/ControlTick.cpp"]) == ["host", "sim", "firmware_build", "deploy", "tier_a", "tier_c"]
    assert verify.required_stages(["src/sim/plant.cpp"]) == ["host", "sim"]
    assert verify.required_stages(["docs/x.md"]) == ["host"]


@pytest.mark.parametrize("path", ["src/ES-02/BalanceController.cpp", "src/ES-02/lib/rc_mode.h", "src/ES-02/OllieFOCdrive/OllieFOCdrive.ino"])
def test_tier_b_is_required_for_the_guided_trigger_files(path):
    assert verify.required_stages([path]) == ["host", "firmware_build", "deploy", "tier_a", "tier_b", "tier_c"]


def test_junit_report_maps_each_outcome_to_its_node_id():
    assert verify.junit_outcomes(JUNIT_TIER_A) == {
        "tests/hil/test_tier_a.py::test_reboot": "passed",
        "tests/hil/test_tier_a.py::test_param_set_restore": "failed",
        "tests/hil/test_guided.py::TestWheels::test_spin": "skipped",
    }


def test_tier_c_without_acceptance_tests_fails_with_the_listed_message():
    stage = verify._tier_c([Acceptance("tier_b", "guided", "skipped", "no robot (2026-10-31)")], {})
    assert stage["status"] == "fail"
    assert stage["detail"] == "no acceptance tests listed in state.md"


def test_tier_c_sets_each_item_status_and_leaves_skips_alone(monkeypatch):
    acceptance = [
        Acceptance("a::t1", "unattended", "pending"),
        Acceptance("a::t2", "guided", "pending"),
        Acceptance("a::t3", "unattended", "skipped", "no cable (2026-10-31)"),
    ]
    results = {
        "deploy": {"status": "pass"},
        "tier_a": {"status": "pass", "tests": {"a::t1": "passed"}},
        "tier_b": {"status": "fail", "tests": {"a::t2": "failed"}},
    }
    stage = verify._tier_c(acceptance, results)
    assert (stage["status"], stage["detail"]) == ("fail", "failed: a::t2")
    assert [item.status for item in acceptance] == ["pass", "fail", "skipped"]


def test_tier_c_is_blocked_while_a_guided_result_is_missing_and_failed_when_deploy_failed():
    acceptance = [Acceptance("a::t2", "guided", "pending")]
    pending = verify._tier_c(acceptance, {"deploy": {"status": "pass"}, "tier_a": {"status": "pass", "tests": {}}})
    assert (pending["status"], pending["detail"]) == ("blocked", "no result for a::t2")
    failed = verify._tier_c(acceptance, {"deploy": {"status": "fail"}, "tier_a": {"status": "fail"}})
    assert (failed["status"], failed["detail"]) == ("fail", "deploy failed")
    blocked = verify._tier_c(acceptance, {"deploy": {"status": "blocked"}})
    assert (blocked["status"], blocked["detail"]) == ("blocked", "deploy blocked")


def test_a_skip_records_the_users_words_with_the_date(monkeypatch):
    monkeypatch.setattr(verify, "_today", lambda: "2026-10-31")
    acceptance: list[Acceptance] = []
    verify._skip(acceptance, "tests/hil/test_tier_a.py::test_reboot", "the transmitter is off")
    assert acceptance == [
        Acceptance("tests/hil/test_tier_a.py::test_reboot", "unattended", "skipped", "the transmitter is off (2026-10-31)")
    ]


def test_host_cannot_be_skipped_and_unknown_ids_are_refused():
    with pytest.raises(verify.Refused, match="host cannot be skipped"):
        verify._skip([], "host", "not now")
    with pytest.raises(verify.Refused, match="neither a stage name nor a pytest node id"):
        verify._skip([], "tier_z", "not now")


def test_skip_needs_a_reason(capsys):
    with pytest.raises(SystemExit) as exit_info:
        verify.main(["--skip", "tier_b"])
    assert exit_info.value.code == 2


@pytest.mark.parametrize(
    ("stages", "dirty", "result"),
    [
        ({"host": {"status": "pass"}, "tier_a": {"status": "pass"}}, False, "done"),
        ({"host": {"status": "pass"}, "tier_a": {"status": "skipped"}}, False, "done"),
        ({"host": {"status": "pass"}, "tier_a": {"status": "pass"}}, True, "failed"),
        ({"host": {"status": "pass"}, "deploy": {"status": "blocked"}}, False, "blocked"),
        ({"deploy": {"status": "blocked"}, "tier_a": {"status": "fail"}}, False, "failed"),
    ],
)
def test_result_is_done_blocked_or_failed(stages, dirty, result):
    assert verify._result(stages, dirty) == result


def test_summary_text_is_literal():
    stages = {
        "host": {"status": "pass", "detail": "1 passed"},
        "tier_b": {"status": "blocked", "detail": verify.GUIDED_PENDING},
    }
    text = verify._summary(
        "feat/x", "0123456789abcdef", "blocked", ["host", "tier_b"], stages,
        [Acceptance("tier_b", "guided", "skipped", "no robot (2026-10-31)")], dirty=False,
    )
    assert text == (
        "# verify feat/x at 01234567: blocked\n"
        "\n"
        "| Stage | Status | Detail |\n"
        "| --- | --- | --- |\n"
        "| host | pass | 1 passed |\n"
        "| tier_b | blocked | guided tests pending: run python3 scripts/verify.py --guided |\n"
        "\n"
        "Skipped:\n"
        "- tier_b: no robot (2026-10-31)"
    )


def test_host_stage_leaves_the_simulation_to_the_sim_stage(tmp_path, monkeypatch):
    make_repo(tmp_path, monkeypatch, {"src/navbot_link/link.py": "x = 2\n"}, [])
    fake = FakeRobotRun()
    monkeypatch.setattr(devtools, "run", fake)
    assert verify.main([]) == 0
    assert ["python3", "-m", "pytest", "tests", "-q", "--ignore", "tests/simulation"] in fake.calls


def test_sim_without_a_compiler_is_blocked(tmp_path, monkeypatch, capsys):
    make_repo(tmp_path, monkeypatch, {"src/sim/plant.cpp": "x\n"}, [])
    monkeypatch.setattr(devtools, "run", FakeRobotRun())
    monkeypatch.setattr(verify.shutil, "which", lambda name: None)
    assert verify.main([]) == 3
    assert "| sim | blocked | no C++ compiler for the simulation on this host |" in capsys.readouterr().out


@pytest.mark.parametrize(
    ("passed", "detail", "status", "code", "result"),
    [(False, "1 failed", "fail", 1, "failed"), (True, "3 passed", "pass", 0, "done")],
)
def test_sim_input_change_runs_the_sim_check_for_head(tmp_path, monkeypatch, capsys, passed, detail, status, code, result):
    repo, _ = make_repo(tmp_path, monkeypatch, {"src/sim/plant.cpp": "x\n"}, [])
    monkeypatch.setattr(devtools, "run", FakeRobotRun())
    seen = []

    def sim_check(root, sha):
        seen.append(sha)
        return passed, detail

    monkeypatch.setattr(devtools, "sim_check", sim_check)
    assert verify.main([]) == code
    sha = git(repo, "rev-parse", "HEAD")
    assert seen == [sha]
    assert f"| sim | {status} | {detail} |" in capsys.readouterr().out
    assert evidence_of(repo)["stages"]["sim"]["status"] == status
    assert evidence_of(repo)["verify"]["result"] == result


def test_navlink_change_is_done_and_recorded_for_head(tmp_path, monkeypatch, capsys):
    repo, _ = make_repo(tmp_path, monkeypatch, {"src/navbot_link/link.py": "x = 2\n"}, [])
    fake = FakeRobotRun()
    monkeypatch.setattr(devtools, "run", fake)
    assert verify.main([]) == 0
    evidence = evidence_of(repo)
    assert evidence["verify"]["result"] == "done"
    assert evidence["stages"]["tier_a"]["tests"] == {"tests/hil/test_tier_a.py::test_reboot": "passed"}
    assert ["python3", "scripts/deploy.py", "--json"] in fake.calls
    out = capsys.readouterr().out
    assert "| tier_a | pass | 2 passed |" in out
    assert out.endswith("Skipped:\n- none\n")


def test_es02_change_is_blocked_until_guided_tests_run(tmp_path, monkeypatch, capsys):
    repo, _ = make_repo(tmp_path, monkeypatch, {"src/ES-02/BalanceController.cpp": "x\n"}, ACCEPTANCE_BOTH)
    monkeypatch.setattr(devtools, "run", FakeRobotRun())
    assert verify.main([]) == 3
    assert evidence_of(repo)["verify"]["result"] == "blocked"
    assert f"| tier_b | blocked | {verify.GUIDED_PENDING} |" in capsys.readouterr().out


def test_guided_pass_recorded_for_head_lets_the_change_be_done(tmp_path, monkeypatch):
    repo, _ = make_repo(tmp_path, monkeypatch, {"src/ES-02/BalanceController.cpp": "x\n"}, ACCEPTANCE_BOTH)
    monkeypatch.setattr(devtools, "run", FakeRobotRun())
    sha = git(repo, "rev-parse", "HEAD")
    devtools.update_evidence(
        repo / "work" / "feat-x", sha, "feat/x",
        stages={"tier_b": {"status": "pass", "at": "t", "detail": "1 tests, 0 failed", "tests": {"tests/hil/test_guided.py::test_spin": "passed"}}},
    )
    assert verify.main([]) == 0
    assert evidence_of(repo)["verify"]["result"] == "done"


def use_robot(monkeypatch, fake: FakeRobotRun) -> None:
    monkeypatch.setattr(devtools, "run", fake)
    monkeypatch.setattr(devtools, "peer", lambda: "cm5")
    monkeypatch.setattr(devtools, "probe", lambda host: {"mux": False, "robot_usb": "/dev/ttyUSB0", "ros": False})
    monkeypatch.setattr(devtools, "ship_tree", lambda host, sha: "/tree")


def test_guided_run_reads_its_junit_report(tmp_path, monkeypatch):
    repo, _ = make_repo(tmp_path, monkeypatch, {"src/ES-02/BalanceController.cpp": "x\n"}, ACCEPTANCE_BOTH)
    sha = git(repo, "rev-parse", "HEAD")
    fake = FakeRobotRun(build=sha[:8])
    use_robot(monkeypatch, fake)
    assert verify.main(["--guided"]) == 0
    assert evidence_of(repo)["stages"]["tier_b"]["tests"] == {"tests/hil/test_guided.py::test_spin": "passed"}


def test_guided_run_is_blocked_when_the_robot_runs_another_build(tmp_path, monkeypatch, capsys):
    repo, _ = make_repo(tmp_path, monkeypatch, {"src/ES-02/BalanceController.cpp": "x\n"}, ACCEPTANCE_BOTH)
    sha = git(repo, "rev-parse", "HEAD")
    fake = FakeRobotRun(build="deadbeef")
    use_robot(monkeypatch, fake)
    assert verify.main(["--guided"]) == 3
    out = capsys.readouterr().out
    assert f"| tier_b | blocked | robot runs build deadbeef, HEAD is {sha[:8]}: deploy HEAD before the guided tests |" in out
    assert not any("--guided" in call for call in fake.calls)


def test_dirty_tracked_file_fails_the_result(tmp_path, monkeypatch, capsys):
    repo, _ = make_repo(tmp_path, monkeypatch, {"src/navbot_link/link.py": "x = 2\n"}, ACCEPTANCE_TIER_A)
    monkeypatch.setattr(devtools, "run", FakeRobotRun())
    (repo / "README.md").write_text("changed\n", encoding="utf-8")
    assert verify.main([]) == 1
    assert evidence_of(repo)["verify"]["result"] == "failed"
    assert "Uncommitted changes to tracked files: the result cannot be done." in capsys.readouterr().out


def test_uncommitted_firmware_build_environment_json_is_not_dirty(tmp_path, monkeypatch):
    changed = {"src/navbot_link/link.py": "x = 2\n", "firmware-build-environment.json": "{}\n"}
    repo, _ = make_repo(tmp_path, monkeypatch, changed, ACCEPTANCE_TIER_A)
    monkeypatch.setattr(devtools, "run", FakeRobotRun())
    (repo / "firmware-build-environment.json").write_text('{"rewritten": true}\n', encoding="utf-8")
    assert verify.main([]) == 0
    assert evidence_of(repo)["verify"]["result"] == "done"


def test_tier_c_is_not_computed_when_it_is_not_required(tmp_path, monkeypatch):
    repo, _ = make_repo(tmp_path, monkeypatch, {"src/navbot_link/link.py": "x = 2\n"}, ACCEPTANCE_TIER_A)
    monkeypatch.setattr(devtools, "run", FakeRobotRun())
    assert verify.main([]) == 0
    state = devtools.read_state(repo / "work" / "feat-x" / "state.md")
    assert [item.status for item in state.acceptance] == ["pending"]
    assert "tier_c" not in evidence_of(repo)["stages"]


def test_skipped_tier_b_ends_done_with_its_reason(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(verify, "_today", lambda: "2026-10-31")
    repo, _ = make_repo(tmp_path, monkeypatch, {"src/ES-02/BalanceController.cpp": "x\n"}, ACCEPTANCE_BOTH)
    monkeypatch.setattr(devtools, "run", FakeRobotRun())
    assert verify.main(["--skip", "tier_b", "--reason", "no robot"]) == 0
    sha = git(repo, "rev-parse", "HEAD")
    assert capsys.readouterr().out == (
        f"# verify feat/x at {sha[:8]}: done\n"
        "\n"
        "| Stage | Status | Detail |\n"
        "| --- | --- | --- |\n"
        "| host | pass | 1 passed in 0.1s |\n"
        "| firmware_build | pass | control built; diagnostic built |\n"
        "| deploy | pass | flashed |\n"
        "| tier_a | pass | 2 passed |\n"
        "| tier_b | skipped | no robot (2026-10-31) |\n"
        "| tier_c | pass | 1 acceptance tests passed, 1 skipped |\n"
        "\n"
        "Skipped:\n"
        "- tier_b: no robot (2026-10-31)\n"
    )
    assert evidence_of(repo)["verify"]["result"] == "done"


def test_skipped_deploy_ends_done_with_its_reason(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(verify, "_today", lambda: "2026-10-31")
    repo, _ = make_repo(tmp_path, monkeypatch, {"src/navbot_link/link.py": "x = 2\n"}, ACCEPTANCE_TIER_A)
    monkeypatch.setattr(devtools, "run", FakeRobotRun())
    assert verify.main(["--skip", "deploy", "--reason", "transmitter off"]) == 0
    sha = git(repo, "rev-parse", "HEAD")
    assert capsys.readouterr().out == (
        f"# verify feat/x at {sha[:8]}: done\n"
        "\n"
        "| Stage | Status | Detail |\n"
        "| --- | --- | --- |\n"
        "| host | pass | 1 passed in 0.1s |\n"
        "| sim | pass | 3 passed |\n"
        "| deploy | skipped | transmitter off (2026-10-31) |\n"
        "| tier_a | skipped | deploy skipped: transmitter off (2026-10-31) |\n"
        "\n"
        "Skipped:\n"
        "- deploy: transmitter off (2026-10-31)\n"
    )
    assert evidence_of(repo)["verify"]["result"] == "done"


def test_ship_failure_is_recorded_as_a_failed_stage(tmp_path, monkeypatch, capsys):
    repo, _ = make_repo(tmp_path, monkeypatch, {"src/ros2_pc/ros2/src/node.py": "x\n"}, [])
    monkeypatch.setattr(devtools, "run", FakeRobotRun())
    monkeypatch.setattr(devtools, "peer", lambda: "cm5")
    monkeypatch.setattr(devtools, "probe", lambda host: {"mux": False, "robot_usb": None, "ros": True})

    def ship_tree(host, sha):
        raise RuntimeError("shipping sha to cm5 failed: no space left")

    monkeypatch.setattr(devtools, "ship_tree", ship_tree)
    assert verify.main([]) == 1
    assert "| ros2_pc | fail | shipping sha to cm5 failed: no space left |" in capsys.readouterr().out
    assert evidence_of(repo)["stages"]["ros2_pc"]["status"] == "fail"
    assert evidence_of(repo)["verify"]["result"] == "failed"


def test_deploy_timeout_is_recorded_as_a_failed_deploy(tmp_path, monkeypatch, capsys):
    repo, _ = make_repo(tmp_path, monkeypatch, {"src/navbot_link/link.py": "x = 2\n"}, ACCEPTANCE_TIER_A)
    fake = FakeRobotRun()

    def run(host, argv, **kwargs):
        if argv[:2] == ["python3", "scripts/deploy.py"]:
            raise subprocess.TimeoutExpired(argv, verify.DEPLOY_TIMEOUT)
        return fake(host, argv, **kwargs)

    monkeypatch.setattr(devtools, "run", run)
    assert verify.main([]) == 1
    out = capsys.readouterr().out
    assert f"| deploy | fail | deploy.py did not finish within {verify.DEPLOY_TIMEOUT} s |" in out
    assert "| tier_a | fail | not reached |" in out
    assert evidence_of(repo)["verify"]["result"] == "failed"


def test_failed_deploy_fails_the_change_and_tier_c(tmp_path, monkeypatch, capsys):
    stages = {
        "firmware_build": {"status": "pass", "at": "t", "detail": "built"},
        "deploy": {"status": "fail", "at": "t", "detail": "gate refused"},
    }
    make_repo(tmp_path, monkeypatch, {"src/ES-02/OllieFOCdrive/x.cpp": "x\n"}, ACCEPTANCE_TIER_A)
    monkeypatch.setattr(devtools, "run", FakeRobotRun(deploy_code=4, stages=stages))
    assert verify.main([]) == 1
    out = capsys.readouterr().out
    assert "| deploy | fail | gate refused |" in out
    assert "| tier_a | fail | not reached |" in out
    assert "| tier_c | fail | deploy failed |" in out


def test_skipped_host_is_refused_before_anything_runs(tmp_path, monkeypatch, capsys):
    make_repo(tmp_path, monkeypatch, {"src/navbot_link/link.py": "x = 2\n"}, [])
    fake = FakeRobotRun()
    monkeypatch.setattr(devtools, "run", fake)
    assert verify.main(["--skip", "host", "--reason", "not now"]) == 2
    assert fake.calls == []
