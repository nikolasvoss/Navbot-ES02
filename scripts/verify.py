#!/usr/bin/env python3
"""verify: run the stages a task needs for HEAD, record the result, and print a summary.

The stage table, skips and result rules: docs/integration/software/agent-workflow.md, "Shared names".
Exit codes: 0 done, 1 failed, 2 usage error, 3 blocked.
"""

from __future__ import annotations

import argparse
import fnmatch
import json
import sys
import xml.etree.ElementTree as ET
from datetime import date, datetime
from pathlib import Path

import devtools

STAGE_NAMES = ("host", "firmware_build", "deploy", "tier_a", "tier_b", "tier_c", "ros2_pc")
STAGE_TIER = {"tier_b": "guided"}
TIER_B_FILES = (
    "BalanceController.*",
    "rc_mode.h",
    "OverspeedGuard.*",
    "WheelBoundary.*",
    "link_endpoint.*",
    "link_subscriptions.*",
    "OllieFOCdrive.ino",
)
ROS_COMMAND = "source /opt/ros/jazzy/setup.bash && cd src/ros2_pc/ros2 && colcon build && colcon test && colcon test-result --verbose"
PYTEST_TIMEOUT = 1800
DEPLOY_TIMEOUT = 3600
GUIDED_TIMEOUT = 3600
ROS_TIMEOUT = 1800
GUIDED_PENDING = "guided tests pending: run python3 scripts/verify.py --guided"
EXIT_CODES = {"done": 0, "failed": 1, "blocked": 3}
SEVERITY = {"pass": 0, "skipped": 0, "blocked": 1, "fail": 2}


class Refused(Exception):
    """A usage error: verify changes nothing and exits 2."""


def _now() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _today() -> str:
    return date.today().isoformat()


def _stage(status: str, detail: str, tests: dict[str, str] | None = None) -> dict:
    stage = {"status": status, "at": _now(), "detail": detail}
    if tests is not None:
        stage["tests"] = tests
    return stage


def _last_line(text: str) -> str:
    lines = [line for line in text.splitlines() if line.strip()]
    return lines[-1] if lines else ""


def _unreached(code: int) -> str:
    """Status of a stage that deploy.py did not report: blocked on exit 3, otherwise failed."""
    return "blocked" if code == 3 else "fail"


def _stage_of(data: dict | None, name: str) -> dict | None:
    return (data or {}).get("stages", {}).get(name)


def required_stages(changed: list[str]) -> list[str]:
    """The stages a change to these paths needs, in STAGE_NAMES order."""
    needed = {"host"}
    for path in changed:
        if path.startswith("src/ES-02/"):
            needed |= {"firmware_build", "deploy", "tier_a", "tier_c"}
            if any(fnmatch.fnmatchcase(Path(path).name, pattern) for pattern in TIER_B_FILES):
                needed.add("tier_b")
        elif path.startswith("src/navbot_link/"):
            needed |= {"deploy", "tier_a"}
        elif path.startswith("src/ros2_pc/"):
            needed.add("ros2_pc")
    return [name for name in STAGE_NAMES if name in needed]


def junit_outcomes(report: str) -> dict[str, str]:
    """Map each pytest node id in a JUnit report to passed, failed or skipped."""
    outcomes = {}
    for case in ET.fromstring(report).iter("testcase"):
        parts = case.get("classname", "").split(".")
        classes = [parts.pop()] if parts and parts[-1][:1].isupper() else []
        node = "::".join(["/".join(parts) + ".py", *classes, case.get("name", "")])
        if case.find("failure") is not None or case.find("error") is not None:
            outcomes[node] = "failed"
        elif case.find("skipped") is not None:
            outcomes[node] = "skipped"
        else:
            outcomes[node] = "passed"
    return outcomes


def _probes() -> list[tuple[str | None, dict]]:
    """The probe of this host (host None) and of its peer, in that order."""
    peer = devtools.peer()
    return [(None, devtools.probe(None)), (peer, devtools.probe(peer))]


def _link(probes: list[tuple[str | None, dict]]) -> tuple[str | None, str] | None:
    """The link host and its port: the multiplexer if one runs, else the USB port."""
    for host, info in probes:
        if info["mux"]:
            return host, f"tcp:{devtools.MUX_ADDR[0]}:{devtools.MUX_ADDR[1]}"
    for host, info in probes:
        if info["robot_usb"]:
            return host, info["robot_usb"]
    return None


def _deploy_run(worktree: Path, extra: list[str]) -> tuple[int, dict | None, str]:
    proc = devtools.run(
        None, ["python3", "scripts/deploy.py", *extra, "--json"], cwd=str(worktree), timeout=DEPLOY_TIMEOUT
    )
    try:
        data = json.loads(proc.stdout)
    except ValueError:
        data = None
    return proc.returncode, data, _last_line(proc.stderr)


def _host_tests(worktree: Path) -> dict:
    proc = devtools.run(None, ["python3", "-m", "pytest", "tests", "-q"], cwd=str(worktree), timeout=PYTEST_TIMEOUT)
    return _stage("pass" if proc.returncode == 0 else "fail", _last_line(proc.stdout) or _last_line(proc.stderr))


def _firmware_build(worktree: Path) -> dict:
    parts = {}
    for image in ("control", "diagnostic"):
        code, data, err = _deploy_run(worktree, ["--build-only", "--image", image])
        stage = _stage_of(data, "firmware_build")
        parts[image] = dict(stage) if stage else _stage(_unreached(code), err or "no firmware_build result")
    worst = max(parts.values(), key=lambda stage: SEVERITY[stage["status"]])["status"]
    return _stage(worst, "; ".join(f"{image} {stage['detail']}" for image, stage in parts.items()))


def _deploy_stages(worktree: Path) -> dict[str, dict]:
    """The deploy and tier_a stages of one deploy.py run; a stage it did not reach takes its exit status."""
    code, data, err = _deploy_run(worktree, [])
    deploy = _stage_of(data, "deploy")
    tier_a = _stage_of(data, "tier_a")
    return {
        "deploy": dict(deploy) if deploy else _stage(_unreached(code), err or "no deploy result"),
        "tier_a": dict(tier_a) if tier_a else _stage(_unreached(code), "not reached"),
    }


def _guided(sha: str, probes: list[tuple[str | None, dict]]) -> dict:
    link = _link(probes)
    if link is None:
        return _stage("blocked", "robot not connected")
    host, port = link
    tree = devtools.ship_tree(host, sha)
    run = devtools.run(
        host,
        ["python3", "-m", "pytest", "tests/hil", "-m", "guided", "--guided", "--robot", port, "-s", "--junitxml", f"{tree}/guided.xml"],
        cwd=tree,
        timeout=GUIDED_TIMEOUT,
        interactive=True,
    )
    report = devtools.run(host, ["cat", f"{tree}/guided.xml"])
    if report.returncode:
        return _stage("fail", f"no JUnit report from the guided run (pytest exit {run.returncode})")
    try:
        tests = junit_outcomes(report.stdout)
    except ET.ParseError:
        return _stage("fail", "unreadable JUnit report from the guided run")
    failed = sum(1 for outcome in tests.values() if outcome == "failed")
    status = "pass" if run.returncode == 0 and not failed else "fail"
    return _stage(status, f"{len(tests)} tests, {failed} failed", tests=tests)


def _ros2(sha: str, probes: list[tuple[str | None, dict]]) -> dict:
    hosts = [host for host, info in probes if info["ros"]]
    if not hosts:
        return _stage("blocked", "no ROS host")
    host = hosts[0]
    tree = devtools.ship_tree(host, sha)
    proc = devtools.run(host, ["bash", "-c", ROS_COMMAND], cwd=tree, timeout=ROS_TIMEOUT)
    return _stage("pass" if proc.returncode == 0 else "fail", _last_line(proc.stdout) or _last_line(proc.stderr))


def _tier_c(acceptance: list[devtools.Acceptance], results: dict[str, dict]) -> dict:
    """Pass when every acceptance test passed in its tier's run; sets each item's status in place."""
    items = [item for item in acceptance if item.id not in STAGE_NAMES]
    if not items:
        return _stage("fail", "no acceptance tests listed in state.md")
    upstream = [results[name]["status"] for name in ("deploy", "tier_a") if name in results]
    if "fail" in upstream:
        return _stage("fail", "deploy failed")
    if "blocked" in upstream:
        return _stage("blocked", "deploy blocked")
    for item in items:
        if item.status == "skipped":
            continue
        tests = results.get("tier_b" if item.tier == "guided" else "tier_a", {}).get("tests", {})
        item.status = {"passed": "pass", "failed": "fail"}.get(tests.get(item.id), "pending")
    live = [item for item in items if item.status != "skipped"]
    failed = [item.id for item in live if item.status == "fail"]
    pending = [item.id for item in live if item.status == "pending"]
    if failed:
        return _stage("fail", f"failed: {', '.join(failed)}")
    if pending:
        return _stage("blocked", f"no result for {', '.join(pending)}")
    return _stage("pass", f"{len(live)} acceptance tests passed")


def _skip(acceptance: list[devtools.Acceptance], target: str, reason: str) -> None:
    if target == "host":
        raise Refused("host cannot be skipped")
    if target not in STAGE_NAMES and "::" not in target:
        raise Refused(f"{target} is neither a stage name nor a pytest node id (path::name)")
    text = f"{reason} ({_today()})"
    for item in acceptance:
        if item.id == target:
            item.status, item.skip = "skipped", text
            return
    acceptance.append(devtools.Acceptance(id=target, tier=STAGE_TIER.get(target, "unattended"), status="skipped", skip=text))


def _skipped(acceptance: list[devtools.Acceptance]) -> dict[str, str]:
    """Stage name to the skip text. Skipping deploy also skips tier_a, which only deploy.py produces."""
    skipped = {item.id: item.skip or "" for item in acceptance if item.status == "skipped" and item.id in STAGE_NAMES}
    if "deploy" in skipped:
        skipped.setdefault("tier_a", f"deploy skipped: {skipped['deploy']}")
    return skipped


def _result(stages: dict[str, dict], dirty: bool) -> str:
    statuses = {stage["status"] for stage in stages.values()}
    if "fail" in statuses:
        return "failed"
    if "blocked" in statuses:
        return "blocked"
    return "failed" if dirty else "done"


def _cell(text: str) -> str:
    return text.replace("|", "/").replace("\n", " ")


def _summary(branch: str, sha: str, result: str, required: list[str], stages: dict[str, dict], acceptance: list[devtools.Acceptance], dirty: bool) -> str:
    lines = [f"# verify {branch} at {sha[:8]}: {result}", "", "| Stage | Status | Detail |", "| --- | --- | --- |"]
    lines += [f"| {name} | {stages[name]['status']} | {_cell(stages[name]['detail'])} |" for name in required]
    if dirty:
        lines += ["", "Uncommitted changes to tracked files: the result cannot be done."]
    skips = [f"- {item.id}: {item.skip}" for item in acceptance if item.status == "skipped"]
    lines += ["", "Skipped:", *(skips or ["- none"])]
    return "\n".join(lines)


def _verify(args: argparse.Namespace) -> int:
    branch = devtools.current_branch()
    if branch is None:
        raise Refused("detached HEAD: run verify in a task worktree on its branch")
    task = devtools.task_dir(branch)
    state_path = task / "state.md"
    if not state_path.exists():
        raise Refused(f"no task state at {state_path}; create the task with python3 scripts/task.py start {branch}")
    state = devtools.read_state(state_path)
    if args.skip:
        _skip(state.acceptance, args.skip, args.reason)

    worktree = Path(devtools.git(["rev-parse", "--show-toplevel"]))
    sha = devtools.git(["rev-parse", "HEAD"])
    changed = devtools.git(["diff", "--name-only", f"{state.baseline}...HEAD"]).splitlines()
    required = required_stages(changed)
    skipped = _skipped(state.acceptance)
    prior = (devtools.read_evidence(task, sha) or {}).get("stages", {})
    probes = _probes() if args.guided or "ros2_pc" in required else []

    def runs(name: str) -> bool:
        return name in required and name not in skipped

    fresh: dict[str, dict] = {}
    if "host" in required:
        fresh["host"] = _host_tests(worktree)
    if runs("firmware_build"):
        fresh["firmware_build"] = _firmware_build(worktree)
    if runs("deploy"):
        fresh.update(_deploy_stages(worktree))
    if runs("ros2_pc"):
        fresh["ros2_pc"] = _ros2(sha, probes)
    if args.guided and "tier_b" not in skipped:
        fresh["tier_b"] = _guided(sha, probes)
    if runs("tier_b") and "tier_b" not in fresh:
        earlier = prior.get("tier_b")
        fresh["tier_b"] = earlier if earlier and earlier["status"] in ("pass", "fail") else _stage("blocked", GUIDED_PENDING)
    fresh["tier_c"] = _tier_c(state.acceptance, {**prior, **fresh})
    for name, text in skipped.items():
        fresh[name] = _stage("skipped", text)

    stages = {**prior, **fresh}
    devtools.write_state(state_path, state)
    dirty = bool(devtools.git(["status", "--porcelain", "--untracked-files=no"]))
    result = _result({name: stages[name] for name in required}, dirty)
    devtools.update_evidence(task, sha, branch, stages=fresh, verify={"result": result, "at": _now()})
    print(_summary(branch, sha, result, required, stages, state.acceptance, dirty))
    return EXIT_CODES[result]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run the stages a task needs for HEAD and record the result.")
    parser.add_argument("--guided", action="store_true", help="run the guided device tests (tier B) on the robot, with the user at it")
    parser.add_argument("--skip", metavar="ID", help="skip a stage or a pytest node id; needs --reason")
    parser.add_argument("--reason", help="why the skip is taken, in the user's words")
    args = parser.parse_args(argv)
    if bool(args.skip) != bool(args.reason):
        parser.error("--skip and --reason go together")
    try:
        return _verify(args)
    except Refused as err:
        print(f"verify: {err}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
