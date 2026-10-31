#!/usr/bin/env python3
"""Build the committed HEAD, check that the robot is safe, flash it, check the boot and run tier A.

Steps, stages and exit codes: docs/integration/software/agent-workflow.md, "Command lines and exit codes".
Hosts, shipped trees and evidence come from scripts/devtools.py, so the same command runs on either host.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

import devtools

ROOT = Path(__file__).resolve().parents[1]
MUX_PORT = "tcp:127.0.0.1:7650"
USB_BAUD = "576000"
GATE_SIGNALS = ["balance_state", "motor_test_active", "rc_valid", "ch5_position"]
OUTPUT_DIRS = {"control": "flash", "diagnostic": "flash-diagnostic"}
DIAGNOSTIC_PROPERTY = "compiler.cpp.extra_flags=-DSENSOR_DIAGNOSTIC_MODE=1"
BOOT_WAIT_S = 40
BOOT_TIMEOUT = 120
BUILD_TIMEOUT = 1800
FLASH_TIMEOUT = 900
TIER_A_TIMEOUT = 900
EXIT_CODES = """exit codes: 0 done, 1 other error, 2 uncommitted changes, 3 blocked (no toolchain, robot not connected),
4 gate refused, 5 build failed, 6 flash failed, 7 boot check failed, 8 tier A failed,
9 port held by another process"""

EXIT_DONE, EXIT_OTHER, EXIT_DIRTY, EXIT_BLOCKED, EXIT_GATE, EXIT_BUILD, EXIT_FLASH, EXIT_BOOT, EXIT_TIER_A, EXIT_HOLDER = range(10)


class Host:
    """One of the two hosts. ssh is None for this host."""

    def __init__(self, ssh: str | None, info: dict):
        self.ssh = ssh
        self.info = info

    @property
    def name(self) -> str:
        return self.info["hostname"]


class Stop(Exception):
    """Ends the run with an exit code. stage defaults to the stage in progress."""

    def __init__(self, code: int, detail: str, stage: str | None = None, tests: dict[str, str] | None = None):
        super().__init__(detail)
        self.code = code
        self.detail = detail
        self.stage = stage
        self.tests = tests


class Run:
    """What the run has done so far: the output, and the evidence stages."""

    def __init__(self, image: str):
        self.image = image
        self.sha: str | None = None
        self.hosts: dict[str, str | None] = {"build_host": None, "usb_host": None, "link_host": None}
        self.stages: dict[str, dict] = {}
        self.stage: str | None = None
        self.raw_usb: str | None = None

    def begin(self, stage: str) -> None:
        self.stage = stage

    def pass_stage(self, detail: str = "", tests: dict[str, str] | None = None) -> None:
        self.stages[self.stage] = record("pass", detail, tests)
        self.stage = None


def record(status: str, detail: str, tests: dict[str, str] | None = None) -> dict:
    data = {"status": status, "at": datetime.now().astimezone().isoformat(timespec="seconds"), "detail": detail}
    if tests is not None:
        data["tests"] = tests
    return data


def tail(proc: subprocess.CompletedProcess) -> str:
    lines = (proc.stderr or proc.stdout or "").strip().splitlines()
    return lines[-1] if lines else f"exit {proc.returncode}"


def json_out(proc: subprocess.CompletedProcess) -> dict:
    try:
        return json.loads(proc.stdout)
    except json.JSONDecodeError:
        return {}


def check_clean(root: Path) -> None:
    changed = subprocess.run(
        ["git", "status", "--porcelain", "--untracked-files=no", "--", ".", ":(top,exclude)firmware-build-environment.json"],
        cwd=root, capture_output=True, text=True, check=True,
    ).stdout
    if changed.strip():
        raise Stop(EXIT_DIRTY, "commit the changes first; deploy builds the committed HEAD:\n" + changed.rstrip())


def probe_hosts() -> list[Host]:
    try:
        local = Host(None, devtools.probe(None))
        peer = devtools.peer()
    except (RuntimeError, ValueError) as error:
        raise Stop(EXIT_OTHER, str(error), "firmware_build")
    try:
        remote = Host(peer, devtools.probe(peer))
    except RuntimeError as error:
        raise Stop(EXIT_BLOCKED, f"{peer} did not answer the probe: {error}", "firmware_build")
    return [local, remote]


def select_hosts(hosts: list[Host], build_only: bool) -> tuple[Host, Host | None, Host | None]:
    build = next((h for h in hosts if h.info["toolchain"]), None)
    if build is None:
        raise Stop(EXIT_BLOCKED, "no toolchain on either host", "firmware_build")
    if build_only:
        return build, None, None
    usb_hosts = [h for h in hosts if h.info["robot_usb"]]
    if not usb_hosts:
        raise Stop(EXIT_BLOCKED, "robot not connected", "deploy")
    if len(usb_hosts) > 1:
        raise Stop(EXIT_OTHER, "robot USB on both hosts", "deploy")
    usb = usb_hosts[0]
    link = next((h for h in hosts if h.info["mux"]), usb)
    return build, usb, link


def link_port(link: Host) -> str:
    return MUX_PORT if link.info["mux"] else link.info["robot_usb"]


def navlink_args(link: Host) -> list[str]:
    return ["--port", link_port(link)] + ([] if link.info["mux"] else ["--baud", USB_BAUD])


def environment_detail(build: Host, tree: str, sha: str, root: Path) -> str:
    rewritten = devtools.run(build.ssh, ["cat", f"{tree}/firmware-build-environment.json"])
    if rewritten.returncode:
        return "firmware-build-environment.json was not rewritten"
    committed = devtools.git(["show", f"{sha}:firmware-build-environment.json"], cwd=root)
    if json.loads(rewritten.stdout) != json.loads(committed):
        return "firmware-build-environment.json differs from the committed file"
    return ""


def build_images(build: Host, tree: str, sha: str, image: str, root: Path) -> str:
    """Build the control image; the diagnostic image also builds both, so a build-only check covers both."""
    names = ["control", "diagnostic"] if image == "diagnostic" else ["control"]
    for name in names:
        argv = ["python3", "scripts/build_firmware.py", "--revision", sha, "--output-dir", f"{tree}/build/{OUTPUT_DIRS[name]}"]
        if name == "diagnostic":
            argv += ["--build-property", DIAGNOSTIC_PROPERTY]
        proc = devtools.run(build.ssh, argv, cwd=tree, timeout=BUILD_TIMEOUT)
        if proc.returncode:
            raise Stop(EXIT_BUILD, f"{name} build failed: {tail(proc)}")
    return environment_detail(build, tree, sha, root)


def copy_output(src_host: str | None, src: str, dst_host: str | None, dst: str) -> None:
    made = devtools.run(dst_host, ["mkdir", "-p", dst])
    if made.returncode:
        raise Stop(EXIT_FLASH, f"cannot create {dst}: {tail(made)}")
    source = f"{src_host}:{src}/" if src_host else f"{src}/"
    target = f"{dst_host}:{dst}/" if dst_host else f"{dst}/"
    copied = subprocess.run(["rsync", "-a", source, target], capture_output=True, text=True)
    if copied.returncode:
        raise Stop(EXIT_FLASH, f"rsync to {dst_host or 'this host'} failed: {tail(copied)}")


def gate(link: Host, tree: str) -> None:
    proc = devtools.run(link.ssh, ["python3", "scripts/navlink.py", "get", *GATE_SIGNALS, "--json", *navlink_args(link)], cwd=tree)
    if proc.returncode:
        raise Stop(EXIT_GATE, f"navlink could not read the gate signals: {tail(proc)}")
    reason = devtools.gate_ok(json_out(proc))
    if reason is not None:
        raise Stop(EXIT_GATE, f"refused: {reason}")


def flash(usb: Host, tree: str, out: str) -> None:
    proc = devtools.run(
        usb.ssh, ["python3", "scripts/upload_firmware.py", out, "--port", usb.info["robot_usb"]], cwd=tree, timeout=FLASH_TIMEOUT
    )
    if proc.returncode:
        raise Stop(EXIT_FLASH, f"upload failed: {tail(proc)}")


def boot_check(usb: Host, usb_tree: str, link: Host, link_tree: str, sha: str, run: Run) -> None:
    navlink = ["python3", "scripts/navlink.py"]
    try:
        info = devtools.run(
            link.ssh, [*navlink, "info", "--wait", str(BOOT_WAIT_S), "--json", *navlink_args(link)],
            cwd=link_tree, timeout=BOOT_TIMEOUT,
        )
        if info.returncode:
            raise Stop(EXIT_BOOT, f"no build ID: {tail(info)}")
        build = json_out(info).get("build", "")
        if not build.startswith(sha[:8]):
            raise Stop(EXIT_BOOT, f"build ID {build!r} does not start with {sha[:8]}")
        reset = devtools.run(link.ssh, [*navlink, "get", "reset_reason", "--json", *navlink_args(link)], cwd=link_tree)
        value = json_out(reset).get("reset_reason")
        if reset.returncode or value != 1:
            raise Stop(EXIT_BOOT, f"reset_reason {value} is not 1 (power-on)")
    except Stop:
        raw = devtools.run(usb.ssh, ["python3", "scripts/devtools.py", "raw-serial", usb.info["robot_usb"], "10"], cwd=usb_tree, timeout=BOOT_TIMEOUT)
        run.raw_usb = raw.stdout or raw.stderr
        raise


def read_junit(text: str) -> dict[str, str]:
    """Map each test's node id to passed, failed or skipped. Empty when the text is not JUnit XML."""
    try:
        root = ET.fromstring(text)
    except ET.ParseError:
        return {}
    outcomes = {}
    for case in root.iter("testcase"):
        node = "/".join(case.get("classname", "").split(".")) + ".py::" + case.get("name", "")
        tags = {child.tag for child in case}
        if tags & {"failure", "error"}:
            outcomes[node] = "failed"
        elif "skipped" in tags:
            outcomes[node] = "skipped"
        else:
            outcomes[node] = "passed"
    return outcomes


def tier_a(link: Host, tree: str, run: Run) -> None:
    xml = f"{tree}/hil.xml"
    proc = devtools.run(
        link.ssh,
        ["python3", "-m", "pytest", "tests/hil", "-m", "not guided", "--robot", link_port(link), "-q", "--junitxml", xml],
        cwd=tree, timeout=TIER_A_TIMEOUT,
    )
    tests = read_junit(devtools.run(link.ssh, ["cat", xml]).stdout)
    failed = [node for node, outcome in tests.items() if outcome == "failed"]
    if not failed and not proc.returncode and tests:
        run.pass_stage("", tests)
        return
    if failed:
        detail = f"failed: {', '.join(failed)}"
    elif not tests:
        detail = f"no JUnit results: {tail(proc)}"
    else:
        detail = f"pytest exit {proc.returncode}: {tail(proc)}"
    raise Stop(EXIT_TIER_A, detail, tests=tests)


def deploy(args: argparse.Namespace, run: Run, root: Path) -> None:
    check_clean(root)
    run.sha = devtools.git(["rev-parse", "HEAD"], cwd=root)
    build, usb, link = select_hosts(probe_hosts(), args.build_only)
    run.hosts = {
        "build_host": build.name,
        "usb_host": usb.name if usb is not None else None,
        "link_host": link.name if link is not None else None,
    }

    run.begin("firmware_build")
    build_tree = devtools.ship_tree(build.ssh, run.sha)
    run.pass_stage(build_images(build, build_tree, run.sha, args.image, root))
    if args.build_only:
        return

    run.begin("deploy")
    usb_tree = devtools.ship_tree(usb.ssh, run.sha)
    link_tree = devtools.ship_tree(link.ssh, run.sha)
    folder = OUTPUT_DIRS[args.image]
    if usb.ssh != build.ssh:
        copy_output(build.ssh, f"{build_tree}/build/{folder}", usb.ssh, f"{usb_tree}/build/{folder}")
    port = usb.info["robot_usb"]
    holders = devtools.run(usb.ssh, ["python3", "scripts/devtools.py", "port-holder", port], cwd=usb_tree).stdout.strip()
    if holders:
        raise Stop(EXIT_HOLDER, f"{port} is held by {holders}")
    gate(link, link_tree)
    flash(usb, usb_tree, f"{usb_tree}/build/{folder}")
    boot_check(usb, usb_tree, link, link_tree, run.sha, run)
    run.pass_stage()
    if args.image == "control":
        run.begin("tier_a")
        tier_a(link, link_tree, run)


def save_evidence(run: Run, root: Path) -> None:
    branch = devtools.current_branch(root)
    if run.sha is None or not run.stages or branch is None:
        return
    task = devtools.task_dir(branch, devtools.main_checkout(root))
    if task.is_dir():
        devtools.update_evidence(task, run.sha, branch, stages=run.stages)


def report(args: argparse.Namespace, run: Run, code: int) -> None:
    if args.json:
        result = {"sha": run.sha, "image": run.image, **run.hosts, "stages": run.stages, "raw_usb": run.raw_usb, "exit": code}
        print(json.dumps(result, indent=2))
        return
    print(f"deploy {run.image} at {(run.sha or '')[:8]}: exit {code}")
    for name, data in run.stages.items():
        print(f"{name}: {data['status']}" + (f" ({data['detail']})" if data["detail"] else ""))
    if run.raw_usb:
        print("raw USB output:\n" + run.raw_usb)


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build the committed HEAD, check the gate, flash the robot, check the boot and run tier A.",
        epilog=EXIT_CODES,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--image", choices=sorted(OUTPUT_DIRS), default="control",
                        help="control: the robot firmware; diagnostic: the sensor diagnostic image, built with the control image")
    parser.add_argument("--build-only", action="store_true", help="build and stop; no robot needed")
    parser.add_argument("--json", action="store_true", help="print the result as JSON")
    return parser.parse_args(argv)


def fail(run: Run, code: int, detail: str, stage: str | None = None, tests: dict[str, str] | None = None) -> None:
    """Record the stage that was in progress, or the given one, with the error text."""
    stage = stage or run.stage
    if stage is not None:
        run.stages[stage] = record("blocked" if code == EXIT_BLOCKED else "fail", detail, tests)
    print(detail, file=sys.stderr)


def main(argv: list[str] | None = None, root: Path = ROOT) -> int:
    args = parse_args(argv)
    run = Run(args.image)
    try:
        deploy(args, run, root)
        code = EXIT_DONE
    except Stop as stop:
        code = stop.code
        fail(run, code, stop.detail, stop.stage, stop.tests)
    except (subprocess.TimeoutExpired, RuntimeError) as error:
        code = EXIT_OTHER
        fail(run, code, str(error))
    save_evidence(run, root)
    report(args, run, code)
    return code


if __name__ == "__main__":
    sys.exit(main())
