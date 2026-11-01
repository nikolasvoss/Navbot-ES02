#!/usr/bin/env python3
"""devtools: paths, task state, evidence, host probe, ssh and serial helpers for the agent workflow.

Standard library only for the probe and the CLI (the probe runs on hosts without extra packages);
task state uses PyYAML. Plan: docs/integration/software/agent-workflow.md
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shlex
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

PEERS = {"niko-desktop": "cm5", "cm5": "desktop"}
SYS_TTY = Path("/sys/class/tty")
PROC = Path("/proc")
ROS_SETUP = Path("/opt/ros/jazzy/setup.bash")
MUX_ADDR = ("127.0.0.1", 7650)
ROBOT_USB_IDS = ("1a86", "7523")
SHIP_PATHS = ["src", "scripts", "tests", "firmware-build-environment.json"]
SHIPPED_TREES = 5
SSH_OPTIONS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=10"]
SIM_SKETCH = "src/ES-02/OllieFOCdrive"
SIM_SKETCH_FILES = (
    "BalanceController.cpp", "BalanceController.h", "BoardConfig.h", "Calibration.h", "ControlTick.cpp",
    "ControlTick.h", "filter.cpp", "filter.h", "ICM42688.h", "imu.cpp", "imu.h", "LegPoseRamp.h",
    "link_endpoint.cpp", "link_endpoint.h", "link_frame.cpp", "link_frame.h", "link_log.cpp", "link_log.h",
    "link_schedule.h", "link_subscriptions.cpp", "link_subscriptions.h", "link_tx.cpp", "link_tx.h",
    "link_uart.h", "Logging.cpp", "Logging.h", "MahonyFilter.cpp", "MahonyFilter.h", "MathConstants.h",
    "MotorDiagnostic.h", "OverspeedGuard.cpp", "OverspeedGuard.h", "rc_mode.h", "RobotConfig.cpp",
    "RobotConfig.h", "ServoLimits.h", "settings_groups.h", "signals.def.h", "signal_table_expand.h",
    "signal_table.h", "WheelBoundary.cpp", "WheelBoundary.h",
)
SIM_INPUTS = (
    "src/sim",
    "tests/simulation",
    "tests/conftest.py",
    "src/navbot_link",
    "scripts/signals_catalog.py",
    f"{SIM_SKETCH}/src/navbot_link",
    *(f"{SIM_SKETCH}/{name}" for name in SIM_SKETCH_FILES),
)
SIM_CACHE = Path.home() / ".cache" / "navbot-sim" / "passed"
SIM_TIMEOUT = 900


@dataclass
class Acceptance:
    id: str
    tier: str
    status: str
    skip: str | None = None


@dataclass
class State:
    branch: str
    worktree: str
    baseline: str
    status: str
    acceptance: list[Acceptance]
    body: str


def git(args: list[str], cwd: Path | None = None) -> str:
    return subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True, text=True).stdout.strip()


def main_checkout(cwd: Path | None = None) -> Path:
    return Path(git(["rev-parse", "--path-format=absolute", "--git-common-dir"], cwd)).parent


def slug(branch: str) -> str:
    return re.sub(r"[^a-z0-9.-]", "-", branch.lower().replace("/", "-"))


def worktree_dir(branch: str, main: Path | None = None) -> Path:
    return (main or main_checkout()) / ".claude" / "worktrees" / slug(branch)


def task_dir(branch: str, main: Path | None = None) -> Path:
    return (main or main_checkout()) / "work" / slug(branch)


def current_branch(cwd: Path | None = None) -> str | None:
    proc = subprocess.run(["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=cwd, capture_output=True, text=True)
    name = proc.stdout.strip()
    return None if proc.returncode or name == "HEAD" else name


def read_state(path: Path) -> State:
    import yaml

    text = path.read_text(encoding="utf-8")
    end = text.index("\n---\n", 3)
    meta = yaml.safe_load(text[4:end])
    return State(
        branch=meta["branch"],
        worktree=meta["worktree"],
        baseline=meta["baseline"],
        status=meta["status"],
        acceptance=[Acceptance(**item) for item in meta["acceptance"]],
        body=text[end + 5 :],
    )


def write_state(path: Path, state: State) -> None:
    import yaml

    class Dumper(yaml.SafeDumper):
        def increase_indent(self, flow=False, indentless=False):
            return super().increase_indent(flow, False)

    meta = {
        "branch": state.branch,
        "worktree": state.worktree,
        "baseline": state.baseline,
        "status": state.status,
        "acceptance": [{k: v for k, v in vars(a).items() if v is not None} for a in state.acceptance],
    }
    front = yaml.dump(meta, Dumper=Dumper, sort_keys=False, allow_unicode=True)
    path.write_text(f"---\n{front}---\n{state.body}", encoding="utf-8")


def read_evidence(task: Path, sha: str) -> dict | None:
    path = task / "evidence" / f"{sha}.json"
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else None


def update_evidence(
    task: Path, sha: str, branch: str, stages: dict[str, dict] | None = None, verify: dict | None = None
) -> Path:
    data = read_evidence(task, sha) or {"sha": sha, "branch": branch, "stages": {}}
    data["stages"].update(stages or {})
    if verify is not None:
        data["verify"] = verify
    path = task / "evidence" / f"{sha}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    return path


def peer() -> str:
    name = socket.gethostname()
    if name not in PEERS:
        raise ValueError(f"unknown host {name}; add it to PEERS in scripts/devtools.py")
    return PEERS[name]


def _text(data: bytes | None) -> str | None:
    return None if data is None else data.decode("utf-8", errors="replace")


def run(
    host: str | None,
    argv: list[str],
    *,
    cwd: str | None = None,
    timeout: float = 600,
    interactive: bool = False,
    input: bytes | None = None,
) -> subprocess.CompletedProcess[str]:
    if host is None:
        args = list(argv)
        local_cwd = cwd
    else:
        command = shlex.join(argv)
        if cwd is not None:
            command = f"cd {shlex.quote(cwd)} && {command}"
        args = ["ssh", *SSH_OPTIONS, *(["-t"] if interactive else []), host, "bash -lc " + shlex.quote(command)]
        local_cwd = None
    proc = subprocess.run(
        args,
        input=input,
        cwd=local_cwd,
        timeout=timeout,
        stdout=None if interactive else subprocess.PIPE,
        stderr=None if interactive else subprocess.PIPE,
    )
    return subprocess.CompletedProcess(args, proc.returncode, _text(proc.stdout), _text(proc.stderr))


def ship_tree(host: str | None, sha: str) -> str:
    home = run(host, ["sh", "-c", 'printf %s "$HOME"']).stdout
    base = f"{home}/.cache/navbot-deploy"
    tree = f"{base}/{sha}"
    if run(host, ["test", "-f", f"{tree}/.complete"]).returncode == 0:
        return tree
    archive = subprocess.run(["git", "archive", sha, *SHIP_PATHS], cwd=main_checkout(), check=True, capture_output=True).stdout
    script = (
        f"rm -rf {shlex.quote(tree)} && mkdir -p {shlex.quote(tree)} && tar -x -C {shlex.quote(tree)}"
        f" && touch {shlex.quote(tree + '/.complete')}"
        f" && cd {shlex.quote(base)} && ls -1t | tail -n +{SHIPPED_TREES + 1} | xargs -r rm -rf --"
    )
    proc = run(host, ["sh", "-c", script], input=archive)
    if proc.returncode:
        raise RuntimeError(f"shipping {sha} to {host or 'this host'} failed: {proc.stderr}")
    return tree


def probe(host: str | None) -> dict:
    if host is None:
        return _probe_here()
    proc = run(host, ["python3", "-", "probe"], input=Path(__file__).read_bytes())
    if proc.returncode:
        raise RuntimeError(f"probe on {host} failed: {proc.stderr}")
    return json.loads(proc.stdout)


def _probe_here() -> dict:
    return {
        "hostname": socket.gethostname(),
        "toolchain": _toolchain(),
        "robot_usb": _robot_usb(),
        "mux": _mux_up(),
        "ros": ROS_SETUP.exists(),
    }


def _toolchain() -> str | None:
    return subprocess.run(["bash", "-lc", "command -v arduino-cli"], capture_output=True, text=True).stdout.strip() or None


def _attr(directory: Path, name: str) -> str | None:
    path = directory / name
    return path.read_text().strip() if path.exists() else None


def _usb_device(entry: Path) -> Path | None:
    device = (entry / "device").resolve()
    for candidate in [device, *device.parents]:
        if (candidate / "idVendor").exists():
            return candidate
    return None


def _robot_usb() -> str | None:
    found = []
    for entry in sorted([*SYS_TTY.glob("ttyUSB*"), *SYS_TTY.glob("ttyACM*")]):
        usb = _usb_device(entry)
        if usb and (_attr(usb, "idVendor"), _attr(usb, "idProduct")) == ROBOT_USB_IDS:
            found.append(f"/dev/{entry.name}")
    if len(found) > 1:
        raise RuntimeError(f"more than one robot USB port: {', '.join(found)}")
    return found[0] if found else None


def _mux_up() -> bool:
    try:
        socket.create_connection(MUX_ADDR, timeout=1).close()
    except OSError:
        return False
    return True


def is_sim_input(path: str) -> bool:
    """A repo-relative path is a simulator input: equal to an entry of SIM_INPUTS or inside a directory entry."""
    return any(path == entry or path.startswith(entry + "/") for entry in SIM_INPUTS)


def sim_key(sha: str, cwd: Path | None = None) -> str:
    """sha256 hex digest of `git ls-tree -r <sha> -- *SIM_INPUTS`: the same input contents give the same key."""
    listing = git(["ls-tree", "-r", sha, "--", *SIM_INPUTS], cwd)
    return hashlib.sha256(listing.encode()).hexdigest()


def _last_line(text: str | None) -> str:
    lines = [line for line in (text or "").splitlines() if line.strip()]
    return lines[-1] if lines else ""


def sim_check(root: Path, sha: str) -> tuple[bool, str]:
    """(passed, detail). A key recorded by an earlier pass skips the run; a failure records nothing.

    The tests run in root, so root must be at sha with no uncommitted or untracked change to an input:
    otherwise a pass would be recorded for content that was not tested.
    """
    if git(["rev-parse", "HEAD"], root) != sha:
        return False, f"{root} is not at {sha[:8]}"
    changed = git(["status", "--porcelain", "--untracked-files=all", "--", *SIM_INPUTS], root)
    if changed:
        return False, "uncommitted changes to simulator inputs: " + " ".join(line.split(maxsplit=1)[1] for line in changed.splitlines())
    key = sim_key(sha, cwd=root)
    marker = SIM_CACHE / key
    if marker.exists():
        return True, f"inputs unchanged, passed before (key {key[:12]})"
    try:
        proc = run(None, ["python3", "-m", "pytest", "tests/simulation", "-q"], cwd=str(root), timeout=SIM_TIMEOUT)
    except subprocess.TimeoutExpired:
        return False, f"timed out after {SIM_TIMEOUT} s"
    if proc.returncode:
        return False, _last_line(proc.stdout) or _last_line(proc.stderr)
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.touch()
    return True, _last_line(proc.stdout)


def gate_ok(values: dict) -> str | None:
    keys = ["balance_state", "motor_test_active", "rc_valid", "ch5_position"]
    missing = [k for k in keys if values.get(k) is None]
    if missing:
        return f"missing signal(s): {', '.join(missing)}"
    if values["balance_state"] != 0:
        return f"balance_state={values['balance_state']}; the balance controller is active"
    if values["motor_test_active"] != 0:
        return f"motor_test_active={values['motor_test_active']}; a motor test is running"
    if values["rc_valid"] != 0 and values["ch5_position"] != 0:
        return f"rc_valid={values['rc_valid']} and ch5_position={values['ch5_position']}; the transmitter may drive the robot"
    return None


def port_holders(port: str) -> list[str]:
    lines = []
    pids = sorted((p for p in PROC.iterdir() if p.name.isdigit()), key=lambda p: int(p.name))
    for pid in pids:
        try:
            fds = list((pid / "fd").iterdir())
        except OSError:
            continue
        if any(_link(fd) == port for fd in fds):
            try:
                comm = (pid / "comm").read_text().strip()
            except OSError:
                continue
            lines.append(f"{pid.name} {comm}")
    return lines


def _link(path: Path) -> str | None:
    try:
        return os.readlink(path)
    except OSError:
        return None


def raw_serial(port: str, seconds: float) -> None:
    """Print the bytes from port for seconds.

    DTR and RTS are deasserted as early as pyserial allows. On Linux the kernel may still pulse DTR
    when the port opens, which can reset the board, so this does not guarantee that it never resets.
    """
    import serial

    link = serial.Serial()
    link.port = port
    link.baudrate = 576000
    link.timeout = 0.2
    link.dtr = False
    link.rts = False
    link.open()
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            data = link.read(link.in_waiting or 1)
            if data:
                sys.stdout.write(data.decode("utf-8", errors="replace"))
                sys.stdout.flush()
    finally:
        link.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Host helpers for the agent workflow.")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("probe", help="print this host's probe as JSON")
    raw = sub.add_parser(
        "raw-serial",
        help="print bytes from a port for SECONDS; DTR and RTS are deasserted as early as pyserial allows, "
        "but on Linux the kernel may still pulse DTR on open and reset the board",
    )
    raw.add_argument("port")
    raw.add_argument("seconds", type=float)
    holder = sub.add_parser("port-holder", help="list processes holding a port open")
    holder.add_argument("port")
    args = parser.parse_args(argv)
    if args.command == "probe":
        print(json.dumps(_probe_here()))
    elif args.command == "raw-serial":
        raw_serial(args.port, args.seconds)
    else:
        print("\n".join(port_holders(args.port)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
