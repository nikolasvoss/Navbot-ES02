# Plan: agent workflow, deploy and definition of done

Status: **proposal, not implemented.** Branch `feat/agent-workflow`, worktree `/home/niko/Dokumente/Bastelei/roboter/Navbot-ES02/.claude/worktrees/feat-agent-workflow` on the desktop, based on `feat/link-firmware` at `9c3c553`. The device parts use the link features of [link-autonomy.md](link-autonomy.md): the multiplexer, `balance_state`, `ch5_position`, `rc_valid`, `motor_test_active` and `navlink --json --wait`. When the plan is implemented, task T13 moves its content into a software page and deletes this file.

Goal: the user says what they want; the agent implements it in its own worktree, verifies it on the host and on the robot with fixed scripts, and its working files disappear when the branch is merged. Agent instructions stay short; scripts and hooks do the rest.

## Decisions (user, 2026-10-31)

| # | Topic | Decision |
| --- | --- | --- |
| 1 | Task state | One folder per task in `work/`, shared by parallel agents, deleted automatically when the task's branch is merged |
| 2 | Worktrees | One fixed location: `<repo>/.claude/worktrees/<branch-slug>/` on every host |
| 3 | Codex | Not considered |
| 4 | Deploy | One script, the same on the desktop and the ros2_pc; it finds the toolchain and the robot's USB cable on every run |
| 5 | Hosts page | Minimal: cables and installed tools change |
| 6 | Agent instructions | Minimal; scripts and hooks enforce the rules |
| 7 | Device tests | Tier A unattended with the transmitter off, tier B guided by the user, tier C per task |
| 8 | Reset reason | A signal, checked after every boot. Crash dumps stay out |
| 9 | Skipping | The user may skip any hardware test or stage; host checks cannot be skipped |
| 10 | Unattended deploys | Allowed. The robot is always secured; the legs move and the wheels align at every boot |
| 11 | Robot model | A roadmap entry for a host simulation of the balancing robot |
| 12 | Deprecated files | Deleted |
| 13 | Uncommitted files | Deleted when clearly deprecated or older than two weeks (done, see below) |
| 14 | Implementation | Parallel Claude Haiku agents at medium effort |

## Done before the tasks (2026-10-31)

- Removed the 19 old worktrees in `.worktrees/` and `~/.codex/worktrees/`. Their uncommitted files were older than two weeks, or edited files that no longer exist in the tree. The commits that no branch contained are kept as `archive/logging-capture`, `archive/logging-engine` and `archive/touchscreen-build-switch`. No branch was deleted.
- Deleted from the desktop checkout: `TASK_STATE.md` (its open reminders are in the [integration roadmap](roadmap.md)), `agent_notes/`, `logs/`, `balance-trace-current.log`, `.aws/`, `.codex/`, `.agents/`, `src/RDK_X5/`, `tests/serial_logger/`, `tests/telemetry/`, the old folders in `build/`, and 139 entries of `work/`. Kept in `work/`: `localization.scope.md` (active branch), `verify-robust-control-plan.py`, the 2026-10-22 trial captures, and `recovered-bugs-register.md`.
- Deleted the old binaries in `build/flash` on the ros2_pc.
- Kept on purpose: the flash MCP and its venv (removed in step O4 once `deploy.py` works), `src/ES-02/OllieFOCdrive/build/` (Arduino IDE output, ignored), and `~/Navbot-wt/main` on the ros2_pc, because the multiplexer and bridge services run from it (step O8).

## How it works

- A task is a branch. `task.py start <branch>` creates its worktree and its task folder. Everything an agent must not lose goes into the task folder; git holds the history.
- Parallel agents of one task share the lead's task folder. Each writes only its own file there, so nobody overwrites anyone.
- After compaction or resume, a hook prints the task's `state.md` into the context.
- `deploy.py` finds where the toolchain and the robot's USB cable are, builds the exact commit, checks that the robot is safe, flashes, checks the boot and runs the unattended device tests.
- `verify.py` decides from the changed paths which stages a task needs, runs them, and writes the result for the commit. "Done" is that result, not the agent's opinion.
- The task folder and worktree are removed when the branch is merged into `main`. Cleanup on commit or on opening a PR is not used: a task has many commits, and review fixes, CI and the guided tests come after the PR, all of which need the state.

## Shared names

Every task uses exactly these names. A task that needs a name not listed here stops and reports it.

### Paths

| Name | Value |
| --- | --- |
| `MAIN` | the parent of `git rev-parse --path-format=absolute --git-common-dir` |
| `slug(branch)` | the branch lowercased, `/` replaced by `-`, every other character outside `[a-z0-9.-]` replaced by `-` |
| worktree | `MAIN/.claude/worktrees/<slug>/` |
| task folder | `MAIN/work/<slug>/` with `state.md`, `agents/<name>.md`, `evidence/<sha>.json` (full 40-character SHA) |
| shipped tree | `~/.cache/navbot-deploy/<sha>/` on a host: `git archive <sha> -- src scripts tests firmware-build-environment.json`, extracted, then an empty file `.complete` |
| firmware output | `<shipped tree>/build/flash` (control image), `<shipped tree>/build/flash-diagnostic` (sensor diagnostic image) |

New tracked paths, which T1 allows in the layout rules: `.claude/settings.json`, `tests/conftest.py`, `tests/requirements.txt`, `tests/hil/`, `scripts/devtools.py`, `scripts/task.py`, `scripts/deploy.py`, `scripts/verify.py`, `scripts/claude_hooks.py`, `docs/integration/software/dev-hosts.md`.

### Hosts

`PEERS = {"niko-desktop": "cm5", "cm5": "desktop"}` in `scripts/devtools.py`: this host's name maps to the SSH alias of the other host. Any other hostname is refused with `unknown host <name>; add it to PEERS in scripts/devtools.py`.

`devtools.py probe` prints one JSON object for the host it runs on. It uses only the Python standard library, and needs nothing installed:

| Key | Value |
| --- | --- |
| `hostname` | `socket.gethostname()` |
| `toolchain` | output of `bash -lc 'command -v arduino-cli'`, or `null` |
| `robot_usb` | `/dev/<name>` of the one entry in `/sys/class/tty/ttyUSB*` or `/sys/class/tty/ttyACM*` whose USB device (the nearest parent of `device` with `idVendor`) has `idVendor` `1a86` and `idProduct` `7523`, or `null`; more than one is an error |
| `mux` | `true` when a TCP connect to `127.0.0.1:7650` succeeds within 1 s |
| `ros` | `true` when `/opt/ros/jazzy/setup.bash` exists |

Selection, always local host first:

- build host: the first host with `toolchain`; none: blocked, `no toolchain on either host`.
- USB host: the host with `robot_usb`; none: blocked, `robot not connected`; both: error, `robot USB on both hosts`.
- link host: the first host with `mux` (port `tcp:127.0.0.1:7650`); none: the USB host with its `robot_usb` port.
- ROS host: the first host with `ros`; none: blocked, `no ROS host`.

### Python environment

`tests/requirements.txt` lists `pytest`, `PyYAML`, `protobuf` and `pyserial`, unpinned. They are installed in the default `python3` of each host (user decision, 2026-10-31); every script and subprocess runs `python3`. There is no venv.

### `scripts/devtools.py` (T0)

```python
PEERS: dict[str, str]

@dataclass
class Acceptance:
    id: str                 # a pytest node id under tests/hil, or a stage name
    tier: str               # "unattended" | "guided"
    status: str             # "pending" | "pass" | "fail" | "skipped"
    skip: str | None = None # the user's words and date, only when skipped

@dataclass
class State:
    branch: str
    worktree: str
    baseline: str           # full SHA
    status: str             # "implementing" | "blocked" | "ready"
    acceptance: list[Acceptance]
    body: str               # everything after the front matter, unchanged

def main_checkout(cwd: Path | None = None) -> Path: ...
def slug(branch: str) -> str: ...
def worktree_dir(branch: str, main: Path | None = None) -> Path: ...
def task_dir(branch: str, main: Path | None = None) -> Path: ...
def current_branch(cwd: Path | None = None) -> str | None: ...
def read_state(path: Path) -> State: ...
def write_state(path: Path, state: State) -> None: ...
def read_evidence(task: Path, sha: str) -> dict | None: ...
def update_evidence(task: Path, sha: str, branch: str, stages: dict[str, dict] | None = None,
                    verify: dict | None = None) -> Path: ...
def peer() -> str: ...
def run(host: str | None, argv: list[str], *, cwd: str | None = None, timeout: float = 600,
        interactive: bool = False, input: bytes | None = None) -> subprocess.CompletedProcess[str]: ...
def ship_tree(host: str | None, sha: str) -> str: ...
def probe(host: str | None) -> dict: ...
def gate_ok(values: dict) -> str | None: ...
```

- `run`: `host=None` runs `argv` locally without a shell. Otherwise it runs `ssh -o BatchMode=yes -o ConnectTimeout=10 <host> bash -lc <quoted command>`, with `cd <cwd> &&` in front when `cwd` is set. `input` bytes go to stdin. Unless `interactive`, stdout and stderr are captured and decoded as UTF-8 with `errors="replace"`; `interactive` adds `-t` and inherits the terminal. It never raises on a non-zero exit code.
- `ship_tree`: `git archive` runs locally; the tar bytes are extracted on the host (`tar -x -C <dir>`, passed to `run` as `input` for the peer). It returns the tree's absolute path. If `.complete` exists it does nothing. It keeps the 5 newest shipped trees on that host and deletes older ones.
- `probe(None)` runs the probe in this process; `probe(peer)` runs `python3 <shipped tree>/scripts/devtools.py probe` there, so the caller ships the tree first.
- `gate_ok`: takes the values of `balance_state`, `motor_test_active`, `rc_valid` and `ch5_position`. It returns `None` when `balance_state == 0`, `motor_test_active == 0`, and `rc_valid == 0` or `ch5_position == 0`; otherwise it returns a reason that names the values. A missing value is a refusal. `deploy.py` and `tests/hil/conftest.py` both use it.
- `update_evidence`: replaces only the stages it gets, and sets the top-level `verify` key when given.
- CLI, used through `run` on the other host: `devtools.py probe`; `devtools.py raw-serial PORT SECONDS` opens the port at 576000 baud with DTR and RTS deasserted before opening (so the robot is not reset), and prints what arrives, decoded with `errors="replace"`; `devtools.py port-holder PORT` prints one `pid name` line per process with the port open (from `/proc/*/fd`).

### `state.md`

```markdown
---
branch: feat/example
worktree: /home/niko/Dokumente/Bastelei/roboter/Navbot-ES02/.claude/worktrees/feat-example
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

## Plan

## Decisions

## Progress

## Next step
```

### Evidence

```json
{
  "sha": "0123456789abcdef0123456789abcdef01234567",
  "branch": "feat/example",
  "stages": {
    "host": {"status": "pass", "at": "2026-11-02T10:15:00+01:00", "detail": "312 passed"},
    "tier_a": {"status": "pass", "at": "2026-11-02T10:21:00+01:00", "detail": "",
               "tests": {"tests/hil/test_tier_a.py::test_reboot": "passed"}},
    "tier_b": {"status": "skipped", "at": "2026-11-02T10:22:00+01:00", "detail": "",
               "skip": "the user's words, verbatim (2026-11-02)"}
  },
  "verify": {"result": "done", "at": "2026-11-02T10:22:00+01:00"}
}
```

Stage names: `host`, `firmware_build`, `deploy`, `tier_a`, `tier_b`, `tier_c`, `ros2_pc`. Stage status: `pass`, `fail`, `blocked`, `skipped`. `verify.result`: `done`, `failed`, `blocked`.

### Command lines and exit codes

`scripts/task.py`:

| Command | Effect |
| --- | --- |
| `start <branch> [--from REF]` | creates the branch from `REF` (default `main`) or reuses an existing one, the worktree, the task folder and `state.md` (status `implementing`, baseline = `REF`'s SHA for a new branch, `git merge-base <branch> main` for an existing one); prints JSON `{"branch", "worktree", "task_dir", "state"}`. An existing task prints the same JSON and changes nothing |
| `sweep [--dry-run]` | the cleanup below; one line per removed or skipped item |
| `drop <branch>` | removes the worktree (refused with uncommitted changes) and the task folder; keeps the branch |
| `status` | JSON `{"branch", "task_dir", "status", "evidence"}` for the current worktree's branch; `evidence` is `done`, `failed`, `blocked` or `missing` for HEAD |

Exit codes: 0 done, 1 refused (reason on stderr), 2 usage error.

`sweep` runs `git fetch origin main`, then for each worktree under `MAIN/.claude/worktrees/` and each task folder in `MAIN/work/`:

- Remove the worktree, delete the local branch with `git branch -d`, and delete the task folder when all of these hold: the branch is an ancestor of `origin/main`; the worktree has no uncommitted changes; no process has its working directory inside the worktree; no file `/etc/systemd/system/navbot-*.service` contains the worktree path.
- Worktrees whose directory name starts with `agent-` (subagent worktrees, branch `worktree-<name>`) are removed when they have no uncommitted changes and another branch contains their HEAD; they have no task folder.
- Task folders whose branch no longer exists are deleted.
- Anything else is skipped with the reason. Sweep never deletes uncommitted work.
- Files in `work/` that are not task folders are left alone.

`scripts/deploy.py [--image control|diagnostic] [--build-only] [--json]`:

1. Refuse uncommitted changes to tracked files (exit 2). Use the HEAD SHA.
2. `ship_tree` on the peer, then probe both hosts; select the build host and, unless `--build-only`, the USB and link hosts.
3. `ship_tree` on the build host. Build with `build_firmware.py --revision <sha>` into `build/flash`; the diagnostic image adds `--build-property compiler.cpp.extra_flags=-DSENSOR_DIAGNOSTIC_MODE=1 --output-dir build/flash-diagnostic`. Compare the rewritten `firmware-build-environment.json` with `git show <sha>:firmware-build-environment.json`; report a difference in `detail`, do not fail. With `--build-only`, stop here.
4. `ship_tree` on the USB and link hosts; copy the firmware output to the USB host with `rsync -a` when it differs from the build host (one of the two is always local).
5. `devtools.py port-holder <robot_usb>` on the USB host: a holder means exit 9, naming it.
6. Gate: `navlink.py get balance_state motor_test_active rc_valid ch5_position --json` on the link host, checked with `gate_ok`. A refusal, or a non-zero navlink exit code (robot not answering, firmware without one of the signals), is exit 4 with the reason; nothing is flashed. No override. A robot whose firmware cannot pass the gate is flashed once by the user with `python3 scripts/upload_firmware.py` on the USB host.
7. Flash: `upload_firmware.py <output dir> --port <robot_usb>` on the USB host.
8. Boot check on the link host: `navlink.py info --wait 40 --json`; the `build` field must start with `sha[:8]`. Then `navlink.py get reset_reason --json` must give `1` (power-on). On failure, add the output of `devtools.py raw-serial <robot_usb> 10` from the USB host as `raw_usb`.
9. Control image only: tier A on the link host with `python3 -m pytest tests/hil -m "not guided" --robot <link port> -q --junitxml <tree>/hil.xml`, run in the shipped tree; read the XML back for the per-test outcomes.
10. Print the JSON result `{"sha", "image", "build_host", "usb_host", "link_host", "stages", "raw_usb", "exit"}` (text summary without `--json`). `stages` has exactly the shape of the evidence `stages`: keys `firmware_build`, `deploy` and `tier_a` (only those reached), each `{"status", "at", "detail"}`, and `tier_a` also `"tests": {<node id>: "passed" | "failed" | "skipped"}`. Exit 3 gives the reached stage `blocked`; exits 4 to 9 give it `fail`. When the current branch has a task folder, write these stages to its evidence.

Exit codes: 0 done, 1 other error, 2 uncommitted changes, 3 blocked (no toolchain, robot not connected), 4 gate refused, 5 build failed, 6 flash failed, 7 boot check failed, 8 tier A failed, 9 port held by another process.

`scripts/verify.py [--guided] [--skip ID --reason TEXT]`:

| Changed since the baseline (`git diff --name-only <baseline>...HEAD`) | Required stages |
| --- | --- |
| any path | `host`: `python3 -m pytest tests -q` in the worktree |
| `src/ES-02/**` | `firmware_build` (both images, `deploy.py --build-only --image diagnostic`), `deploy` and `tier_a` (`deploy.py --json`), `tier_c`; `tier_b` when any of `BalanceController.*`, `rc_mode.h`, `OverspeedGuard.*`, `WheelBoundary.*`, `link_endpoint.*`, `link_subscriptions.*`, `OllieFOCdrive.ino` changed |
| `src/navbot_link/**` | `deploy` and `tier_a` |
| `src/ros2_pc/**` | `ros2_pc`: on the ROS host, in the shipped tree: `source /opt/ros/jazzy/setup.bash && cd src/ros2_pc/ros2 && colcon build && colcon test && colcon test-result --verbose` |

- `tier_c` passes when every unattended acceptance test passed in the `tier_a` results and every guided one in the `tier_b` results; each acceptance item's `status` in `state.md` is updated. An empty acceptance list fails `tier_c` with `no acceptance tests listed in state.md`; the user can skip `tier_c`.
- `deploy.py` exit 3 makes `deploy`, `tier_a` and `tier_c` blocked; other non-zero exits make them failed.
- A required `tier_b` without a passed or skipped result is blocked with `guided tests pending: run python3 scripts/verify.py --guided`.
- `--guided`: run by the user in a terminal. Runs `python3 -m pytest tests/hil -m guided --guided --robot <link port> -s --junitxml <tree>/guided.xml` on the link host with `interactive=True`, then writes `tier_b` and the guided `tier_c` results.
- `--skip ID --reason TEXT`: `ID` is an acceptance test id or a stage name other than `host`. Adds or updates the acceptance item with `status: skipped` and `skip: "<TEXT> (<date>)"`. `host` is refused (exit 2).
- Result: `done` when every required stage is `pass` or `skipped` and the tree is clean; `blocked` when a stage is blocked and none failed; otherwise `failed`. Written to `verify` in the evidence; a Markdown summary goes to stdout and lists every skip with its words. Exit codes: 0 done, 1 failed, 2 usage error, 3 blocked.

`scripts/claude_hooks.py <hook>` reads the hook's JSON on stdin:

| Hook | Behavior |
| --- | --- |
| `session-start` | `source` `startup`: runs `task.py sweep` and prints its lines only when something was removed or skipped. `compact` or `resume`: prints the current branch's `state.md` (at most 8000 characters) after the line `Task state (work/<slug>/state.md):` |
| `worktree-create` | reads `name`. Names starting with `agent-`: `git worktree add -b worktree-<name> MAIN/.claude/worktrees/<name> HEAD`, no task folder. Other names: runs `task.py start <name>` and takes `worktree` from its JSON. Prints only the absolute worktree path |
| `pre-bash` | reads `tool_input.command`. Exit 2 with `Flash only with python3 scripts/deploy.py; it checks that the robot is safe first.` when it matches `FLASH_PATTERN` below; exit 2 with `Create worktrees with python3 scripts/task.py start <branch>.` when it matches `WORKTREE_PATTERN`; otherwise exit 0. Reading or testing these files (`grep esptool`, `pytest tests/scripts/test_upload_firmware.py`) is not blocked |
| `stop` | exit 0 when `stop_hook_active` is true. Exit 2 with `Run python3 scripts/verify.py, or say why the task is blocked.` when the current branch's `state.md` has status `implementing`, the worktree has no uncommitted changes, HEAD differs from the baseline, and the evidence for HEAD has no `verify` result; otherwise exit 0 |

```python
FLASH_PATTERN = re.compile(r"(^|[;&|]\s*)(\S*python3?\s+\S*upload_firmware\.py|\S*esptool\S*(\s|$)|arduino-cli\s+upload)")
WORKTREE_PATTERN = re.compile(r"(^|[;&|]\s*)git\s+worktree\s+add\b")
```

### `.claude/settings.json` (T12)

```json
{
  "hooks": {
    "SessionStart": [
      {"matcher": "startup|compact|resume",
       "hooks": [{"type": "command", "command": "python3 \"$CLAUDE_PROJECT_DIR\"/scripts/claude_hooks.py session-start"}]}
    ],
    "WorktreeCreate": [
      {"hooks": [{"type": "command", "command": "python3 \"$CLAUDE_PROJECT_DIR\"/scripts/claude_hooks.py worktree-create"}]}
    ],
    "PreToolUse": [
      {"matcher": "Bash",
       "hooks": [{"type": "command", "command": "python3 \"$CLAUDE_PROJECT_DIR\"/scripts/claude_hooks.py pre-bash"}]}
    ],
    "Stop": [
      {"hooks": [{"type": "command", "command": "python3 \"$CLAUDE_PROJECT_DIR\"/scripts/claude_hooks.py stop"}]}
    ]
  },
  "permissions": {
    "allow": [
      "Bash(python3 scripts/task.py:*)",
      "Bash(python3 scripts/deploy.py:*)",
      "Bash(python3 scripts/verify.py:*)",
      "Bash(python3 scripts/navlink.py info:*)",
      "Bash(python3 scripts/navlink.py list:*)",
      "Bash(python3 scripts/navlink.py get:*)",
      "Bash(python3 scripts/navlink.py log:*)",
      "Bash(python3 scripts/navlink.py capture:*)",
      "Bash(python3 -m pytest:*)"
    ],
    "deny": ["mcp__serial"]
  }
}
```

The serial MCP is denied because opening the USB port resets the robot.

### Firmware (T6)

- `uint8_t resetReason`, set from `esp_reset_reason()` at the start of `setup()`, and `SIGNAL(reset_reason, "", resetReason)` in the "Link and system health" section of `signals.def.h`, after `uptime_s`. Values (ESP-IDF `esp_reset_reason_t`): 1 power-on, 2 external pin, 3 software, 4 panic, 5 interrupt watchdog, 6 task watchdog, 7 other watchdog, 8 deep sleep, 9 brownout, 10 SDIO, 11 USB, 12 JTAG, 13 eFuse, 14 power glitch, 15 CPU lockup.
- `SENSOR_DIAGNOSTIC_MODE` becomes `#ifndef SENSOR_DIAGNOSTIC_MODE` / `#define SENSOR_DIAGNOSTIC_MODE 0` / `#endif`, so the build property `compiler.cpp.extra_flags=-DSENSOR_DIAGNOSTIC_MODE=1` selects the diagnostic image. That property is empty in the installed ESP32 core 3.3.12.

### pytest (T4)

`tests/conftest.py` registers the options `--robot PORT` (default: none, which means the fake firmware) and `--guided` (flag), and the markers `guided` and `robot`. Without `--guided`, tests marked `guided` are skipped; tests marked `robot` are skipped without `--robot`. Options must live in this file because pytest only reads options from the first conftest on the path. It collects the host test scripts with `pytest_collect_file(file_path, parent)`: for `file_path.name == "run_host_test.sh"` it returns a `pytest.File` subclass whose `collect()` yields one `pytest.Item`.

`tests/hil/conftest.py` (T9) inserts `src` and `tests/navbot_link` into `sys.path`. Its `link` fixture opens `navbot_link.link.Link(port, baud)` with baud 576000 for a port containing `USB` or `ACM` and 2000000 otherwise (a `tcp:` port ignores it). With `--robot` and without `--guided`, it reads the four gate signals once per session before the first test and calls `pytest.exit(<reason>, returncode=4)` when `devtools.gate_ok` refuses, so a direct `pytest tests/hil --robot …` is gated too.

### Tier A tests (T9)

`tests/hil/test_tier_a.py`, in this order:

| Test | Checks |
| --- | --- |
| `test_catalog_matches_signals_def` | the robot's entry names, in order, equal `[e.name for e in parse(DEFAULT_FILE.read_text())]` from `scripts/signals_catalog.py`, loaded with `importlib.util.spec_from_file_location` as in `tests/scripts/test_navlink_presets.py` |
| `test_stream_rate_and_no_lost_frames` | the `trace` preset (`load_presets(...)["trace"]`) for 10 s gives 1000 rows within `RATE_TOLERANCE` (0.10) and no `FramesLost` |
| `test_health_within_limits` | `heap_min_free_b >= HEAP_MIN_FREE_B` (20000), `stack_free_loop_b` and `stack_free_link_b >= STACK_MIN_FREE_B` (512), `uptime_s` rises between two reads 2 s apart |
| `test_link_counters_not_rising` | the six `link_*` counters are equal in two reads 5 s apart |
| `test_log_level_round_trip` | `set log_level 3`, drain `link.events`, then no INFO event in 5 s of log, restore the old value |
| `test_param_set_restore` | `set speed_cap 0.25`, read back 0.25, restore the old value |
| `test_reboot` | `call reboot`, link back within 40 s, `uptime_s` lower than before, `reset_reason == 3` |

`test_catalog_matches_signals_def` and `test_stream_rate_and_no_lost_frames` are marked `robot`; the others also run against the fake firmware. The robot-only tests first run in robot check 1. The limits are named module constants; the orchestrator replaces them after robot check 3.

## Rules for every agent

1. Work only in `/home/niko/Dokumente/Bastelei/roboter/Navbot-ES02/.claude/worktrees/feat-agent-workflow` on branch `feat/agent-workflow`. Do not switch branches, commit, push, flash, run `deploy.py` against a robot, open a serial port, `ssh` to `cm5` or `desktop`, or run `systemctl`. Do not touch the main checkout: another session works there. The orchestrator commits each task.
2. Before you start, read `AGENTS.md`, this file up to this section, and your task.
3. Change only the files your task lists. If you need another file, stop and report it.
4. No legacy: delete what your task replaces. No compatibility shims, no fallbacks.
5. Match the surrounding code: Python standard library plus PyYAML, `argparse`, type hints and docstrings as in `scripts/navlink.py`; pytest with literal expected values; C++ host tests with a `run_host_test.sh` per folder.
6. Tests never use the network, a real `ssh`, a serial port or the robot. Pass `run` and `probe` in as parameters or monkeypatch them; test git behavior in temporary repositories under `tmp_path`.
7. Run tests with `python3 -m pytest`. Run your task's tests and `tests/layout/test_repo_layout.py`, and report the exact pass and fail counts. A failure in a file that another running task owns: run it again later, then report it.
8. Change an existing file only with Edit, never by rewriting it whole. No `git checkout`, `git stash`, `git reset`, or formatters over whole files.
9. Report in at most 300 words: files changed, test output, open questions.

## Schedule

Tasks start as soon as what they need is committed. Implementation agents: Agent tool, `subagent_type` `pstack:effort-medium`, `model` `haiku`.

| Task | Starts when |
| --- | --- |
| T1 layout rules | O1 done |
| T0 `devtools.py` | T1 committed |
| T2 robot docs cleanup | T1 committed |
| T3 navlink quick start | T1 committed |
| T4 test environment | T1 committed |
| T5 build revision | T1 committed |
| T6 firmware | T1 committed |
| T7 `task.py` | T0 committed |
| T8 `deploy.py` | T0 and T5 committed |
| T9 tier A tests | T4 committed |
| T10 `verify.py` | T0 and T4 committed |
| T11 tier B tests | T9 committed |
| T12 hooks and settings | T7 and T10 committed |
| T13 docs | every other task committed, robot checks 1 to 3 done |
| T14 RC rule | robot check 2 shows `ch5_position != 0` with the transmitter off |

T1 runs alone first, because the layout test refuses the new paths of the others until it is committed. Then T0 and T2 to T6 run at the same time and share no file. So do T7 to T10.

## Orchestrator

- O1: this plan and its index line are committed (2026-10-31). The test dependencies are installed in the default `python3` on both hosts (done 2026-10-31). Check that `feat/simulation` has not already added a simulation entry to the robot roadmap; if it has, drop that item from T2. Whoever runs task T8 of [link-autonomy.md](link-autonomy.md) leaves `navlink-quick-start.md` to T3.
- O2, for each task: run its tests, `python3 -m pytest tests -q` and the layout test; commit its files as `<area>: <what>`. Then start a Sonnet review (`pstack:effort-medium`, `model` `sonnet`) of that commit against this plan, with findings by file and line; send accepted findings to a Haiku fix agent while other tasks keep running.
- O3: when T5, T6, T8 and T9 are committed, run robot check 1.
- O4: when robot check 1 passes, delete the flash MCP: `scripts/flash_mcp.py`, `scripts/flash_mcp_requirements.txt` and their lines in `.git/info/exclude`, `build/flash-mcp-venv`, and the `navbot_flash` entry in `~/.claude.json`, on both hosts.
- O5: robot check 2; start T14 if it applies.
- O6: robot check 3; replace the tier A limits; commit.
- O7: when T12 is committed, start one `claude --worktree hook-check` session, confirm that the `WorktreeCreate` payload has `name` and the worktree lands in `.claude/worktrees/`, then `task.py drop hook-check`.
- O8, with the user: on the ros2_pc, `git worktree move ~/Navbot-wt/main ~/Navbot-ES02/.claude/worktrees/main`, run `install_link_bridge_service.py` and `install_lidar_service.py` from there, restart the services, and check `/odom` and `navlink info`.
- O9: when T11 is committed, robot check 4 with the user.
- O10: T13; a Sonnet review at high effort (`pstack:effort-high`, `model` `sonnet`) of `git diff 9c3c553..HEAD` against this plan; fixes; then the final scope check by a fresh read-only agent on the strongest model. With the user's agreement, change the `TASK_STATE.md` rule in `~/.claude/CLAUDE.md` to `task.py`, and the Scope Contract location in the workflow skill to `state.md`. Finally run `verify.py` on this branch.

## T0: `scripts/devtools.py`

Files: `scripts/devtools.py`, `tests/scripts/test_devtools.py`.

Implement the API, the CLI and `PEERS` exactly as in [Shared names](#shared-names). Tests: `slug` cases (`feat/Link firmware` → `feat-link-firmware`), path functions in a temporary repository with a worktree, `read_state`/`write_state` round trip that keeps `body` unchanged, `update_evidence` replacing only the given stages, `run` building the exact `ssh` argument list (monkeypatch `subprocess.run`), `ship_tree` skipping when `.complete` exists and keeping 5 trees, `probe` on a fake `/sys/class/tty` tree with `socket` monkeypatched, `port-holder` on a fake `/proc` tree.

## T1: layout rules

Files: `AGENTS.md` (only the "Where files go" table), `.gitignore`, `tests/layout/test_repo_layout.py`.

- Table: a `.claude/` row (`settings.json`, tracked; `worktrees/`, ignored); the `work/` row becomes "One folder per task, `work/<branch-slug>/`, written by `scripts/task.py`; deleted when the branch is merged. Ignored by git."; the `tests/` row adds "`tests/conftest.py` and `tests/requirements.txt` serve every area".
- `.gitignore`: remove `/TASK_STATE.md`; add `/.claude/worktrees/` and `/.claude/settings.local.json`.
- Layout test: allow the new tracked paths of [Shared names](#shared-names). New check: every entry in the repository root is one of `ROOT_FILES`, `ROOT_DIRS`, `.git`, `.claude`, `work`, `build` or `.pytest_cache`. Prove it with a test that fails for an extra root entry in a temporary copy.

## T2: robot docs cleanup

Files: `docs/robot/software/control-flow.md` (delete), `docs/robot/software/balance-controller.md`, `docs/robot/README.md`, `docs/robot/hardware/system-overview.md` (only the link), `docs/robot/plans/robust-control-system.md` (only the links), `docs/robot/plans/roadmap.md`, `docs/robot/software/trace-capture.md`, `scripts/plot_channel_fft.py`, `scripts/reset_board.py` (delete).

- `control-flow.md`: check each statement against the firmware; move what is still true into a "Signal flow" section of `balance-controller.md`; delete the page and its index line. Point the links in `system-overview.md` and `robust-control-system.md` to `balance-controller.md#signal-flow`. Report any other page that links to it.
- Robot roadmap: rewrite the open items of "Verify sensor data and filters" and its numbered list for navlink captures (no CSV, no SerialPlot), delete what no longer applies, delete the empty section "Clean up controller start values and diagnostic switches". The simulation entry is not added here: branch `feat/simulation` owns it (commit 48e1f33).
- `plot_channel_fft.py`: run it on a navlink capture from `tests/scripts/fixtures/`. If it reads it, describe it in one paragraph of `trace-capture.md`; otherwise delete it. Report which.
- Delete `scripts/reset_board.py` and report every reference to it.

## T3: navlink quick start

Files: `docs/integration/software/navlink-quick-start.md`.

| Section | Change |
| --- | --- |
| 1 | No bridge stop: the multiplexer is the default port. The checkout on the ros2_pc is `~/Navbot-ES02`. From the desktop, first `ssh -fNL 7650:127.0.0.1:7650 cm5` |
| 3 | Add the one-shot commands `info`, `list`, `get`, `set`, `call --yes`, `log`, `capture` with `--json`, `--wait`, and the exit codes 0, 1, 2, 3, 130; add the `@health` preset |
| 5 | 8 streams (`kLinkMaxSlots`); recompute every example from the budgets in `link_subscriptions.h` and the presets |
| 6 | The live parameters and their steps, the three CH5 positions, the `reboot` and `balance` actions |
| 7 | Remove the bridge advice; "another connection owns the stream" |

Check every command against `scripts/navlink.py --help` and its subcommands.

## T4: test environment

Files: `tests/conftest.py`, `tests/requirements.txt`.

- `tests/requirements.txt` as in [Shared names](#shared-names).
- `tests/conftest.py`: the options and markers of [Shared names](#shared-names); collect every `tests/*/run_host_test.sh` as one test item named `<area>/run_host_test.sh` that runs it with `bash` and fails with the last 40 lines of output.
- Run `python3 -m pytest tests -q` and report every failure with its first error line. Fix nothing outside your files; the orchestrator assigns fixes (`tests/scripts/test_start_lidar.py` failed on 2026-10-25).

## T5: build revision

Files: `scripts/build_firmware.py`, `tests/scripts/test_build_firmware.py`.

`--revision SHA`: write the build ID `<SHA[:8]> <time>` without calling git and without the dirty check, for a source tree without `.git` (the shipped tree). Without the option, behavior stays as it is. Tests with literal IDs, as in the existing tests.

## T6: firmware

Files: `src/ES-02/OllieFOCdrive/signals.def.h`, `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`, `tests/link/test_signals_def.cpp` (only the stand-in for `resetReason`), `docs/integration/software/signal-table.md`.

`reset_reason` and the `SENSOR_DIAGNOSTIC_MODE` guard as in [Shared names](#shared-names). The diagnostic boot log line uses `resetReason`. Add the row and the value table to `signal-table.md`. Tests: `bash tests/link/run_host_test.sh`. The sketch is compiled by the orchestrator (`deploy.py --build-only` after T8, or `build_firmware.py` before it); compile errors come back to you.

## T7: `scripts/task.py`

Files: `scripts/task.py`, `tests/scripts/test_task.py`.

The commands, `sweep` rules and exit codes of [Shared names](#shared-names), using `devtools`. Tests in temporary repositories with a bare `origin`: start a new branch and an existing one; start twice prints the same JSON; sweep removes a merged clean task; sweep skips a dirty worktree, an unmerged branch, a worktree with a process inside (monkeypatched check), and a worktree named in a fake systemd directory (make the directory a parameter, default `/etc/systemd/system`); `agent-…` worktrees; drop refused with uncommitted changes; status for each evidence state.

## T8: `scripts/deploy.py` and the hosts page

Files: `scripts/deploy.py`, `tests/scripts/test_deploy.py`, `docs/integration/software/dev-hosts.md` (new), `docs/integration/README.md` (one index line), `scripts/build_remote.sh` (delete).

- The ten steps and exit codes of [Shared names](#shared-names), using `devtools`.
- `dev-hosts.md`, the whole page: "Two hosts take part: the desktop PC (`niko-desktop`, SSH `desktop`) and the ros2_pc (`cm5`, SSH `cm5`). Which of them has the Arduino toolchain and which has the robot's USB cable can change; `scripts/deploy.py` checks on every run. The robot's UART link is wired to the ros2_pc, whose multiplexer service shares it on `127.0.0.1:7650`."
- Tests with fake `run` and `probe`: every host combination (toolchain local, peer, none; USB local, peer, both, none; mux local, peer, none), uncommitted changes, port held, each gate input that refuses, build ID mismatch, reset reason other than 1, tier A failure from a JUnit XML fixture, the evidence written, every exit code.
- Report every remaining reference to `build_remote.sh`.

## T9: tier A tests

Files: `tests/hil/conftest.py`, `tests/hil/test_tier_a.py`, `tests/navbot_link/fake_firmware.py` (additions only).

- `tests/hil/conftest.py`: the path setup, `link` fixture and gate of [Shared names](#shared-names); without `--robot` the fixture starts the fake firmware as `tests/navbot_link/` does.
- The tests, markers and constants of [Shared names](#shared-names).
- Add to the fake, as `FakeEntry` rows: `reset_reason`, `uptime_s`, `heap_free_b`, `heap_min_free_b`, `stack_free_loop_b`, `stack_free_link_b` and the six `link_*` counters as U32 signals; `log_level` as a U32 PARAM (limits 1 to 4, default 2, `live_step` `inf`); `speed_cap` as an F32 PARAM (default 0.3, `live_step` 0.1); a `reboot` action. Behavior: `_log` drops events below the current `log_level`; `uptime_s` is the whole seconds since a `boot_monotonic` time, not the subscription clock; `reboot` calls `restart()`, sets `reset_reason` to 3 and resets `boot_monotonic`.
- Tests: `python3 -m pytest tests/hil tests/navbot_link -q`.

## T10: `scripts/verify.py`

Files: `scripts/verify.py`, `tests/scripts/test_verify.py`.

The stage selection, `--guided`, `--skip`, result rules and exit codes of [Shared names](#shared-names), using `devtools`; `deploy.py` is called as a subprocess. Tests with fake `run` and fake `deploy.py` results: each path class selects its stages; tier B trigger files; a skip with the user's words; `host` cannot be skipped; blocked versus failed; tier C mapping from JUnit results; the evidence and the summary text, as literal strings.

## T11: tier B tests

Files: `tests/hil/test_tier_b.py`.

Every test is marked `guided` and `robot`. A helper prints one instruction, then polls a signal until it has the expected value (timeout 60 s, then fail). Steps, in order:

1. CH5 to 0, 1 and 2: `ch5_position` follows.
2. CH5 at 1, then 2: `call reboot` is refused with `CH5 must be 0 and balance off`.
3. With the robot on the floor, CH5 at 2: balance runs (`balance_state == 2`); CH5 to 1: it keeps balancing.
4. `call balance on=0`, then `on=1`.
5. `Link.command(0.1, 0.0)` for 2 s (not ROS); stop sending: the demand returns to zero within the command timeout.
6. CH5 from 1 to 0: balance ends. CH5 from 0 to 2 in one move: balance starts only at 2. From 2 to 0 in one move: it ends.

## T12: hooks and settings

Files: `scripts/claude_hooks.py`, `tests/scripts/test_claude_hooks.py`, `.claude/settings.json`.

The four hooks and the settings file exactly as in [Shared names](#shared-names). Tests feed the JSON payloads on stdin and assert exit codes, stdout and stderr as literal strings; `task.py` and the evidence come from temporary repositories.

## T13: docs

Files: `AGENTS.md` (only "Build and flash"), `README.md` (only the build and flash paragraph), `docs/robot/software/usb-serial-flashing.md`, `docs/robot/software/sensor-diagnostic-mode.md`, `docs/robot/software/firmware-build-design.md`, `docs/integration/software/agent-workflow.md` (new), `docs/integration/README.md`, this file (delete).

- `AGENTS.md`: replace the section "Build and flash" with:

  ```markdown
  ## Tasks, build and verification

  - Start or resume a task with `python3 scripts/task.py start <branch>`. Work in the worktree it prints and keep its `state.md` current.
  - Put firmware on the robot only with `python3 scripts/deploy.py`.
  - A task is done when `python3 scripts/verify.py` passes. Only the user can skip a hardware test; copy their words into `state.md`.
  ```

- `README.md`: the build and flash paragraph becomes one sentence that points to `deploy.py` and the hosts page.
- `usb-serial-flashing.md`: keep the verified connection facts; replace the MCP, `build_remote.sh` and SerialPlot procedures with `deploy.py`.
- `sensor-diagnostic-mode.md`: build and flash with `deploy.py --image diagnostic`.
- `firmware-build-design.md`: `--revision`.
- `agent-workflow.md`: the implemented state of this plan: task folders and `state.md`, worktrees, sweep, `deploy.py`, the device test tiers, `verify.py`, skips, hooks. Add it to the index and delete this plan and its index line.
- Report every remaining reference to `TASK_STATE.md`, `flash_firmware`, `navbot_flash`, `build_remote.sh` and `reset_board.py` in the tree.

## T14: RC rule (only if robot check 2 needs it)

Files: `src/ES-02/OllieFOCdrive/rc_mode.h`, `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`, `tests/rc_mode/test_rc_mode.cpp`.

`RcMode::BalanceRequested(bool rcValid, uint8_t position, bool active)` returns `(rcValid && position != 0) || active`, and `balanceRequested()` uses it with `rcValid` from the current tick. A lost remote still blocks Arming, and a balancing robot still refuses through `active`. Tests cover every combination of the three arguments.

## Robot checks

The robot is secured. The transmitter is off unless a step says otherwise.

1. First deploys: `deploy.py` from the desktop with the USB cable on the ros2_pc, from the ros2_pc, and once with the cable on the desktop. Each: exit 0, build ID equals HEAD, `reset_reason` after the flash is 1. If the robot's current firmware cannot pass the gate (exit 4, signals missing), the user flashes this branch once by hand with `python3 scripts/upload_firmware.py`, then the deploys run. If the measured value is not 1, record it, and the orchestrator changes the expected value in `deploy.py` and this plan.
2. Transmitter off, after a power cycle by the user and after `navlink call reboot --yes`: `navlink get rc_valid ch5_position --json`. `ch5_position != 0` starts T14.
3. Tier A baseline: three runs; set `HEAP_MIN_FREE_B` and `STACK_MIN_FREE_B` to about 80 % of the lowest values seen, and confirm them with the user.
4. With the user: `python3 scripts/verify.py --guided`.

Record the results in a lab note `lab/integration/<date>-agent-workflow-robot-checks.md`.
