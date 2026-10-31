# Agent workflow

Agents work on a task in its own worktree. Scripts and hooks decide when the task is done: `scripts/verify.py` runs the stages the task's changes need and records the result for the commit. Rules for agents are in [AGENTS.md](../../../AGENTS.md#tasks-build-and-verification).

## Tasks

A task is a branch with two places on disk:

| Place | Content | Removed by the sweep |
| --- | --- | --- |
| `.claude/worktrees/<slug>/` | the checked-out branch, the agent's working copy | yes |
| `work/<slug>/` (ignored by git) | `state.md`, `agents/<name>.md` (one file per parallel agent), `evidence/<sha>.json` (one file per commit) | yes |

`<slug>` is the branch name in lowercase, with `/` and every character outside `a-z`, `0-9`, `.` and `-` replaced by `-`. For example, `feat/Link firmware` becomes `feat-link-firmware`.

`state.md` starts with front matter, followed by the sections Contract, Plan, Decisions, Progress and Next step. The front matter holds:

- `branch` and `worktree`.
- `baseline`: the full SHA the task started from. Verification and the sweep count commits after it.
- `status`: `implementing`, `blocked` or `ready`.
- `acceptance`: one item per test or stage the task must pass, with `id`, `tier` (`unattended` or `guided`), `status` and, after a skip, `skip` (the user's words and the date).

Each `evidence/<sha>.json` holds the stage results for one commit: `status` (`pass`, `fail`, `blocked` or `skipped`), `at` and `detail` per stage, the per-test outcomes of tier A and tier B, and the `verify` result (`done`, `failed` or `blocked`).

`scripts/task.py` manages tasks:

| Command | Effect |
| --- | --- |
| `start <branch> [--from REF]` | Creates the branch from `REF` (default `main`), or reuses an existing branch, and creates the worktree and the task folder. Prints JSON with `branch`, `worktree`, `task_dir` and `state`. Starting a task that exists changes nothing. |
| `sweep [--dry-run]` | Removes finished tasks (see below). Prints one line per item. |
| `drop <branch>` | Removes the worktree and the task folder and keeps the branch. Refused with uncommitted changes. |
| `status` | Prints JSON with the current task's `status` and its evidence for HEAD (`done`, `failed`, `blocked` or `missing`). |

Exit codes: 0 done, 1 refused (reason on stderr), 2 usage error.

Claude Code creates sub-agent worktrees with names starting `agent-`. These use the branch `worktree-<name>` and have no task folder.

## Sweep

`task.py sweep` runs at each new session start. It fetches `origin main` and then removes:

| Removed | When all of these hold |
| --- | --- |
| A task's worktree, its local branch and its task folder | The branch is merged into `origin/main`; it has commits after the baseline; the worktree has no uncommitted changes; no process has its working directory in the worktree; no `/etc/systemd/system/navbot-*.service` unit names the worktree. |
| A sub-agent worktree (`agent-` name) and its branch | It has a commit; no process runs in it; no uncommitted changes; HEAD is also on another branch. |
| A task folder | Its branch no longer exists. |

Anything else stays, with the reason printed. A task without commits, a detached worktree and a task without `state.md` are kept. The sweep never deletes uncommitted work.

## Deploy

`python3 scripts/deploy.py [--image control|diagnostic] [--build-only] [--json]` builds the committed HEAD, checks that the robot is safe, flashes it, checks the boot and, for the control image, runs tier A. Which host builds and which host has the robot's USB cable is in [development hosts](dev-hosts.md).

| Step | What it does |
| --- | --- |
| 1 | Refuses uncommitted changes to tracked files (exit 2), except `firmware-build-environment.json`, which the build rewrites. Builds HEAD. |
| 2 | Checks both hosts and selects the build, USB and link hosts. |
| 3 | Copies HEAD with `git archive` to `~/.cache/navbot-deploy/<sha>/` on the build host (the 5 newest copies are kept) and builds it with `build_firmware.py --revision <sha>`. The diagnostic image adds `-DSENSOR_DIAGNOSTIC_MODE=1` and builds into `build/flash-diagnostic`. A difference in the rewritten `firmware-build-environment.json` is reported, not failed. `--build-only` stops here, without a robot. |
| 4 | Copies the build to the USB host when it is not the build host. |
| 5 | Checks that no other process holds the robot's USB port (exit 9). |
| 6 | Gate: reads `balance_state`, `motor_test_active`, `rc_valid` and `ch5_position` on the link host. It passes when `balance_state` and `motor_test_active` are 0 and either `rc_valid` or `ch5_position` is 0. A refusal, or a read that fails, stops before flashing (exit 4). There is no override. |
| 7 | Flashes with `upload_firmware.py` on the USB host. |
| 8 | Boot check: `navlink.py info --wait 40 --json` must report a build ID starting with the first 8 characters of HEAD, and `reset_reason` must be 1 (power-on). On failure, the output of the USB port is added as `raw_usb`. |
| 9 | Control image only: tier A on the link host, in the shipped tree. |
| 10 | Prints the result (text, or JSON with `--json`) and writes the stages to the task's evidence when the branch has a task folder. |

Exit codes: 0 done, 1 other error, 2 uncommitted changes, 3 blocked (no toolchain, robot not connected), 4 gate refused, 5 build failed, 6 flash failed, 7 boot check failed, 8 tier A failed, 9 port held by another process.

## Device tests

| Tier | Runs | Where | Content |
| --- | --- | --- | --- |
| host | unattended, every change | any host: `python3 -m pytest tests -q` | Python tests and the C++ host tests (`run_host_test.sh` in each area) |
| A | unattended, transmitter off | link host, by `deploy.py` | 7 tests of the firmware's link, health and parameters (below) |
| B | guided: the user moves CH5 and places the robot | link host, by the user through `verify.py --guided` | 7 tests of the RC link mode and balance control (below) |
| C | changes under `src/ES-02/` only | `verify.py` | Each acceptance item in `state.md` takes the result of its tier A or B test |

Without a running multiplexer, the gate and the checks use the USB port directly, and opening the CH340 can reset the robot.

Tests that need a robot are marked `robot` and skip without `--robot`. Tests marked `guided` run only with `--guided`.

Tier A (`tests/hil/test_tier_a.py`):

| Test | Checks |
| --- | --- |
| `test_catalog_matches_signals_def` | The robot's signal names equal `src/ES-02/OllieFOCdrive/signals.def.h`, in order. Robot only. |
| `test_stream_rate_and_no_lost_frames` | The `trace` preset gives 1000 rows in 10 s within 10 %, and no lost frames. Robot only. |
| `test_health_within_limits` | `heap_min_free_b` at least 235000 B, `stack_free_loop_b` and `stack_free_link_b` at least 2400 B, `uptime_s` rises. |
| `test_link_counters_not_rising` | The six `link_*` counters are unchanged over 5 s. |
| `test_log_level_round_trip` | At `log_level` 3, no INFO log line arrives in 5 s except the endpoint's own `param`, `action` and `link` lines; the old level is restored. |
| `test_param_set_restore` | `speed_cap` takes 0.25 and is restored. |
| `test_reboot` | `reboot` returns within 40 s, `uptime_s` falls and `reset_reason` is 3. |

The two health limits are the constants `HEAP_MIN_FREE_B` and `STACK_MIN_FREE_B` in `test_tier_a.py`.

Tier B (`tests/hil/test_tier_b.py`), in file order. Each test prints the step for the user and polls the robot for up to 60 s:

| Test | Checks |
| --- | --- |
| `test_ch5_position_follows_switch` | `ch5_position` follows CH5 at 0, 1 and 2. |
| `test_reboot_refused_while_ch5_requests` | `reboot` is refused with `CH5 must be 0 and balance off` while CH5 requests balance. |
| `test_balance_runs_at_ch5_2` | With the robot on the floor, CH5 at 2 starts balance (`balance_state` 2). |
| `test_balance_keeps_running_at_ch5_1` | Balance continues at CH5 1. |
| `test_link_balance_off_and_on` | `balance` off and on through the link. |
| `test_command_stops_after_timeout` | A link `Command` of 0.1 m/s for 2 s ends when the sending stops. |
| `test_ch5_moves_start_and_end_balance` | Balance starts only at CH5 2 and ends when CH5 leaves it. |

## Verification

`python3 scripts/verify.py [--guided] [--skip ID --reason TEXT]` picks the stages from the files that changed since the baseline (`git diff --name-only <baseline>...HEAD`):

| Changed path | Stages |
| --- | --- |
| any path | `host` |
| `src/ES-02/**` | `firmware_build` (both images), `deploy`, `tier_a`, `tier_c`; also `tier_b` when a balance, RC, overspeed, wheel-boundary, link endpoint, subscription or sketch file changed |
| `src/navbot_link/**` | `deploy`, `tier_a` |
| `src/ros2_pc/**` | `ros2_pc`: `colcon build`, `colcon test` and `colcon test-result` on the ROS host |

Stage names: `host`, `firmware_build`, `deploy`, `tier_a`, `tier_b`, `tier_c`, `ros2_pc`.

- `tier_c` passes when every acceptance item passes in its tier's results. It fails with no acceptance items in `state.md`.
- A required `tier_b` without a result is blocked: `guided tests pending: run python3 scripts/verify.py --guided`. The user runs `--guided` in a terminal.
- The result is `done` when every required stage is `pass` or `skipped` and the tracked files are clean. It is `blocked` when a stage is blocked and none failed, and `failed` otherwise. The result is written to the evidence, and the Markdown summary lists every skip with its reason. Exit codes: 0 done, 1 failed, 2 usage error, 3 blocked.

### Skips

The user decides every skip; the reason is the user's words, verbatim. `--skip` takes a stage name other than `host`, or a pytest node id (`path::name`). The skip and its date go into the acceptance item in `state.md` and into the summary. `host` cannot be skipped. Skipping `deploy` also skips `tier_a`.

## Hooks and permissions

`.claude/settings.json` (tracked) registers four hooks with `scripts/claude_hooks.py`. Each hook reads its payload from stdin:

| Hook | Effect |
| --- | --- |
| `SessionStart`, `startup` | Runs the sweep and prints the removed items. |
| `SessionStart`, `compact` or `resume` | Prints the branch's `state.md` (at most 8000 characters). |
| `WorktreeCreate` | Creates `agent-` worktrees as `worktree-<name>` at HEAD, with no task folder. Other names run `task.py start`. Prints the worktree path. |
| `PreToolUse`, `Bash` | Exit 2 for a direct firmware upload (`upload_firmware.py`, `esptool`, `arduino-cli upload`) and for `git worktree add`. Exit 0 otherwise. |
| `Stop` | Exit 2 when the task is `implementing`, HEAD has commits after the baseline, the tracked files are clean, and HEAD has no `verify` result. |

Permissions allow `task.py`, `deploy.py`, `verify.py`, the read-only `navlink.py` commands (`info`, `list`, `get`, `log`, `capture`) and `python3 -m pytest`. The serial MCP server is denied: opening the USB port resets the robot.

## Python environment

`tests/requirements.txt` lists `pytest`, `PyYAML`, `protobuf`, `pyserial` and `lds2d[viz]` (the LiDAR service tests need it). They are installed in the default `python3` of each host; the scripts run `python3`, and there is no virtual environment. The host tests run with `python3 -m pytest tests -q`.

## Robot verification (2026-10-31)

- `deploy.py` from the desktop, with the robot's USB cable on the ros2_pc, at commit `595bb80`: exit 0, passed the gate, flashed, passed the boot check (build ID of HEAD, `reset_reason` 1 after the flash) and all 7 tier A tests.
- Tier A limits, from the idle baseline: `heap_min_free_b` 294392 B, `stack_free_loop_b` 4668 B, `stack_free_link_b` 2992 B. The limits are set to 235000 B and 2400 B, about 80 % of those values.
- Transmitter off after `navlink call reboot`: `rc_valid` 0 and `ch5_position` 0.

Open checks are listed in the [integration roadmap](../plans/roadmap.md#agent-workflow-on-the-robot).
