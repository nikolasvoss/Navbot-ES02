# Agent workflow: first robot checks

Status: concluded

## Observation

On 2026-10-31 `scripts/deploy.py` ran from the desktop (`niko-desktop`, Arduino toolchain) with the robot's USB cable on the ros2_pc (`cm5`, multiplexer running, transmitter off).

- Commit `d893001`: build, gate, flash and boot check passed (build ID `d893001e`, `reset_reason` 1 after the flash). Tier A: 5 of 7 passed. `test_log_level_round_trip` counted the endpoint's own `param` log line, which is sent at every log level. `test_reboot` read `uptime_s` inside the 300 ms reboot delay and saw the old value; `reset_reason` was 3 afterwards, so the reboot itself worked.
- Commit `595bb80`, with both tests fixed: exit 0, all 7 tier A tests passed on the robot.
- After the link reboot, with the transmitter off: `rc_valid` false, `ch5_position` 0.
- Health, three reads 20 s apart, robot idle: `heap_free_b` and `heap_min_free_b` 294392, `stack_free_loop_b` 4668, `stack_free_link_b` 2992, unchanged between reads.

## Evidence

`deploy.py --json` results of both runs; `navlink get` output on the ros2_pc from the shipped tree `~/.cache/navbot-deploy/595bb802…`.

## Practical consequence

- An agent on the desktop can build, flash and check the robot on the ros2_pc with one command.
- The receiver keeps CH5 at 0 with the transmitter off after a link reboot, so `reboot` and non-live `set` work unattended; the RC rule change (plan task T14) is not needed for that case.
- Tier A limits: `HEAP_MIN_FREE_B` 235000 and `STACK_MIN_FREE_B` 2400, about 80 % of the idle values. Values while balancing are not measured.

Open: deploy started on the ros2_pc, deploy with the USB cable on the desktop, the transmitter-off reading after a power cycle, the guided tier B run, and the user's confirmation of the limits.

## 2026-11-01

- Commit `73139e8`: `python3 scripts/verify.py --guided` run by the user at the robot. Tier B: 7 of 7 passed; verify result `done`.
- `python3 scripts/deploy.py --json` started on the ros2_pc, from a worktree of `feat/agent-workflow` in `~/Navbot-ES02/.claude/worktrees/`, with the USB cable on the ros2_pc: built on the desktop, flashed, all 7 tier A tests passed, exit 0.
- The ros2_pc services moved from `~/Navbot-wt/main` to `~/Navbot-ES02/.claude/worktrees/main` (workspace rebuilt, both installers rerun). Afterwards all three services were active, `navlink info` answered and `/odom` published.
- The user skipped the transmitter-off reading after a power cycle.
- Commit `7ac895e`: `python3 scripts/deploy.py --json` started on the desktop with the USB cable on the desktop (`/dev/ttyUSB0`, CH340 `1a86:7523`): exit 0, all 7 tier A tests passed.
- The user skipped measuring the health values while balancing; the tier A limits stay at the idle-based 235000 B and 2400 B.
