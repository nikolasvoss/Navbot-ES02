# navlink quick start

Talk to the robot from the ros2_pc or the desktop: read values, watch streams, change settings, record CSV files. Full reference: [navlink](navlink.md).

## 1. Start

On the ros2_pc, the link multiplexer owns the robot's UART. The default port is the multiplexer, so `navlink` works while the ROS bridge runs. Do not stop the bridge.

```bash
cd ~/Navbot-ES02
python3 scripts/navlink.py shell
```

From the desktop, open the tunnel to the multiplexer first, then run the same command in the desktop checkout:

```bash
ssh -fNL 7650:127.0.0.1:7650 cm5
```

Type `quit` when you are done. Over USB, use `--port /dev/ttyUSB0`. Opening that port can reset the robot, so add `--wait <seconds>` to retry until the robot answers.

## 2. The three kinds of things

| Kind | What it is | Example |
| --- | --- | --- |
| Signal | A value you can read or stream. Read-only. | `roll_deg`, `voltage_filtered_v` |
| Parameter | A setting you can change. | `speed_kp`, `servo1_trim` |
| Action | Something the robot does. Asks `y/N` first. | `motor_stop`, `calibrate` |

## 3. Commands

| Type | Effect |
| --- | --- |
| `list` | Show everything. `list motor*` shows only matching names. |
| `get roll_deg` | Read the current value. |
| `set speed_kp 0.12` | Change a parameter. It prints `old -> new`. |
| `sub @bal` | Stream a preset to the screen. |
| `sub roll_deg gyro_z_rad_s @50 -o a.csv` | Stream two signals at 50 Hz into `a.csv`. |
| `unsub 1` | Stop stream number 1. |
| `call motor_stop` | Run an action after you confirm with `y`. |
| `log on` / `log off` | Show or hide robot log messages. |
| `quit` | Stop all streams and exit. |

Presets are named signal lists defined in `scripts/navlink_presets.yaml`: `@trace` (13 signals), `@ctrl` (10), `@bal` (7) and `@drive` (28) at 100 Hz; `@health` (20 signals) at 1 Hz.

### One-shot commands

Each runs once and exits. Add `--json` for machine-readable output (`capture` has no `--json`). Every command takes `--port`, `--wait` and `--presets`.

| Command | Effect |
| --- | --- |
| `info` | Build ID, entry count and port. |
| `list [pattern]` | Entries whose name matches a glob. |
| `get <name>...` | Current values. |
| `set <param> <value>` | Changes a parameter and prints `old -> new`. |
| `call <action> [arg=value]... --yes` | Starts an action. Without `--yes` it sends nothing and exits 2. |
| `log --seconds <s> [--min-level WARN]` | Prints robot log events for `<s>` seconds. |
| `capture` | Records a CSV file, see section 4. |

```bash
python3 scripts/navlink.py get roll_deg --json
python3 scripts/navlink.py call motor_stop --yes
```

| Exit code | Meaning |
| --- | --- |
| 0 | Done. |
| 1 | The robot refused the request. |
| 2 | Usage error: unknown name, bad value, or `call` without `--yes`. |
| 3 | No connection: the port did not open, the robot did not answer, or the link died. |
| 130 | Interrupted with Ctrl-C. |

## 4. Record a file without the shell

```bash
python3 scripts/navlink.py capture --preset drive --seconds 30 -o drive-01.csv
```

The file has a `#` header with the firmware build and all parameters, then `t_us,<names>` rows. Robot log messages appear as `#` lines.

## 5. How many streams

- **At most 8 streams at once**, counted over all connections to the robot. The robot assigns the number; `unsub` frees it.
- **At most 48 signals per stream.**
- **Each port has a bandwidth budget.** A stream costs `(4 + 4 × signals + 26) × rate` bytes per second. `rate` is the stream's Hz, at most 710 (every control tick). The shell prints the cost when you start a stream. The robot refuses a stream that goes over the budget and names both numbers.

| Port | Budget |
| --- | --- |
| Robot UART, through the multiplexer | 135.0 kB/s, after 5 kB/s kept for odometry and IMU |
| USB (`/dev/ttyUSB0`) | 40.3 kB/s |

Examples at 100 Hz: `@bal` (7 signals) costs 5.8 kB/s, `@ctrl` 7.0 kB/s, `@trace` 8.2 kB/s and `@drive` (28 signals) 14.2 kB/s. `@health` costs 0.11 kB/s at 1 Hz. Four `@drive` streams fit on the UART (56.8 of 135.0 kB/s). Over USB two fit (28.4 of 40.3 kB/s) and a third is refused (42.6 kB/s).

A stream belongs to the connection that started it. Another connection cannot stop it and gets `slot in use`.

## 6. Rules and safety

- While CH5 is not at position 0, or the balance controller runs, the robot refuses a `set` unless the parameter is live. A live parameter may move by at most its step per `set`:

| Live parameter | Step while balancing |
| --- | --- |
| `speed_kp` | 0.05 |
| `speed_ki` | 0.02 |
| `tilt_p` | 1.0 |
| `gyro_damping` | 0.1 |
| `yaw_p` | 0.15 |
| `speed_cap` | 0.1 |
| `yaw_rate_cap` | 0.25 |
| `log_level` | any value |

- To change any other parameter, set CH5 to position 0 first.
- The robot refuses a value that breaks a rule, such as `trim_envelope` above `tilt_ref_limit`. The old value stays, and the message says why.

| CH5 | Balance | Allowed actions and sets |
| --- | --- | --- |
| 0 (end) | off | `reboot`, every `set` |
| 1 (middle) | set by `balance` | `balance`, live `set` |
| 2 (other end) | on | live `set` |

- `balance` works only at CH5 position 1. `reboot` works only at position 0 with balance off and no motor test or calibration running.
- `motor_current` (at most 0.35 A) and `motor_voltage` (at most 2.0 V) run for at most 5000 ms and stop when their lease runs out. Both and `calibrate` are refused while balance runs.
- The motor actions and `calibrate` move the motors. Lift the wheels first and keep a hand on the power. `motor_stop` stops them.
- Streams stop 5 seconds after the shell dies, so a crashed shell never leaves the robot streaming.

## 7. If something is wrong

| Message | Meaning |
| --- | --- |
| `no reply to get_catalog` | The robot is still booting, or the port is wrong. Add `--wait <seconds>`, and check that the multiplexer runs (or the desktop tunnel is open). |
| `unknown signal <name>` | The name is not in the robot's list. Use `list`. |
| `<name>: refused while balancing` | CH5 is not at position 0. Set it to 0, or use a live parameter from the table. |
| `<name>: step <x> over <y> while balancing` | A live parameter moved by more than its step. Change it in smaller steps. |
| `slot in use` | Another connection owns that stream. Only it can stop it; its lease ends 5 seconds after its last renewal. |
| `all 8 slots are in use` | Stop one of your own streams with `unsub` in the shell. |
| `over budget: ... kB/s of ... kB/s allowed` | Use fewer signals, a longer period, or another port. |
