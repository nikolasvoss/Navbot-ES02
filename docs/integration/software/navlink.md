# navlink: signals, parameters and actions over the link

Status: **implemented and host-tested against the firmware's entry table; run on the robot** for the catalog, parameters, subscriptions and the diagnostic image. Not yet run on the robot: the motor actions, `calibrate`, live parameter steps, `balance`, `reboot`, the link `Command`, the [link multiplexer](ros2-uart-protocol.md#link-multiplexer-on-the-ros2_pc) and the one-shot commands; these are open in the [roadmap](../plans/roadmap.md#link-autonomy-on-the-robot). A one-page guide is the [quick start](navlink-quick-start.md). The `navbot_link` library and the `navlink` tool speak the catalog, subscription, parameter and action messages of the [protocol](ros2-uart-protocol.md#catalog-subscriptions-parameters-and-actions). The firmware serves the table described in the [signal table](signal-table.md). The tool and the library were tested against the fake firmware in `tests/navbot_link/` and against the firmware's own endpoint code built for the host.

## Run

`navlink` is [scripts/navlink.py](../../../scripts/navlink.py). It needs `python3-serial`, `python3-protobuf` 3.21 and `python3-yaml`.

```bash
python3 scripts/navlink.py shell
python3 scripts/navlink.py capture --preset drive --seconds 30 -o drive-01.csv
```

- The default `--port` is `tcp:127.0.0.1:7650`, the [link multiplexer](ros2-uart-protocol.md#link-multiplexer-on-the-ros2_pc). The multiplexer owns the ros2_pc UART `/dev/ttyAMA3`, so `navlink` runs next to the ROS bridge. Do not pass `/dev/ttyAMA3` while the multiplexer runs: the port is already open.
- `/dev/ttyUSB0` is the robot's USB port, opened directly (no multiplexer). A port name with `USB` or `ACM` defaults to 576000 baud, any other to 2000000. `--baud` overrides it. Opening the CH340 can reset the board ([USB/serial facts](../../robot/software/usb-serial-flashing.md)); `navlink` keeps DTR and RTS off, which does not prevent it on every adapter.
- `--presets` selects the preset file; the default is [scripts/navlink_presets.yaml](../../../scripts/navlink_presets.yaml).

On connect, `navlink` reads the catalog entry by entry and resolves every name against it. A name the firmware does not have fails with `unknown signal <name>`.

## Shell

| Command | Effect |
| --- | --- |
| `list [pattern]` | Entries whose name matches the glob pattern, with kind, type and unit; for an action, its arguments and bounds; for a live parameter, `live ±<step>` (`live` alone when any change is allowed). |
| `get <name>...` | Current values. |
| `set <param> <value>` | Writes a parameter and prints `name: old -> new`. A bool takes `0`, `1`, `true`, `false`, `on` or `off`. |
| `call <action> [arg=value]...` | Checks the arguments against the catalog bounds, shows them and asks `y/N`. Only `y` sends. |
| `sub <names\|@preset>... [@<Hz>] [-o file.csv]` | Starts a subscription. `@<Hz>` overrides the preset rate; without a rate it streams every control tick. Without `-o` the rows are printed. |
| `unsub <slot>` | Stops a subscription and closes its file. |
| `log on\|off` | Shows or hides log events and raw text. |
| `quit` | Stops every subscription, closes all files and exits. |

```text
navlink> sub @bal -o balance-01.csv
slot 1: 7 entries @ 100 Hz, 5.8 kB/s -> balance-01.csv
navlink> set speed_kp 0.12
speed_kp: 0.08 -> 0.12
navlink> unsub 1
slot 1 closed
```

The load is the row size, `4 + 4 × entries` bytes, plus 26 bytes of frame cost, times the rate. The firmware budgets the same figure, so a subscription that the shell reports as fitting is accepted by the robot.

## One-shot commands

Each one connects, does one thing, prints and exits. Scripts and agents use these; the shell does the same things interactively.

| Command | Effect |
| --- | --- |
| `info` | Build ID, entry count and port. |
| `list [pattern]` | Entries, as the shell lists them. |
| `get NAME...` | Current values. |
| `set PARAM VALUE` | Writes a parameter, as `set` in the shell. |
| `call ACTION [arg=value...] --yes` | Starts an action. `--yes` is required: without it nothing is sent and the exit code is 2. |
| `log --seconds S [--min-level DEBUG\|INFO\|WARN\|ERROR]` | Prints log events for `S` seconds of PC time (default level DEBUG). |

Common options: `--port`, `--baud`, `--presets`, and `--wait SECONDS`, which retries opening the link and reading the catalog until the robot answers or the time is up (for use after a reboot or a flash). `--json` on every one-shot command prints JSON on stdout instead of text. Each prints one document, except `log`, which prints one object per line:

```text
info     {"build": "...", "entries": 80, "port": "tcp:127.0.0.1:7650"}
list     [{"name": "speed_kp", "kind": "PARAM", "type": "F32", "unit": "rad/(m/s)", "live_step": 0.05}, ...]
get      {"roll_deg": -0.4, "ch5_position": 0}
set      {"name": "speed_kp", "old": 0.25, "new": 0.3}
call     {"name": "reboot", "args": {}, "accepted": true}
log      {"t_ms": 1020, "level": "INFO", "tag": "param", "text": "speed_kp: 0.25 -> 0.3"}
```

`list` shows `live_step` for parameters only and `args` for actions only; an infinite step is the string `"inf"`.

Exit codes:

| Code | Meaning |
| --- | --- |
| 0 | Done. |
| 1 | The robot refused the request; the reason is on stderr. |
| 2 | Usage error, an unknown name, or `call` without `--yes` (`call needs --yes`). |
| 3 | No connection: no reply, the port did not open, the link dropped, or the robot restarted while the request was pending (`RobotRestarted`, which may or may not have acted on it). |
| 130 | Interrupted with Ctrl-C. |

The build ID (`info`, the capture header, `hardware_id` on `/diagnostics`) names the firmware's git revision. `scripts/build_firmware.py` writes `build_id.h` before each build: `<8-character revision>[-dirty] <YYYY-MM-DD HH:MM>`. `-dirty` means a tracked file differs from the commit (`firmware-build-environment.json` does not count). Without a git revision the text is `nogit <time>`; a build without the script says `unknown <date>`. The ID is at most 32 characters.

## Capture

`capture` streams one subscription into a CSV file for `--seconds` of robot time: it stops at the first row whose `t_us` is that far from the first row. Pass `--preset <name>` or `--signals a,b,c`, and optionally `--rate <Hz>` (0 = every control tick). It fails when no event arrives for 2 s or three periods, whichever is longer.

The shape of the file, with made-up values and a robot signal list:

```text
# build: d9802212 2026-10-30 14:05
# rate: 100 Hz
# param speed_kp=0.25
# param tilt_p=8
t_us,ch5_position,roll_deg,motor1_target
1010000,1,-0.4,0.12
# log t_ms=1020 INFO [param] speed_kp: 0.25 -> 0.12
1020000,1,-0.5,0.11
1030000,1,-0.3,0.10
```

- The header comment holds the firmware build, the rate and the value of every parameter at the start.
- The column header is `t_us,<names>`. `t_us` is the robot's `micros()` at the start of the sampled tick; it wraps at 2^32.
- `f32` values have 7 significant digits; `bool` values are `0` or `1`.
- Each `Log` event becomes a comment line at the position it arrived, so a parameter change made over the other port shows in the file. A gap in the port's `seq` becomes `# lost <n> frames before seq <s>`.

## Presets

A preset is a named signal list with a rate in the preset file:

```yaml
bal:
  rate_hz: 100
  signals: [ch5_position, voltage_min_raw_v, roll_deg, motor1_target, motor2_target,
            leg_pose_min_deg, leg_pose_max_deg]
```

The presets in [scripts/navlink_presets.yaml](../../../scripts/navlink_presets.yaml) are `trace` (13 signals), `ctrl` (10), `bal` (7) and `drive` (28), each at 100 Hz, and `health` (20 signals) at 1 Hz. `health` holds exactly the `HEALTH_SIGNALS` of [navbot_link.health](../../../src/navbot_link/health.py), which the bridge publishes. The [trace capture page](../../robot/software/trace-capture.md#capture-with-navlink) describes what each one is for.

A row of a 100 Hz preset is due every 10 ms. The control loop runs at about 710 ticks per second, so each row is taken on the first tick at or after its deadline. `tests/scripts/test_navlink_presets.py` resolves every preset against [signals.def.h](../../../src/ES-02/OllieFOCdrive/signals.def.h). A renamed or removed signal fails that test.

## Analysis scripts

`scripts/summarize_balance_trace.py` and `scripts/analyze_drive_trace.py` read navlink CSV files only. They take time from `t_us`, not from the row count, and a file without the `t_us,` column header is refused. `analyze_drive_trace.py` needs the drive columns named in [trace capture](../../robot/software/trace-capture.md#analyse-a-drive-trial). `scripts/run_drive_trial.py` captures the `drive` preset and writes the report. `tests/scripts/test_navlink_traces.py` checks both tools against the fixtures in `tests/scripts/fixtures/`.

## Library

`navbot_link.link.Link(port, baud)` opens the port: `tcp:HOST:PORT` connects to the multiplexer, any other string is a serial port. It also takes any object with `read()`, `write(data)` and `close()`. It runs one I/O thread.

- `catalog()` reads the table once per connection and again after a robot restart. A `GetCatalog` without a reply is retried 3 times with a 0.5 s timeout.
- `subscribe(names, rate_hz)` returns a `Subscription` with the slot the robot assigned (1 to 8, shared by all ports). The library renews the 5 s lease every 2 s. `unsubscribe(slot)` keeps the slot and its rows until the robot's `Ack` confirms. Without a reply it raises `NoReply`, stops renewing, and keeps the slot reserved until the 5 s lease has run out. `slots` lists the slots the link holds.
- `get(name)`, `set(name, value)` and `call(name, **args)` return when the robot acknowledges. A refused request raises `LinkError` with the robot's reason. `call` is not retried, so a lost `Ack` never starts an action twice.
- `command(speed_mps, turn_rad_s)` sends a drive `Command` and returns at once, with no retry. The robot's refusal of a recent command (the last 64) comes back as `CommandRefused(reason)` on `events`; an accepted one is dropped silently.
- `events` is a queue in stream order: `Row`, `LogEvent`, `Text` (unframed text, such as ROM boot output), `CommandRefused`, `FramesLost`, `Restarted`, `SubscriptionEnded` and `Disconnected`. A `Link` opened with `telemetry=True` also puts `Telemetry(message)` for each `WheelOdometry` and `Imu` frame on the queue; the [ROS bridge](navbot-link-bridge.md) uses this, `navlink` does not. `restarts`, `lost_frames` and `bad_frames` count what the link saw.
- A port error or an unexpected error on the I/O thread ends the link with `Disconnected(reason)`; pending and later requests raise `LinkError(reason)`. A `Log` level the schema does not name shows as its number. A reply value that is not 4 bytes raises `LinkError`.
- A restart of the robot (its `seq` starts over) drops the catalog and ends every subscription, because table indices can change between builds. A request still waiting for its reply raises `LinkError("robot restarted")`, even when the reply that reveals the restart accepts it.

## Parameters and actions on the robot

A `set` applies at the start of the next control tick. The robot refuses a value that breaks a cross-field rule (for example `trim_envelope` above `tilt_ref_limit`) and keeps the running value; the Ack names the rule. While balance is requested (CH5 not at 0, or the controller running), a parameter with no live step is refused with `<name>: refused while balancing`. A live parameter may still change by at most its step per `set`; a larger change is refused with `<name>: step <d> over <step> while balancing`. The live parameters and their steps are in the [signal table](signal-table.md#parameters-settings-groups). The rule is checked in double precision with a relative slack of 1e-6, so a change of exactly one step is accepted despite float rounding. `get` and the streams always work.

The balance parameters are `speed_kp`, `speed_ki`, `speed_i_on`, `tilt_ref_limit`, `trim_envelope`, `tilt_p`, `gyro_damping`, `yaw_p`, `speed_cap` and `yaw_rate_cap`; the IMU parameters are `mahony_kp`, `mahony_ki` and `gyro_cutoff_hz`; the servo trims are `servo1_trim` to `servo4_trim`. `list` shows them with their units. The balance settings that copy motor constants are not parameters, by design ([signal table](signal-table.md#parameters-settings-groups)).

The actions are `motor_current`, `motor_voltage`, `motor_stop`, `motor_reset`, `calibrate`, `balance` and `reboot`; their bounds and safety rules are in [signal table](signal-table.md#actions-and-safety) and [motor commissioning](../../robot/software/motor-commissioning.md#actions).

- `balance on=1` (or `on=0`) sets the link's balance flag. It works only with CH5 in position 1, the link position. Refusals: `CH5 not in link position (1)`; `CH5 position change pending` when the switch moved on the last frame; with `on=1` only, `wheel overspeed latched, switch CH5 to 0`, `motor test running` and `calibration running`.
- `reboot` restarts the robot 300 ms after the Ack. It is refused with `CH5 must be 0 and balance off` while balance is requested, and with `motor test running` or `calibration running`. Balance or motor activity that starts during the delay cancels the reboot, which logs a warning. Both images reboot.
- The drive `Command` is not an action; it is described in the [protocol](ros2-uart-protocol.md#command).

## Tests

```bash
python3 -m pytest -q tests/navbot_link tests/scripts
```

`tests/navbot_link/test_firmware_endpoint.py` runs the library against the firmware's endpoint code (`link_endpoint.cpp` and its parts) built for the host with `c++`; it is skipped where no compiler is installed. `scripts/signals_catalog.py` prints the entries of `signals.def.h`; the tests use it as the expected catalog.

`tests/navbot_link/test_mux.py` runs the multiplexer against the fake firmware on a pseudo-terminal with two clients over TCP. The `balance`, `reboot`, `Command` and live-step rules are tested in the firmware's host tests (`tests/link/`, `tests/rc_mode/`) and against the fake firmware's `balancing` and `link_position` flags.

`tests/navbot_link/fake_firmware.py` implements the robot side of the catalog, subscription, parameter and action messages, with leases, seq gaps, restarts and USB text. Its clock advances one control tick per read while a subscription is active, so rows and `t_us` are reproducible. The tests run the library and `navlink` against it in memory and, for one capture, as a separate process on a pseudo-terminal. They check request bytes, CSV files and shell output against literal values.
