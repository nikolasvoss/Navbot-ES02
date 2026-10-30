# navlink: signals, parameters and actions over the link

Status: **implemented, host-tested against the firmware's entry table, and run on the robot** for the catalog, parameters, subscriptions and the diagnostic image. The motor actions, `calibrate` and the refusal while balancing are not yet run on the robot. A one-page guide is the [quick start](navlink-quick-start.md). The `navbot_link` library and the `navlink` tool speak the catalog, subscription, parameter and action messages of the [protocol](ros2-uart-protocol.md#catalog-subscriptions-parameters-and-actions). The firmware serves the table described in the [signal table](signal-table.md). The tool and the library were tested against the fake firmware in `tests/navbot_link/` and against the firmware's own endpoint code built for the host.

## Run

`navlink` is [scripts/navlink.py](../../../scripts/navlink.py). It needs `python3-serial`, `python3-protobuf` 3.21 and `python3-yaml`.

```bash
python3 scripts/navlink.py shell --port /dev/ttyAMA3
python3 scripts/navlink.py capture --port /dev/ttyAMA3 --preset drive --seconds 30 -o drive-01.csv
```

- `/dev/ttyAMA3` is the ros2_pc UART. Stop the bridge service first; only one program can read the port.
- `/dev/ttyUSB0` is the robot's USB port. A port name with `USB` or `ACM` defaults to 576000 baud, any other to 2000000. `--baud` overrides it. Opening the CH340 can reset the board ([USB/serial facts](../../robot/software/usb-serial-flashing.md)); `navlink` keeps DTR and RTS off, which does not prevent it on every adapter.
- `--presets` selects the preset file; the default is [scripts/navlink_presets.yaml](../../../scripts/navlink_presets.yaml).

On connect, `navlink` reads the catalog entry by entry and resolves every name against it. A name the firmware does not have fails with `unknown signal <name>`.

## Shell

| Command | Effect |
| --- | --- |
| `list [pattern]` | Entries whose name matches the glob pattern, with kind, type and unit; for an action, its arguments and bounds. |
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

## Capture

`capture` streams one subscription into a CSV file for `--seconds` of robot time: it stops at the first row whose `t_us` is that far from the first row. Pass `--preset <name>` or `--signals a,b,c`, and optionally `--rate <Hz>` (0 = every control tick). It fails when no event arrives for 2 s or three periods, whichever is longer.

The shape of the file, with made-up values and a robot signal list:

```text
# build: Oct 28 2026 10:12:03
# rate: 100 Hz
# param speed_kp=0.25
# param tilt_p=8
t_us,ch5_mode,roll_deg,motor1_target
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
  signals: [ch5_mode, voltage_min_raw_v, roll_deg, motor1_target, motor2_target,
            leg_pose_min_deg, leg_pose_max_deg]
```

The presets in [scripts/navlink_presets.yaml](../../../scripts/navlink_presets.yaml) are `trace` (13 signals), `ctrl` (10), `bal` (7) and `drive` (28), each at 100 Hz. The [trace capture page](../../robot/software/trace-capture.md#capture-with-navlink) describes what each one is for.

A preset row is due every 10 ms. The control loop runs at about 710 ticks per second, so each row is taken on the first tick at or after its deadline. `tests/scripts/test_navlink_presets.py` resolves every preset against [signals.def.h](../../../src/ES-02/OllieFOCdrive/signals.def.h). A renamed or removed signal fails that test.

## Analysis scripts

`scripts/summarize_balance_trace.py` and `scripts/analyze_drive_trace.py` read navlink CSV files only. They take time from `t_us`, not from the row count, and a file without the `t_us,` column header is refused. `analyze_drive_trace.py` needs the drive columns named in [trace capture](../../robot/software/trace-capture.md#analyse-a-drive-trial). `scripts/run_drive_trial.py` captures the `drive` preset and writes the report. `tests/scripts/test_navlink_traces.py` checks both tools against the fixtures in `tests/scripts/fixtures/`.

## Library

`navbot_link.link.Link(port, baud)` opens the port; it also takes any object with `read()`, `write(data)` and `close()`. It runs one I/O thread.

- `catalog()` reads the table once per connection and again after a robot restart. A `GetCatalog` without a reply is retried 3 times with a 0.5 s timeout.
- `subscribe(names, rate_hz)` returns a `Subscription` with the slot the robot assigned (1 to 4, shared by all ports). The library renews the 5 s lease every 2 s. `unsubscribe(slot)` keeps the slot and its rows until the robot's `Ack` confirms. Without a reply it raises `NoReply`, stops renewing, and keeps the slot reserved until the 5 s lease has run out. `slots` lists the slots the link holds.
- `get(name)`, `set(name, value)` and `call(name, **args)` return when the robot acknowledges. A refused request raises `LinkError` with the robot's reason. `call` is not retried, so a lost `Ack` never starts an action twice.
- `events` is a queue in stream order: `Row`, `LogEvent`, `Text` (unframed text, such as ROM boot output), `FramesLost`, `Restarted`, `SubscriptionEnded` and `Disconnected`. A `Link` opened with `telemetry=True` also puts `Telemetry(message)` for each `WheelOdometry` and `Imu` frame on the queue; the [ROS bridge](navbot-link-bridge.md) uses this, `navlink` does not. `restarts`, `lost_frames` and `bad_frames` count what the link saw.
- A port error or an unexpected error on the I/O thread ends the link with `Disconnected(reason)`; pending and later requests raise `LinkError(reason)`. A `Log` level the schema does not name shows as its number. A reply value that is not 4 bytes raises `LinkError`.
- A restart of the robot (its `seq` starts over) drops the catalog and ends every subscription, because table indices can change between builds. A request still waiting for its reply raises `LinkError("robot restarted")`, even when the reply that reveals the restart accepts it.

## Parameters and actions on the robot

A `set` applies at the start of the next control tick. The robot refuses a value that breaks a cross-field rule (for example `trim_envelope` above `tilt_ref_limit`) and keeps the running value; the Ack names the rule. While balance is requested (CH5 on, or the controller running) the robot refuses every `set` with `<name>: refused while balancing`. `get` and the streams still work, so switch balance off before changing a value.

The balance parameters are `speed_kp`, `speed_ki`, `speed_i_on`, `tilt_ref_limit`, `trim_envelope`, `tilt_p`, `gyro_damping`, `yaw_p`, `speed_cap` and `yaw_rate_cap`; the IMU parameters are `mahony_kp`, `mahony_ki` and `gyro_cutoff_hz`; the servo trims are `servo1_trim` to `servo4_trim`. `list` shows them with their units. The balance settings that copy motor constants are not parameters, by design ([signal table](signal-table.md#parameters-settings-groups)).

The actions are `motor_current`, `motor_voltage`, `motor_stop`, `motor_reset` and `calibrate`; their bounds and safety rules are in [signal table](signal-table.md#actions-and-safety) and [motor commissioning](../../robot/software/motor-commissioning.md#actions).

## Tests

```bash
python3 -m pytest -q tests/navbot_link tests/scripts
```

`tests/navbot_link/test_firmware_endpoint.py` runs the library against the firmware's endpoint code (`link_endpoint.cpp` and its parts) built for the host with `c++`; it is skipped where no compiler is installed. `scripts/signals_catalog.py` prints the entries of `signals.def.h`; the tests use it as the expected catalog.

`tests/navbot_link/fake_firmware.py` implements the robot side of the catalog, subscription, parameter and action messages, with leases, seq gaps, restarts and USB text. Its clock advances one control tick per read while a subscription is active, so rows and `t_us` are reproducible. The tests run the library and `navlink` against it in memory and, for one capture, as a separate process on a pseudo-terminal. They check request bytes, CSV files and shell output against literal values.
