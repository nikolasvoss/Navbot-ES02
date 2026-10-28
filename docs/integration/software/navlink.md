# navlink: signals, parameters and actions over the link

Status: **works against the firmware's signal table; not yet run on the robot.** The `navbot_link` library and the `navlink` tool speak the catalog and subscription messages of the [protocol](ros2-uart-protocol.md#catalog-subscriptions-parameters-and-actions). The firmware answers them (migration step 3 of the [logging and control plan](../plans/logging-and-control-link.md#migration)) with 35 read-only signals, enough for the presets `trace`, `ctrl`, `bal` and `drive`. It has no parameters and no actions yet, so `set` and `call` are refused by the robot. The tool and the library were tested against the fake firmware in `tests/navbot_link/` and against the firmware's own endpoint code built for the host.

## Run

`navlink` is [scripts/navlink.py](../../../scripts/navlink.py). It needs `python3-serial`, `python3-protobuf` 3.21 and `python3-yaml`.

```bash
python3 scripts/navlink.py shell --port /dev/ttyAMA3
python3 scripts/navlink.py capture --port /dev/ttyAMA3 --preset speed_loop --seconds 30 -o speed-01.csv
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
navlink> sub @speed_loop -o speed-01.csv
slot 1: 6 entries @ 100 Hz, 2.8 kB/s -> speed-01.csv
navlink> set speed_kp 0.12
speed_kp: 0.08 -> 0.12
navlink> unsub 1
slot 1 closed
```

The load is the row size, `4 + 4 × entries` bytes, plus 24 bytes of frame cost, times the rate.

## Capture

`capture` streams one subscription into a CSV file for `--seconds` of robot time: it stops at the first row whose `t_us` is that far from the first row. Pass `--preset <name>` or `--signals a,b,c`, and optionally `--rate <Hz>` (0 = every control tick). It fails when no event arrives for 2 s or three periods, whichever is longer.

```text
# build: fake-build-1
# rate: 100 Hz
# param speed_kp=0.08
# param speed_ki=0.5
# param speed_i_on=1
t_us,speed_target,speed_measured,speed_p,speed_i,tilt_ref,tilt_clipped
1010000,0.5,0.125,-0.03125,0.001,0,1
# log t_ms=1020 INFO [param] speed_kp: 0.08 -> 0.12
1020000,0.5,0.25,-0.03125,0.002,0,0
1030000,0.5,0.375,-0.03125,0.003,0,1
```

- The header comment holds the firmware build, the rate and the value of every parameter at the start.
- The column header is `t_us,<names>`. `t_us` is the robot's `micros()` at the start of the sampled tick; it wraps at 2^32.
- `f32` values have 7 significant digits; `bool` values are `0` or `1`.
- Each `Log` event becomes a comment line at the position it arrived, so a parameter change made over the other port shows in the file. A gap in the port's `seq` becomes `# lost <n> frames before seq <s>`.

## Presets

A preset is a named signal list with a rate in the preset file:

```yaml
speed_loop:
  rate_hz: 100
  signals: [speed_target, speed_measured, speed_p, speed_i, tilt_ref, tilt_clipped]
```

The presets in [scripts/navlink_presets.yaml](../../../scripts/navlink_presets.yaml):

| Preset | Replaces | Signals | Rate |
| --- | --- | --- | --- |
| `trace` | `K55` TRACE | 13 | 100 Hz |
| `ctrl` | `K56` CTRL | 10 | 100 Hz |
| `bal` | `K57` BAL | 7 | 100 Hz |
| `drive` | `K58` DRIVE | 28 | 100 Hz |
| `speed_loop` | none | 6 | 100 Hz |

`speed_loop` names signals that only the test firmware has; the robot's table has no speed-loop signals yet. The four K presets use the column names of the K profiles ([trace capture](../../robot/software/trace-capture.md)), with these differences:

- `time_ms`, `sequence`, `logger_dropped` and `uart_write_failures` are gone. `t_us` replaces the time stamp, and the link's frame numbers (`# lost` comment lines) replace the counters.
- `servo1_range_deg` to `servo4_range_deg` and `max_servo_range_deg` become `leg_pose_min_deg` and `leg_pose_max_deg`, the lowest and highest leg pose over the ticks since the previous row. The four servos take the same angle, so the range is the maximum minus the minimum. The values are the unrounded pose, and the K range is taken from the rounded servo commands.
- `roll_ok` is `roll_deg`, and `max_loop_entry_interval_us` is `loop_interval_max_us` (the catalog allows 24 characters). `battery_filtered_instant_v` is left out because it has the value of `voltage_filtered_v`.
- A K row is printed on every 7th control tick. A preset row is due every 10 ms, so both give about 100 rows per second at 710 ticks per second.
- `voltage_min_raw_v`, `loop_interval_max_us` and the leg pose limits are running minimum and maximum values: they cover every tick since the previous row, as the K columns do.

`tests/scripts/test_navlink_presets.py` compares each K preset with the column list of its profile in `scripts/capture_balance_trace.py`, and resolves every preset against `src/ES-02/OllieFOCdrive/signals.def.h`. A renamed or removed signal fails that test.

To compare a preset with a K profile on the robot, capture both during the same run (UART1 for the preset, USB for the K profile) and compare the columns by name; the rows differ in time stamps.

## Analysis scripts

`scripts/summarize_balance_trace.py` and `scripts/analyze_drive_trace.py` read these CSV files and the old `K55` to `K58` logs. They take time from `t_us`, not from the row count. A navlink file for `analyze_drive_trace.py` needs the DRIVE columns named in [trace capture](../../robot/software/trace-capture.md#capture-with-navlink); `speed_loop` has none of them, so it can be summarized but not analyzed as a drive trial. `tests/scripts/test_navlink_traces.py` checks both tools against the fixtures in `tests/scripts/fixtures/`.

## Library

`navbot_link.link.Link(port, baud)` opens the port; it also takes any object with `read()`, `write(data)` and `close()`. It runs one I/O thread.

- `catalog()` reads the table once per connection and again after a robot restart. A `GetCatalog` without a reply is retried 3 times with a 0.5 s timeout.
- `subscribe(names, rate_hz)` returns a `Subscription` with the slot the robot assigned (1 to 4, shared by all ports). The library renews the 5 s lease every 2 s. `unsubscribe(slot)` keeps the slot and its rows until the robot's `Ack` confirms. Without a reply it raises `NoReply`, stops renewing, and keeps the slot reserved until the 5 s lease has run out. `slots` lists the slots the link holds.
- `get(name)`, `set(name, value)` and `call(name, **args)` return when the robot acknowledges. A refused request raises `LinkError` with the robot's reason. `call` is not retried, so a lost `Ack` never starts an action twice.
- `events` is a queue in stream order: `Row`, `LogEvent`, `Text` (raw text from the USB port), `FramesLost`, `Restarted`, `SubscriptionEnded` and `Disconnected`. A `Link` opened with `telemetry=True` also puts `Telemetry(message)` for each `WheelOdometry` and `Imu` frame on the queue; the [ROS bridge](navbot-link-bridge.md) uses this, `navlink` does not. `restarts`, `lost_frames` and `bad_frames` count what the link saw.
- A port error or an unexpected error on the I/O thread ends the link with `Disconnected(reason)`; pending and later requests raise `LinkError(reason)`. A `Log` level the schema does not name shows as its number. A reply value that is not 4 bytes raises `LinkError`.
- A restart of the robot (its `seq` starts over) drops the catalog and ends every subscription, because table indices can change between builds. A request still waiting for its reply raises `LinkError("robot restarted")`, even when the reply that reveals the restart accepts it.

## Tests

```bash
python3 -m pytest -q tests/navbot_link tests/scripts
```

`tests/navbot_link/test_firmware_endpoint.py` runs the library against the firmware's endpoint code (`link_endpoint.cpp` and its parts) built for the host with `c++`; it is skipped where no compiler is installed. `scripts/signals_catalog.py` prints the entries of `signals.def.h`; the tests use it as the expected catalog.

`tests/navbot_link/fake_firmware.py` implements the robot side of the catalog, subscription, parameter and action messages, with leases, seq gaps, restarts and USB text. Its clock advances one control tick per read while a subscription is active, so rows and `t_us` are reproducible. The tests run the library and `navlink` against it in memory and, for one capture, as a separate process on a pseudo-terminal. They check request bytes, CSV files and shell output against literal values. The example file above is the start of the preset capture test output.
