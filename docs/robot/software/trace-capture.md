# Trace capture and live tuning

This page describes how to capture signals from the running robot and how to change its parameters live. Everything goes through [navlink](../../integration/software/navlink.md) over the [NavBot link](../../integration/software/ros2-uart-protocol.md), on the ros2_pc UART or on the robot's USB port.

Status: implemented and host-tested; not yet run on the robot.

## Capture with navlink

`navlink capture` records any list of firmware signals at a rate you choose. Stop the ros2_pc bridge first, because only one program can read the UART.

```bash
python3 scripts/navlink.py capture --port /dev/ttyAMA3 --preset bal --seconds 20 -o balance-01.csv
python3 scripts/summarize_balance_trace.py balance-01.csv
```

- `--port /dev/ttyAMA3` is the ros2_pc UART; `--port /dev/ttyUSB0` is the robot's USB port (576000 baud).
- `--preset <name>` takes a signal list from [navlink_presets.yaml](../../../scripts/navlink_presets.yaml); `--signals a,b,c` names them directly. `--rate <Hz>` overrides the preset rate, and 0 streams every control tick.
- `--seconds` is robot time: the file holds that much data, whatever the PC time is.
- The file starts with `#` comments (firmware build, rate and every parameter value), then a column header `t_us,<signal names>`. A `# log` line marks each parameter change or action that the robot reports, on either port. A `# lost <n> frames before seq <s>` line marks a gap in the link.
- Rows are not evenly spaced. `t_us` is the robot's `micros()` at the sampled tick and wraps at 2^32 (about 71.6 min). The tools take time from `t_us`, not from the row count.

The presets are:

| Preset | Signals | Use |
| --- | --- | --- |
| `trace` | 13 | Tilt, servo commands, leg pose window and motor targets at 100 Hz |
| `ctrl` | 10 | Tilt, yaw rate, body turn command and motor targets at 100 Hz |
| `bal` | 7 | Tilt and motor targets at 100 Hz, for a balance shutdown |
| `drive` | 28 | Forward and reverse runs, IMU timing, loop timing and motor state at 100 Hz |

A preset row is due every 10 ms. The control loop runs at about 710 ticks per second, so each row is taken on the first tick at or after its deadline.

### Signals that change between rows

Some signals are running extremes rather than the value at the sampled tick. They cover every control tick since the previous row of the same subscription:

- `voltage_min_raw_v`: the lowest raw battery reading.
- `leg_pose_min_deg` and `leg_pose_max_deg`: the lowest and highest leg pose. The four servos take the same angle, so their window is the servo range of the row. The range is the maximum minus the minimum.
- `loop_interval_max_us`: the longest time between two loop passes.

`imu_long_gap_count` and `imu_max_interval_us` come from the IMU timing counters, and `imu_sequence` counts IMU samples, so a row skips the samples taken between two rows.

## Analyse a drive trial

`scripts/run_drive_trial.py` captures one drive trial with the `drive` preset, then writes a short Markdown report next to the CSV. It refuses to overwrite an existing file.

```bash
python3 scripts/run_drive_trial.py --port /dev/ttyAMA3 --seconds 25
python3 scripts/analyze_drive_trace.py drive-01.csv drive-02.csv --require-stop --output drive-comparison.md
```

Before the capture starts, set the robot up as the script's message says: channels 3 and 4 neutral, the balance switch off, and the robot supported. Turn the balance switch on after the capture starts, give one forward or reverse command, and return it to neutral.

`scripts/analyze_drive_trace.py` reads a navlink file only and needs these columns from the `drive` preset: `ch5_mode`, `ch3_target`, `motor1_velocity_f`, `motor2_velocity_f`, `roll_deg`, `motor1_target`, `motor2_target`, `voltage_min_raw_v`, `imu_sequence`, `imu_read_time_us`, `imu_dt_s`, `imu_accepted_samples`, `imu_long_gap_count`, `imu_max_interval_us`, `control_execution_us`, `loop_interval_max_us`, `motor1_current_sp`, `motor2_current_sp`, `motor1_voltage_q`, `motor2_voltage_q` and `battery_raw_instant_v`. A file without them fails with `fehlende DRIVE-Spalten: ...`.

`scripts/summarize_balance_trace.py` reports every signal column of a file with its minimum, maximum, mean, first and last value, plus the build, rate, start parameters, lost frames and log events.

## Live parameter changes

`navlink shell` changes parameters on the running robot. `list` shows every parameter with its unit, `get` reads it and `set` writes it:

```text
navlink> set speed_kp 0.12
speed_kp: 0.08 -> 0.12
```

A change is not stored in flash. After a reboot the robot starts with its built-in values again, so no connection is needed to run it after flashing. The robot refuses a `set` while balance is requested (CH5 on, or the controller running).

## Capture a balance shutdown

With `SENSOR_DIAGNOSTIC_MODE` set to `0`, upload the rebuilt sketch and support the robot so it cannot fall. Close every other serial monitor. Then run the `bal` capture above and stop it when the robot is safe. Read the summary before the next trial.

## Sensor diagnostics

The sensor diagnostic mode does not stream signals; see the [sensor diagnostic guide](sensor-diagnostic-mode.md). The raw S.BUS channels are not in the signal table, so they are not captured by navlink. `remote_status.py` reports the receiver-derived signals `ch5_mode`, `ch3_target` and `body_turn_command` instead.
