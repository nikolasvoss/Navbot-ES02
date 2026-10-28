# Trace capture and live tuning

This page describes how to capture serial traces from the running robot, which trace modes exist, and which gains live tuning can change.

## Capture with navlink

`navlink capture` records any list of firmware signals, with a rate you choose, over the [NavBot link](../../integration/software/ros2-uart-protocol.md). It replaces the `K55` to `K58` profiles and needs no firmware rebuild per variable. Full usage is in the [navlink page](../../integration/software/navlink.md).

**Not run on the robot yet.** The firmware serves the signal table (migration steps [3](../../integration/plans/logging-and-control-link.md#migration) and [4](../../integration/plans/logging-and-control-link.md#migration), host-tested), but `navlink capture` has not been run against the robot.

```bash
python3 scripts/navlink.py capture --port /dev/ttyAMA3 --preset speed_loop --seconds 30 -o speed-01.csv
python3 scripts/summarize_balance_trace.py speed-01.csv
python3 scripts/analyze_drive_trace.py drive-01.csv --require-stop
```

- The file starts with `#` comments (firmware build, rate, every parameter value) and a column header `t_us,<signal names>`. `# log` lines mark parameter changes and other robot events, and `# lost <n> frames before seq <s>` marks a gap in the link.
- Rows are not evenly spaced. `t_us` is the robot's `micros()` at the sampled tick and wraps at 2^32 (about 71.6 min). The analysis scripts convert it to `time_ms` and measure spans, rates, the median interval and the stop window from it, not from the row count. Row counts remain counts: valid and malformed rows, the 20-row minimum for a usable capture, active and CH3 rows, and the neutral samples of a stop. The scripts handle a wrap by continuing the timeline. A small `t_us` after a jump back is a robot restart, which the summary reports as a warning, and its downtime is missing from the span. An isolated out-of-range value, or one outside 0 to 2^32-1, is a malformed row. A second `t_us,` header line stops parsing with a warning, and a column named `time_ms` is an error. A signal that changes every control step can alias against a sampling period that is not a multiple of that step.
- `scripts/summarize_balance_trace.py` reports every signal column of a navlink file with min, max, mean, first and last value, plus build, rate, start parameters, lost frames and log events.
- `scripts/analyze_drive_trace.py` reads a navlink file when its columns are the DRIVE names below. A missing column fails with `fehlende DRIVE-Spalten: ...`.

The analysis scripts keep the K58 header names as column names, so a preset that reproduces `DRIVE` has to use them. `analyze_drive_trace.py` needs these columns:

`ch5_mode`, `ch3_target`, `motor1_velocity_f`, `motor2_velocity_f`, `roll_ok`, `motor1_target`, `motor2_target`, `voltage_min_raw_v`, `imu_sequence`, `imu_read_time_us`, `imu_dt_s`, `imu_accepted_samples`, `imu_long_gap_count`, `imu_max_interval_us`, `control_execution_us`, `max_loop_entry_interval_us`, `motor1_current_sp`, `motor2_current_sp`, `motor1_voltage_q`, `motor2_voltage_q`, `battery_raw_instant_v`.

| K58 field | In a navlink file |
| --- | --- |
| `time_ms` | `t_us` (microseconds), converted to `time_ms` by the tools |
| `sequence`, `logger_dropped`, `uart_write_failures` | Gone. `# lost` comment lines report link gaps; the TX queue counters are not signals yet. |
| `control_start_us`, `tick_dt_s`, `voltage_filtered_v`, `battery_filtered_instant_v`, `max_servo_range_deg`, `motor1_shaft_velocity`, `motor2_shaft_velocity` | Optional. `analyze_drive_trace.py` does not need them, and `summarize_balance_trace.py` reports them if present. |

## Legacy: K profiles until firmware migration step 5

The rest of this page describes the `K` command path. It stays until the firmware drops the `K` commands, the CSV profiles and `LogFormat.cpp` (step 5 of the [logging plan](../../integration/plans/logging-and-control-link.md#migration)). `scripts/capture_balance_trace.py` and `scripts/run_drive_trial.py` drive it, and `capture_balance_trace.py --help` points to `navlink capture`. The analysis scripts read both formats, so logs from this path stay readable after the removal.

## Safe sensor diagnostics

The full, current procedure is in the [sensor diagnostic guide](sensor-diagnostic-mode.md), including safe motor outputs, build and flash steps, SerialPlot settings, the 17-column channel map, and reset troubleshooting. The source currently sets `SENSOR_DIAGNOSTIC_MODE=0`, which builds normal motor-control firmware. Set it to `1` and rebuild for sensor diagnostics. Diagnostic mode skips motor and servo initialization and holds the wheel-driver enable pins low. Support the robot mechanically because the legs are not driven. The diagnostic stream currently uses 576000 baud through the CH340 UART at the user's direction. The hardware UART rate at 576000 has not yet been measured, and this setting is not hardware-accepted.

With the normal image, SerialPlot can log one raw SBUS receiver channel while the robot runs. Use `K60` for CH1 through `K75` for CH16. Each row contains firmware uptime in seconds and the selected channel value. Send `K0` to stop logging. The [sensor diagnostic guide](sensor-diagnostic-mode.md) has the SerialPlot setup steps.

## Capture a balance shutdown

With `SENSOR_DIAGNOSTIC_MODE` set to `0`, upload the rebuilt sketch and support the robot so it cannot fall. Close every other serial monitor, then run:

```bash
python3 scripts/capture_balance_trace.py --seconds 20 --output balance-trace.log
```

The script waits for startup, sends `K55`, and records a `TRACE` stream, one row for every seventh executed control gate. That is 142.9 Hz only if the gate runs at the nominal 1 kHz. The 2026-10-22 DRIVE capture measured 424 loop passes per second, which gives about 60.6 rows per second at the same one-in-seven decimation ([voltage sweep, 2 V](../../../lab/robot/captures/2026-10-22-package1-t7-voltage-sweep-2v.log)). Take the rate from each capture. Switch RC channel 5 from off to on **once** during the capture. The columns are `time_ms,ch5_mode,voltage_min_raw_v,voltage_filtered_v,roll_deg,pitch_deg,servo1_cmd_deg,servo2_cmd_deg,servo3_cmd_deg,servo4_cmd_deg,servo1_range_deg,servo2_range_deg,servo3_range_deg,servo4_range_deg,motor1_target,motor2_target,sequence,logger_dropped,uart_write_failures`. Each `range` is the highest minus lowest integer servo command since the preceding row, nominally 7 ms at the 1 kHz gate and about 16.5 ms at the 424 passes per second measured in the 2026-10-22 capture. The minimum voltage uses all battery ADC samples since the previous row; it is still a board ADC estimate, not a measurement of the ESP32's 3.3 V rail. The servo values are commanded angles, not measured shaft positions. The saved log also includes any reboot or brownout text that reaches the serial port. The script sends `K0` at the end to stop tracing. If the robot loses power, retain the log even if the script reports a port error.

After uploading firmware built from the current source, add `--trace-mode 56` and choose a new output file to capture the compact `CTRL` stream instead. It records `time_ms,ch5_mode,voltage_min_raw_v,voltage_filtered_v,roll_deg,gyro_z_rad_s,body_turn_command,motor1_target,motor2_target,max_servo_range_deg,sequence,logger_dropped,uart_write_failures`. Selected trace modes suppress repeated low-voltage debug text to keep it out of CSV rows. `body_turn_command` uses the firmware's numerical scale; do not assume it is physical rad/s without checking the RC mapping. Existing firmware images that only implement `K55` must be reflashed for this mode.

For the balance rows, `--trace-mode 57` records `BAL` rows: `time_ms,ch5_mode,voltage_min_raw_v,roll_deg,motor1_target,motor2_target,max_servo_range_deg,sequence,logger_dropped,uart_write_failures`. Its shorter lines use the same asynchronous sender and 576000 baud stream. Give each run a unique output filename.

For forward/reverse motion and stopping behavior, `--trace-mode 58` records `DRIVE` rows: `time_ms,ch5_mode,voltage_min_raw_v,voltage_filtered_v,imu_sequence,imu_read_time_us,imu_dt_s,imu_accepted_samples,imu_long_gap_count,imu_max_interval_us,control_start_us,control_execution_us,max_loop_entry_interval_us,ch3_target,motor1_velocity_f,motor2_velocity_f,tick_dt_s,roll_ok,motor1_target,motor2_target,motor1_current_sp,motor2_current_sp,motor1_voltage_q,motor2_voltage_q,motor1_shaft_velocity,motor2_shaft_velocity,battery_raw_instant_v,battery_filtered_instant_v,max_servo_range_deg,sequence,logger_dropped,uart_write_failures`. The IMU fields are the sample sequence, read time in µs, the sample interval `dt` in seconds, and the cumulative accepted-sample, long-gap, and maximum-interval counters since boot. `scripts/analyze_drive_trace.py` takes the mean accepted-read interval from the `imu_read_time_us` and `imu_sequence` differences (`imu_mean_interval_s`) and reports the long-gap count with its fraction of accepted samples; the per-row `imu_dt_s` is not used for the mean. `control_start_us` and `control_execution_us` time the control tick, and `max_loop_entry_interval_us` is the largest `loop()` period since the previous row. The motor fields are per wheel (`motor1` is left, `motor2` is right): the commanded target, the current setpoint, the q-axis voltage, and the shaft velocity. The battery fields are the instantaneous raw and filtered bus reading. `motor1_target` is the wheel target in amperes, which stays 0 A until package 4; during `motor_current` and `motor_voltage` tests it is the commanded A or V, with the motor-local sign. [Motor commissioning](motor-commissioning.md) describes those actions. The analysis scripts read the header and reject a log with a different column set; no older DRIVE layout is read. Use this only after confirming the trace output with CH5 off; each run should have a new filename. The logger does not change controller gains or the control-gate cadence, but its row rate follows the measured gate cadence, not a fixed 142.9 Hz. At the measured 60.6 DRIVE rows per second the row Nyquist is about 30 Hz (71.4 Hz at the nominal 142.9 Hz); signals above that can alias. The configured 50 Hz IMU low-pass cutoff is based on nominal 1 kHz sampling rather than a measured runtime cadence; the 2026-10-22 capture measured about 424 Hz. No active-balance hardware measurement has established control-cycle timing or UART integrity under transmission load.

The wheel speed estimate uses the measured control tick. Live changes made with `navlink set` are not stored in flash and reset to the startup values after reboot, so no connection is needed to run the robot after flashing.

To log telemetry through one serial connection, run `python3 scripts/capture_balance_trace.py --trace-mode 56 --interactive --seconds 40 --output control-live.log`. The script logs every trace row and each sent command (`# CMD ...`), while showing only every tenth trace row in the terminal. The interactive input accepts only trace-selection commands (`K0`, `K55` to `K58`). Motor tests are `navlink` actions ([motor commissioning](motor-commissioning.md#actions)) and are not sent from this script. After the capture, the script sends `K0` and logs the firmware replies for two more seconds, so result text such as `MOTOR_ID_RESULT` reaches the log.

For a repeatable agent-facing run summary, pass one or more saved logs to `scripts/summarize_balance_trace.py`. It reports valid and malformed rows, capture span, CH5 mode counts/transitions, and a small set of min/max/mean values without making causal claims:

```bash
python3 scripts/summarize_balance_trace.py balance-trace.log
python3 scripts/summarize_balance_trace.py run-1.log run-2.log --output balance-summary.md
```

For repeated K58 drive/stop trials, one command captures a new log at 576000 baud and writes a short `*.md` report after capture. It refuses to overwrite an existing log or report:

```bash
python3 scripts/run_drive_trial.py --interactive --output drive-test-01.log --seconds 40
```

The report checks trace quality, CH5 activation, CH3 movement, start tilt, and the largest opposite wheel-speed reading in the first second after a CH3 release. If there is no usable stop event, the command returns status 2 and keeps both files. It does not switch the robot. Set live gains with `navlink shell` (`set <param> <value>`, then `get <param>` to read it back) before enabling CH5; this script's `--interactive` accepts trace selections only. Use a new filename for each gain setting and state the observed movement with the report path. Opening a new serial session may restart the board, so set and read back live gains again in each run.

Compare saved trials without opening Serial or recompiling firmware:

```bash
python3 scripts/analyze_drive_trace.py drive-test-01.log drive-test-02.log --require-stop --output drive-comparison.md
```

`--format json` provides the same measurements for other scripts. A trial with neutral CH3 or a strongly tilted start is flagged as unsuitable for a stop comparison; it is not interpreted as a controller improvement or regression.

## The same columns over the NavBot link

The firmware also serves the `K55` to `K58` values as signals of the [NavBot link](../../integration/software/ros2-uart-protocol.md). The presets `trace`, `ctrl`, `bal` and `drive` of [navlink](../../integration/software/navlink.md#presets) stream them as binary rows from the ros2_pc UART or the USB port and write a CSV with the column names above. The `K` commands and profiles work as before, also while a preset streams. A few columns differ (no `time_ms` or logger counters, the servo range as a minimum and maximum); navlink lists them. `Log` frames go to UART1 only, so a USB capture of a `K` profile holds CSV rows and text and nothing binary.
