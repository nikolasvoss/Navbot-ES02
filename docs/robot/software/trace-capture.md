# Trace capture and live tuning

This page describes how to capture serial traces from the running robot, which trace modes exist, and which gains live tuning can change.

## Safe sensor diagnostics

The full, current procedure is in the [sensor diagnostic guide](sensor-diagnostic-mode.md), including safe motor outputs, build and flash steps, SerialPlot settings, the 17-column channel map, and reset troubleshooting. The source currently sets `SENSOR_DIAGNOSTIC_MODE=0`, which builds normal motor-control firmware. Set it to `1` and rebuild for sensor diagnostics. Diagnostic mode skips motor and servo initialization and holds the wheel-driver enable pins low. Support the robot mechanically because the legs are not driven. The diagnostic stream currently uses 576000 baud through the CH340 UART at the user's direction. The hardware UART rate at 576000 has not yet been measured, and this setting is not hardware-accepted.

With the normal image, SerialPlot can log one raw SBUS receiver channel while the robot runs. Use `K60` for CH1 through `K75` for CH16. Each row contains firmware uptime in seconds and the selected channel value. Send `K0` to stop logging. The [sensor diagnostic guide](sensor-diagnostic-mode.md) has the SerialPlot setup steps.

## Capture a balance shutdown

With `SENSOR_DIAGNOSTIC_MODE` set to `0`, upload the rebuilt sketch and support the robot so it cannot fall. Close every other serial monitor, then run:

```bash
python3 scripts/capture_balance_trace.py --seconds 20 --output balance-trace.log
```

The script waits for startup, sends `K55`, and records a `TRACE` stream, one row for every seventh executed control gate. That is 142.9 Hz only if the gate runs at the nominal 1 kHz. The 2026-10-22 DRIVE capture measured 424 loop passes per second, which gives about 60.6 rows per second at the same one-in-seven decimation ([voltage sweep, 2 V](../../../lab/robot/captures/2026-10-22-package1-t7-voltage-sweep-2v.log)). Take the rate from each capture. Switch RC channel 5 from off to on **once** during the capture. The columns are `time_ms,ch5_mode,voltage_min_raw_v,voltage_filtered_v,roll_deg,pitch_deg,servo1_cmd_deg,servo2_cmd_deg,servo3_cmd_deg,servo4_cmd_deg,servo1_range_deg,servo2_range_deg,servo3_range_deg,servo4_range_deg,motor1_target,motor2_target,sequence,logger_dropped,uart_write_failures`. Each `range` is the highest minus lowest integer servo command since the preceding row, nominally 7 ms at the 1 kHz gate and about 16.5 ms at the 424 passes per second measured in the 2026-10-22 capture. The minimum voltage uses all battery ADC samples since the previous row; it is still a board ADC estimate, not a measurement of the ESP32's 3.3 V rail. The servo values are commanded angles, not measured shaft positions. The saved log also includes any reboot or brownout text that reaches the serial port. The script sends `K0` at the end to stop tracing. If the robot loses power, retain the log even if the script reports a port error.

After uploading firmware built from the current source, add `--trace-mode 56` and choose a new output file to capture the compact `CTRL` stream instead. It records `time_ms,ch5_mode,voltage_min_raw_v,voltage_filtered_v,roll_deg,gyro_z_rad_s,body_turn_command,yaw_error,yaw_output,motor1_target,motor2_target,max_servo_range_deg,sequence,logger_dropped,uart_write_failures`. The angle and yaw outputs are the two terms combined into the wheel targets; an inactive balance mode reports zero for their error/output columns. Selected trace modes suppress repeated low-voltage debug text to keep it out of CSV rows. `body_turn_command` and `yaw_error` use the firmware's numerical scale; do not assume they are physical rad/s without checking the RC mapping. Existing firmware images that only implement `K55` must be reflashed for this mode.

For the balance rows, `--trace-mode 57` records `BAL` rows: `time_ms,ch5_mode,voltage_min_raw_v,roll_deg,motor1_target,motor2_target,max_servo_range_deg,sequence,logger_dropped,uart_write_failures`. Its shorter lines use the same asynchronous sender and 576000 baud stream. Give each run a unique output filename.

For forward/reverse motion and stopping behavior, `--trace-mode 58` records `DRIVE` rows: `time_ms,ch5_mode,voltage_min_raw_v,voltage_filtered_v,imu_sequence,imu_read_time_us,imu_dt_s,imu_accepted_samples,imu_long_gap_count,imu_max_interval_us,control_start_us,control_execution_us,max_loop_entry_interval_us,ch3_target,motor1_velocity_f,motor2_velocity_f,tick_dt_s,roll_ok,motor1_target,motor2_target,motor1_current_sp,motor2_current_sp,motor1_voltage_q,motor2_voltage_q,motor1_shaft_velocity,motor2_shaft_velocity,battery_raw_instant_v,battery_filtered_instant_v,max_servo_range_deg,sequence,logger_dropped,uart_write_failures`. The IMU fields are the sample sequence, read time in µs, the sample interval `dt` in seconds, and the cumulative accepted-sample, long-gap, and maximum-interval counters since boot. `scripts/analyze_drive_trace.py` takes the mean accepted-read interval from the `imu_read_time_us` and `imu_sequence` differences (`imu_mean_interval_s`) and reports the long-gap count with its fraction of accepted samples; the per-row `imu_dt_s` is not used for the mean. `control_start_us` and `control_execution_us` time the control tick, and `max_loop_entry_interval_us` is the largest `loop()` period since the previous row. The motor fields are per wheel (`motor1` is left, `motor2` is right): the commanded target, the current setpoint, the q-axis voltage, and the shaft velocity. The battery fields are the instantaneous raw and filtered bus reading. `motor1_target` is the wheel target in amperes, which stays 0 A until package 4; during `T` and `M` diagnostics it is the commanded A or V, with the motor-local sign. [Motor commissioning](motor-commissioning.md) describes those commands. The analysis scripts read the header and reject a log with a different column set; no older DRIVE layout is read. Use this only after confirming the trace output with CH5 off; each run should have a new filename. The logger does not change controller gains or the control-gate cadence, but its row rate follows the measured gate cadence, not a fixed 142.9 Hz. At the measured 60.6 DRIVE rows per second the row Nyquist is about 30 Hz (71.4 Hz at the nominal 142.9 Hz); signals above that can alias. The configured 50 Hz IMU low-pass cutoff is based on nominal 1 kHz sampling rather than a measured runtime cadence; the 2026-10-22 capture measured about 424 Hz. No active-balance hardware measurement has established control-cycle timing or UART integrity under transmission load.

The wheel speed estimate uses the measured control tick. Live changes are not stored in flash and reset to the startup values after reboot, so no USB connection is needed to run the robot after flashing.

To log commands and telemetry through one serial connection, run `python3 scripts/capture_balance_trace.py --trace-mode 56 --interactive --seconds 40 --output control-live.log`. The script logs every trace row and each sent command (`# CMD ...`), while showing only every tenth trace row in the terminal. The interactive input accepts only trace-selection commands (`K0`, `K9`, `K55` to `K58`), and the bounded `T`/`M` motor commands. The `T`/`M` grammar, caps, and procedure are in [motor commissioning](motor-commissioning.md); with `--interactive`, a `T` or `M` line that is outside the grammar or caps is ignored. It cannot send calibration commands. After the capture, the script sends `K0` and logs the firmware replies for two more seconds, so result text such as `MOTOR_ID_RESULT` reaches the log.

For a repeatable agent-facing run summary, pass one or more saved logs to `scripts/summarize_balance_trace.py`. It reports valid and malformed rows, capture span, CH5 mode counts/transitions, and a small set of min/max/mean values without making causal claims:

```bash
python3 scripts/summarize_balance_trace.py balance-trace.log
python3 scripts/summarize_balance_trace.py run-1.log run-2.log --output balance-summary.md
```

For repeated K58 drive/stop trials, one command captures a new log at 576000 baud and writes a short `*.md` report after capture. It refuses to overwrite an existing log or report:

```bash
python3 scripts/run_drive_trial.py --interactive --output drive-test-01.log --seconds 40
```

The report checks trace quality, CH5 activation, CH3 movement, start tilt, and the largest opposite wheel-speed reading in the first second after a CH3 release. If there is no usable stop event, the command returns status 2 and keeps both files. It does not switch the robot. With `--interactive`, type a gain command such as `SP0.04`, then `SP` to read it back, before enabling CH5. The capture logs accepted commands as `# CMD ...` and includes firmware replies. Use a new filename for each gain setting and state the observed movement with the report path. Opening a new serial session may restart the board, so set and read back live gains again in each run.

Compare saved trials without opening Serial or recompiling firmware:

```bash
python3 scripts/analyze_drive_trace.py drive-test-01.log drive-test-02.log --require-stop --output drive-comparison.md
```

`--format json` provides the same measurements for other scripts. A trial with neutral CH3 or a strongly tilted start is flagged as unsuitable for a stop comparison; it is not interpreted as a controller improvement or regression.
