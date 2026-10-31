# Use the sensor diagnostic mode

The sensor diagnostic mode initializes the IMU and the receiver without initializing motor, servo or balance control. It holds the two motor-driver enable pins low, sets up the NavBot link on both ports, and then runs its own loop (`diagnosticLoop()`), which has no control tick. The source currently sets `SENSOR_DIAGNOSTIC_MODE` to `0`. Set it to `1` and rebuild to enable this mode.

Support the robot mechanically during this check. The legs do not move in diagnostic mode. This mode does not establish safe or stable behavior under motor load.

## The link

The diagnostic image is a link build like the normal one. It serves the same catalog (the [signal table](../../integration/software/signal-table.md)) on UART1 and on the USB port, so [navlink](../../integration/software/navlink.md) and `python3 scripts/sensor_monitor.py` work with it the same way. Each port sends its `Log` frames, and the requests on each port are answered. Only one program may hold the USB port at a time.

The image sends no `WheelOdometry` or `Imu` frames, because those come from the control tick, and it runs no control tick.

### Which signals are live

The catalog is the same table as the normal image. In this image the loop updates the following; every other signal reads its initial value, which is 0:

| Live | Source |
| --- | --- |
| `ch5_position`, `rc_valid` | the CH5 switch and the RC state, from each S.BUS frame |
| `voltage_min_raw_v`, `voltage_filtered_v`, `battery_raw_instant_v` | the battery, read every 10 ms (not every control tick) |
| `roll_deg`, `pitch_deg`, `gyro_z_rad_s`, and all `imu_*` signals | the IMU, when it initialized |
| `link_tx_drop_*`, `link_rx_bad_*`, `link_req_dropped`, `link_undecodable` | the link counters |
| `uptime_s`, `heap_*`, `stack_free_*` | the system health, refreshed once a second |
| every parameter | its value, readable; `mahony_kp`, `mahony_ki` and `gyro_cutoff_hz` change the IMU filters. The balance and servo-trim values are stored, but nothing in this image reads them. `log_level` filters the text lines as in the other image |
| `motor_stop`, `motor_reset` | accepted; they change no output |

Read as 0 in this image: `ch3_target`, `body_turn_command`, `servo1_cmd_deg` to `servo4_cmd_deg`, `leg_pose_min_deg`, `leg_pose_max_deg`, the `motor1_*` and `motor2_*` signals, `control_start_us`, `control_execution_us`, `tick_dt_s` and `loop_interval_max_us`. The balance and link-control signals stay at their start values: `balance_state`, `balance_fault`, `overspeed_tripped`, `motor_test_active` and `link_balance` are 0, and `cmd_age_ms` is `4294967295` until a `Command` arrives (which this image refuses).

The actions `motor_current`, `motor_voltage`, `calibrate` and `balance` are refused with a reason in the `Ack`, because this image runs no motor loop and no calibration step. Starting a motor test would enable the drivers. `reboot` is accepted when CH5 is at 0, and restarts this image too.

### Subscriptions in this image

Rows are taken once per pass of `diagnosticLoop()`. The pass rate is not measured, and the subscription budget assumes the control tick's 710 Hz. Subscribe with a period of 1 ms or more; a period of 0 means every pass and may exceed the budget the port was checked against.

## What the mode reports

The mode sends its setup messages, the boot line `DIAG boot reset_reason=… imu_ok=… motors=off servos=off`, and a once-per-second `IMU_TIMING` line as `Log` frames on both link ports. Read them with `python3 scripts/sensor_monitor.py`, which leaves DTR and RTS deasserted, or with `navlink shell` on the ros2_pc.

## Build and upload the diagnostic image

Set `SENSOR_DIAGNOSTIC_MODE` to `1` in `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`, then build from the repository root:

```bash
python3 scripts/build_firmware.py
```

Upload the resulting `build/flash` image with the configured `navbot_flash` MCP tool. Follow the current project instructions in `AGENTS.md`; close the serial connection before uploading and keep the motor outputs safe. Set the define back to `0` and rebuild when you need the normal motor-control firmware.

If the ROM reports `DOWNLOAD(USB/UART0)` and waits for download, the ESP32 entered its flash loader. Keep DTR off, then pulse RTS or press RESET/EN. A normal firmware boot reports `SPI_FAST_FLASH_BOOT`. Do not run `sensor_monitor.py`, `navlink` and another serial program on the USB port at the same time.

Sensor readings establish that the sensors and receiver respond with motor control disabled. They do not establish that the robot is safe or stable under motor load.
