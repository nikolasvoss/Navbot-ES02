# Use the sensor diagnostic mode

The sensor diagnostic mode reads the IMU, battery ADC, and SBUS receiver without initializing motor or servo control. It holds the two motor-driver enable pins low and returns from normal setup before initializing BLE, FOC, servos, or balance control. The source currently sets `SENSOR_DIAGNOSTIC_MODE` to `0`. Set it to `1` and rebuild to enable this procedure.

Support the robot mechanically during this check. The legs do not move in diagnostic mode. This mode does not establish safe or stable behavior under motor load.

## Read the serial stream

The diagnostic firmware sends 17 numeric CSV fields at 576000 baud over the existing CH340 UART. It schedules one row after each fresh software SPI IMU read, nominally every 10 ms (100 Hz). The row contains the pre-read `time_us`, a monotonic `sequence`, the existing sensor/receiver values, and cumulative logger queue-drop and UART-write-failure counters. `time_us` marks the start of the SPI read, not PC receipt time. The sensor remains configured at its existing 1 kHz output rate; the software reads the latest sample at nominal 100 Hz, so this does not mean a new conversion is produced only every 10 ms. Close SerialPlot or another serial monitor before opening the port because the port allows one client at a time.

For plain text output, run:

```bash
python3 scripts/sensor_monitor.py
```

The monitor leaves DTR and RTS deasserted. In SerialPlot, select the current `/dev/ttyUSB*` port, set DTR and RTS off, and choose ASCII, comma delimiter, and 17 channels. The first channel is `time_us`, followed by `sequence`; the remaining fields are gyro XYZ (rad/s), acceleration XYZ (g), roll/pitch/yaw (degrees), raw battery voltage, receiver age/failsafe, IMU-ready, queue drops, and UART-write failures. Port names can change, so identify the connected device before use.

A motor-disabled capture at 576000 baud on 2026-10-07 used the pre-`SerialLogger` 25-field diagnostic image. It contained 250 complete rows with `imu_ok=1`. The 249 timestamp intervals had a 10.000 ms median and ranged from 9.966 to 10.040 ms; none exceeded 15 ms. The current 17-field `SerialLogger` format was not used in this capture, so these measurements do not verify its UART integrity.

The same stationary capture showed filtered-to-unfiltered standard-deviation ratios of 0.641 for gyro X and 0.733 for acceleration Z. Raw battery readings ranged from 7.250077 to 7.307390 V, while filtered readings ranged from 7.262542 to 7.293375 V. These values show smoothing of the stationary signals. They do not verify a 20 Hz cutoff because no controlled vibration or frequency sweep was applied. The host regression test exercises the production filter functions for the 20 Hz and 100 Hz case.

Historical hardware capture at 921600 baud on 2026-10-07 did not pass: across 95.54 seconds of device timestamps, 7,824 valid 17-field rows arrived (81.89 rows/s) out of about 100.01 producer records/s inferred from sequence IDs. There were 1,714 malformed numeric lines, 1,731 missing sequence IDs, and zero reported firmware queue drops or UART-write failures. The host receive path was losing or corrupting bytes; the test did not isolate the CH340, cable, host driver, or MCP. A later capture at 576000 baud confirmed the older 25-field diagnostic stream's timing. It did not verify the current 17-field logger format. Run a UART integrity capture with the current format before treating that path as accepted.

The software IMU read rate is nominally 100 Hz. The diagnostic LPF cutoff is 20 Hz at a nominal 100 Hz software sample rate (10,000 µs period), giving a 50 Hz Nyquist limit. Raw gyro and acceleration fields bypass that LPF, so content above 50 Hz can alias; the 20 Hz filter does not protect those raw fields. Actual timing is determined by software scheduling and must be checked from `time_us` intervals. The first serial lines contain boot and setup text before the CSV rows. The `DIAG,boot` line reports the reset reason and IMU status. The voltage field uses direct ADC conversion and does not have the filtered value's startup delay.

If the ROM reports `DOWNLOAD(USB/UART0)` and waits for download, the ESP32 entered its flash loader. Keep DTR off, then pulse RTS or press RESET/EN. A normal firmware boot reports `SPI_FAST_FLASH_BOOT`. Do not run `sensor_monitor.py` and SerialPlot at the same time.

## Build and upload the diagnostic image

Set `SENSOR_DIAGNOSTIC_MODE` to `1` in `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`, then build from the repository root:

```bash
python3 scripts/build_firmware.py
```

Upload the resulting `build/flash` image with the configured `navbot_flash` MCP tool. Follow the current project instructions in `AGENTS.md`; close the serial connection before uploading and keep the motor outputs safe. Set the define back to `0` and rebuild when you need the normal motor-control firmware.

Sensor readings establish that the sensors and receiver respond with motor control disabled. They do not establish that the robot is safe or stable under motor load.
