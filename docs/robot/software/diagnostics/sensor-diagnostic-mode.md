# Plot and capture sensor data with SerialPlot

The sensor diagnostic mode reads the IMU, battery ADC, and SBUS receiver without initializing motor or servo control. It holds both motor-driver enable pins low and returns from setup before BLE, FOC, servos, or balance control are initialized. `SENSOR_DIAGNOSTIC_MODE` defaults to `0`; enable it in the sketch and rebuild before using this procedure.

Support the robot mechanically during this check. The legs do not move in diagnostic mode. The diagnostic stream does not calculate controller outputs and cannot establish safe or stable behavior under motor load.

## Connect and record

Open SerialPlot and configure its ASCII reader:

- Select the CH340 serial port at 115200 baud. Keep DTR and RTS deasserted; asserted RTS can hold the ESP32 in reset.
- Choose ASCII data, comma as the column delimiter, and 25 channels (or automatic channel detection).
- Leave **Filter by Prefix** disabled. This option filters whole lines; it does not remove characters from CSV rows.
- Connect. Close other serial monitors first because the port accepts one client at a time.

Use SerialPlot's snapshot/export control to save a CSV capture. Each firmware row contains 24 sensor/status values and the monotonic timestamp in seconds as column 25. Rows contain only comma-separated numeric values and end with a newline. The nominal cadence is 50 ms (20 Hz) over the current CH340 USB-UART connection. A device check observed approximately 50 ms between frames in this motor-disabled diagnostic mode; behavior under motor load has not been measured.

SerialPlot provides live multi-channel plots and CSV snapshots; its upstream feature list does not include FFT plots. Use the exported CSV for offline FFT analysis, and measure the sample intervals before trusting the frequency axis. Column 25 is the monotonic timestamp; columns 1–24 are listed below.

## Channel map

The catalog order is fixed because SerialPlot reads values by position. Do not reorder or remove fields without updating this table. “Unfiltered” means bias-corrected and scaled sensor values before the software biquad filter; raw IMU integer counts are not included. Gyroscope values are in radians per second despite stale comments elsewhere in the firmware.

| CSV index | Signal | Unit |
| ---: | --- | --- |
| 1–3 | Unfiltered gyro X, Y, Z | rad/s |
| 4–6 | Filtered gyro X, Y, Z | rad/s |
| 7–9 | Unfiltered acceleration X, Y, Z | g |
| 10–12 | Filtered acceleration X, Y, Z | g |
| 13–15 | Mahony roll, pitch, yaw | deg |
| 16–18 | Complementary roll, pitch, yaw | deg |
| 19 | IMU temperature | °C |
| 20 | Battery voltage from the current raw ADC sample | V |
| 21 | Battery voltage from the filtered ADC value | V |
| 22 | Age of the last SBUS frame (`-1` before the first frame) | ms |
| 23 | SBUS failsafe (`-1` before the first frame) | numeric flag |
| 24 | IMU initialization status | numeric flag |
| 25 | Monotonic timestamp since boot | seconds |

Controller and motor outputs are not part of this sensor-only diagnostic stream. The safe startup path does not run controller calculations; capturing those values needs a separate decision about how the controller should run and whether motor actuation is allowed.

## Build and upload the diagnostic image

Set `SENSOR_DIAGNOSTIC_MODE` to `1` in `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`, then build from the repository root:

```bash
python3 scripts/build_firmware.py
```

Upload the resulting `build/flash` image with the configured `navbot_flash` MCP tool. Follow `AGENTS.md`; close the serial connection before uploading and keep balance disabled with motor outputs safe. Set the define back to `0` and rebuild when you need the normal motor-control firmware.

If the ROM reports `DOWNLOAD(USB/UART0)` and waits for download, the ESP32 entered its flash loader. Keep DTR off, then pulse RTS or press RESET/EN. A normal firmware boot reports `SPI_FAST_FLASH_BOOT`.
