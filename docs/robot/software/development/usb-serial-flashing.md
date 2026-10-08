# USB, Serial, and IMU: verified project facts

## Board connection

- The user's board has one USB-C connector. On the connected PC it enumerated as a CH340 USB-to-serial adapter (`1a86:7523`) at `/dev/ttyUSB0`. The port name may change; check the serial MCP's `list_ports` and recheck the board settings before each flash.
- An `esptool.py chip_id` probe through `/dev/ttyUSB0` identified the target as an ESP32-S3 (revision v0.2). Do not infer board identity from a port name alone.
- No native USB CDC port appeared when firmware with **USB CDC On Boot: Enabled** was flashed and the board rebooted. Treat the observed USB-C connection as the CH340 path; do not assume ESP32-S3 native USB is available on this board.

## Working firmware setting

- Use Arduino IDE **Tools → USB CDC On Boot → Disabled**. With Arduino CLI, use `esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default`.
- Build the default sketch from the project root with:

  ```bash
  python3 scripts/build_firmware.py
  ```

  The build helper defaults to `src/ES-02/OllieFOCdrive`, the FQBN above, and `build/flash`. Pass `--help` for overrides, including another sketch, FQBN, output directory, Arduino CLI build path, repeated build properties, and a clean build. The temporary Arduino build path defaults to a hidden sibling of the output directory so compilation can run inside the repository. The helper requires Arduino CLI and does not install or pin board cores or libraries.

- To upload that compiled build, use the configured `navbot_flash` MCP tool `flash_firmware` and pass the absolute build directory from the active worktree. For the default build, this is the absolute path to `build/flash`.

- Check `list_ports` before reporting the board disconnected. Close the serial MCP connection before flashing. Keep balance disabled and motor outputs safe. The MCP workflow uploads and verifies the supplied build; it does not rebuild, so compile again after source or board-setting changes. A binary alone cannot reveal which CDC option was used to build it. No board is connected in every development environment, so report success only after the flash workflow completes verification.
- The normal control sketch, diagnostic firmware, live-tuning serial stream, and direct PC tools currently use **576000 baud** on the existing CH340 UART. `SERIAL_BAUD_RATE`, `DIAGNOSTIC_SERIAL_BAUD_RATE`, and `LIVE_TUNING_SERIAL_BAUD_RATE` select that rate. `capture_balance_trace.py`, `remote_status.py`, `run_drive_trial.py`, and `sensor_monitor.py` use the same rate by default. The physical UART rate at 576000 has not yet been measured; treat this as a provisional setting pending a future hardware capture. A 2026-10-07 diagnostic capture at 921600 over `/dev/ttyUSB0` (CH340 `1a86:7523`) showed corrupted reception: over 95.54 seconds of device timestamps, 7,824 valid 17-field rows arrived (81.89/s) while sequence numbers implied 9,555 producer records (100.01/s); 1,714 numeric lines were malformed and 1,731 sequence IDs were missing. Firmware queue-drop and UART-write-failure snapshots remained zero. The capture established a problem on the CH340/host/MCP receive path but did not isolate which component caused it. Use `python3 scripts/sensor_monitor.py` for the sensor-only image, or set both DTR and RTS off in SerialPlot. Asserted RTS can hold the board in reset; a SerialPlot session with `DOWNLOAD(USB/UART0)` in the ROM log needs a reset after DTR is deasserted.
- The firmware was rebuilt with USB CDC disabled, flashed through `/dev/ttyUSB0`, and flash hash verification succeeded. The earlier build with USB CDC enabled also flashed successfully, but it routed sketch `Serial` away from the observed CH340 connection.

## Sensor data

- The ICM42688 is read over SPI in `src/ES-02/OllieFOCdrive/ICM42688.cpp`. `ImuUpdate()` in `OllieFOCdrive.ino` updates acceleration, gyro, temperature, and orientation values in `attitude`.
- The diagnostic firmware exports 17-column numeric CSV at a nominal 100 Hz over the existing `Serial`/CH340 connection while motor and servo initialization is skipped. Each row follows a new software IMU SPI read and carries its pre-read microsecond timestamp and sequence. A static bounded queue separates producers from the UART sender; a full queue drops rows and increments a counter rather than waiting for transmission. The nominal line budget is checked by `python3 scripts/test_serial_logger.py`; this is software evidence, not a hardware baud acceptance. README.md defines column order and units. The normal control mode still does not continuously export IMU values. Enabling USB CDC does not add telemetry.

- The shared formatter check exercises the exact emitted CSV format with LF, maximum-width timestamps, sequence values, and counters. Its fixture produced a 155-byte diagnostic row at 100 Hz (155,000 framed bits/s, 26.9% of 576000 baud) and a 311-byte DRIVE row at every seventh executed gate, nominally 142.9 Hz (444,286 framed bits/s, 77.1%). UART accounting uses 10 bits per byte for start/data/stop framing. The DRIVE number is the largest selected trace fixture; actual lengths depend on field values. The selected active trace modes are mutually exclusive.
- Selected debug modes `K1` through `K45` are paced to at most 50 rows/s by the shared logger; `K8` retains its existing 20 rows/s gate. Pacing skips intermediate snapshots without marking the recording incomplete. A row fits in the 384-byte sender buffer, so 50 rows/s uses less than 192,000 framed bits/s of the 576000-baud UART budget. This prevents sustained debug output from filling the queue; a genuinely stalled sender still latches the existing incomplete state. Diagnostic and active trace rates are unchanged.

- Active trace rows are queued once per seventh executed control gate whose existing `time_dt >= 1 ms` condition passes, nominally 142.9 Hz with a 1 kHz gate. This does not alter that gate. Nyquist is nominally 71.4 Hz, so higher-frequency components can alias. The active IMU filter remains configured for a 50 Hz cutoff at nominal 1 kHz sampling; actual loop and sensor-read cadence under control is not established by the serial diagnostic check.
- Future improvement: make the transmitted channels selectable so a capture can omit unused data points and reduce its wire budget. The current row schemas remain fixed; channel selection is not implemented.
