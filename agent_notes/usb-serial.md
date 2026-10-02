# USB, Serial, and IMU: verified project facts

For agent uploads, use the configured `navbot_flash` MCP tool. See the [firmware flashing MCP reference](../docs/reference/firmware-flashing-mcp.md) for the tool contract and sandbox troubleshooting.

## Board connection

- The user's board has one USB-C connector. On the connected PC it enumerated as a CH340 USB-to-serial adapter (`1a86:7523`) at `/dev/ttyUSB0`. The port name may change; inspect `/dev/serial/by-id/` or run `arduino-cli board list` before flashing.
- An `esptool.py chip_id` probe through `/dev/ttyUSB0` identified the target as an ESP32-S3 (revision v0.2). Do not infer board identity from a port name alone.
- No native USB CDC port appeared when firmware with **USB CDC On Boot: Enabled** was flashed and the board rebooted. Treat the observed USB-C connection as the CH340 path; do not assume ESP32-S3 native USB is available on this board.

## Working firmware setting

- Use Arduino IDE **Tools → USB CDC On Boot → Disabled**. With Arduino CLI, use `esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default`.
- Build the default sketch from the project root with:

  ```bash
  python3 scripts/build_firmware.py
  ```

  The build helper defaults to `src/ES-02/OllieFOCdrive`, the FQBN above, and `build/flash`. Pass `--help` for overrides, including another sketch, FQBN, output directory, Arduino CLI build path, repeated build properties, and a clean build. The temporary Arduino build path defaults to a hidden sibling of the output directory so compilation can run inside the repository. The helper requires Arduino CLI and does not install or pin board cores or libraries.

- To upload that compiled build, close SerialPlot and other serial monitors, then run:

  ```bash
  python3 scripts/upload_firmware.py build/flash
  ```

  The upload helper requires `arduino-cli` and `pyserial`, checks for the sketch, bootloader and partition binaries, identifies the CH340 (`1a86:7523`), uses the board options above and asks Arduino CLI to verify the flash. Pass `--port /dev/ttyUSB*` if several CH340 adapters are connected. Close serial monitors first. The helper does not rebuild, so compile again after source or board-setting changes; a binary alone cannot reveal which CDC option was used to build it. The ESP32-S3 identity is checked by Arduino CLI's upload tool when it connects to the chip. No board is connected in every development environment, so an upload must only be reported successful after the CLI completes verification.
- The normal control sketch and diagnostic firmware use **115200 baud** on the CH340 serial connection. `SERIAL_BAUD_RATE`, `DIAGNOSTIC_SERIAL_BAUD_RATE`, and `LIVE_TUNING_SERIAL_BAUD_RATE` all select 115200 baud in the current firmware. Pass `--baud-rate 115200` to `capture_balance_trace.py` or `remote_status.py` with that live-tuning image. Use `python3 scripts/sensor_monitor.py` for the sensor-only image, or set both DTR and RTS off in SerialPlot. Asserted RTS can hold the board in reset; a SerialPlot session with `DOWNLOAD(USB/UART0)` in the ROM log needs a reset after DTR is deasserted.
- Use the configured serial MCP server for direct port discovery and ad-hoc reads or writes. The project scripts add workflow-specific behavior: `remote_status.py` returns parsed channel data as JSON, `capture_balance_trace.py` saves selected telemetry, `run_drive_trial.py` creates a K58 log and report without overwriting existing files, and `upload_firmware.py` checks the CH340 and verifies uploaded firmware hashes. The WLAN tools are `wifi_tune.py` for parameter changes and `wifi_record.py` plus `decode_wifi_trace.py` for recordings. Use those scripts when you need their documented workflows.
- The firmware was rebuilt with USB CDC disabled, flashed through `/dev/ttyUSB0`, and flash hash verification succeeded. The earlier build with USB CDC enabled also flashed successfully, but it routed sketch `Serial` away from the observed CH340 connection.

## Sensor data

- The ICM42688 is read over SPI in `src/ES-02/OllieFOCdrive/ICM42688.cpp`. `ImuUpdate()` in `OllieFOCdrive.ino` updates acceleration, gyro, temperature, and orientation values in `attitude`.
- The diagnostic firmware now exports 14-column numeric CSV at 20 Hz for SerialPlot over the existing `Serial`/CH340 connection while motor and servo initialization is skipped. README.md defines column order and units. The normal control mode still does not continuously export IMU values. Enabling USB CDC does not add telemetry.
