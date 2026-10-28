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
- Where `navbot_flash` is not configured, run `python3 scripts/upload_firmware.py <absolute build directory>`. It finds the CH340 itself (`--port` is only needed when several are connected) and uploads with `arduino-cli` when that is in `PATH`, otherwise with `esptool`. The `esptool` fallback erases the OTA data at `0xe000`, then writes the bootloader at `0x0`, the partition table at `0x8000`, and the app at `0x10000`, verifying each image by hash. Install it with `pipx install esptool`: the distro (apt) package ships without flasher stubs and cannot flash the ESP32-S3. The script needs the same offsets as the build's partition table; recheck them in the script if the partition scheme changes. `python3 -m pytest tests/scripts/test_upload_firmware.py` checks the tool selection and command order without a board.

- Check `list_ports` before reporting the board disconnected. Close the serial MCP connection before flashing. Keep balance disabled and motor outputs safe. The MCP workflow uploads and verifies the supplied build; it does not rebuild, so compile again after source or board-setting changes. A binary alone cannot reveal which CDC option was used to build it. No board is connected in every development environment, so report success only after the flash workflow completes verification.
- The control sketch and the PC tools use **576000 baud** on the robot's USB port (CH340). `kSerialBaudRate` selects that rate. `navlink`, `remote_status.py`, `run_drive_trial.py` and `sensor_monitor.py` use the same rate on a USB port, and the link on UART1 runs at 2000000 baud. The physical UART rate at 576000 has not yet been measured; treat this as a provisional setting pending a future hardware capture. A 2026-10-07 diagnostic capture at 921600 over `/dev/ttyUSB0` (CH340 `1a86:7523`) showed corrupted reception: over 95.54 seconds of device timestamps, 7,824 valid 17-field rows arrived (81.89/s) while sequence numbers implied 9,555 producer records (100.01/s); 1,714 numeric lines were malformed and 1,731 sequence IDs were missing. Firmware queue-drop and UART-write-failure snapshots remained zero. The capture established a problem on the CH340/host/MCP receive path but did not isolate which component caused it. Use `python3 scripts/sensor_monitor.py` for the sensor-only image, or set both DTR and RTS off in SerialPlot. Asserted RTS can hold the board in reset; a SerialPlot session with `DOWNLOAD(USB/UART0)` in the ROM log needs a reset after DTR is deasserted.
- The firmware was rebuilt with USB CDC disabled, flashed through `/dev/ttyUSB0`, and flash hash verification succeeded. The earlier build with USB CDC enabled also flashed successfully, but it routed sketch `Serial` away from the observed CH340 connection.

## Sensor data

- The ICM42688 is read over SPI in `src/ES-02/OllieFOCdrive/ICM42688.cpp`. `ImuUpdate()` in `OllieFOCdrive.ino` updates acceleration, gyro, temperature, and orientation values in `attitude`.
- The robot has no CSV stream. Signals are read through [navlink](../../integration/software/navlink.md) (see [trace capture](trace-capture.md)).
- Firmware text (setup messages, IMU timing, SimpleFOC debug output, the diagnostic boot line) goes out as `Log` frames on both ports. Boot ROM output and panic dumps are not framed.
- The sensor diagnostic build (`SENSOR_DIAGNOSTIC_MODE 1`) skips motor and servo initialization. It no longer streams a CSV; step 6 of the [logging plan](../../integration/plans/logging-and-control-link.md) decides its output.
