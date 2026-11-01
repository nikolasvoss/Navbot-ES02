# USB, Serial, and IMU: verified project facts

## Board connection

- The user's board has one USB-C connector. On the connected PC it enumerated as a CH340 USB-to-serial adapter (`1a86:7523`) at `/dev/ttyUSB0`. The port name may change: `python3 scripts/devtools.py probe` reports the robot's port as `robot_usb` on each host, and `deploy.py` finds it on every run. Recheck the board settings before each flash.
- An `esptool.py chip_id` probe through `/dev/ttyUSB0` identified the target as an ESP32-S3 (revision v0.2). Do not infer board identity from a port name alone.
- No native USB CDC port appeared when firmware with **USB CDC On Boot: Enabled** was flashed and the board rebooted. Treat the observed USB-C connection as the CH340 path; do not assume ESP32-S3 native USB is available on this board.

## Working firmware setting

- Use Arduino IDE **Tools → USB CDC On Boot → Disabled**. With Arduino CLI, use `esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default`.
- Build the default sketch from the project root with:

  ```bash
  python3 scripts/build_firmware.py
  ```

  The build helper defaults to `src/ES-02/OllieFOCdrive`, the FQBN above, and `build/flash`. Pass `--help` for overrides, including another sketch, FQBN, output directory, Arduino CLI build path, repeated build properties, and a clean build. The temporary Arduino build path defaults to a hidden sibling of the output directory so compilation can run inside the repository. The helper requires Arduino CLI and does not install or pin board cores or libraries.

- Flash only with `python3 scripts/deploy.py`, or `python3 scripts/deploy.py --image diagnostic` for the sensor diagnostic image. It builds the committed HEAD, checks the gate, flashes and checks the boot; see [agent workflow](../../integration/software/agent-workflow.md#deploy).
- `deploy.py` flashes with `python3 scripts/upload_firmware.py <build directory> --port <port>` on the host with the USB cable. The script finds the CH340 itself (`--port` is only needed when several are connected) and uploads with `arduino-cli` when that is in `PATH`, otherwise with `esptool`. The `esptool` fallback erases the OTA data at `0xe000`, then writes the bootloader at `0x0`, the partition table at `0x8000`, and the app at `0x10000`, verifying each image by hash. Install it with `pipx install esptool`: the distro (apt) package ships without flasher stubs and cannot flash the ESP32-S3. The script needs the same offsets as the build's partition table; recheck them in the script if the partition scheme changes. `python3 -m pytest tests/scripts/test_upload_firmware.py` checks the tool selection and command order without a board.

- `deploy.py` refuses to flash when another process holds the robot's USB port (exit 9) or the gate refuses (exit 4). It builds before it flashes, so the uploaded binary is the committed HEAD. A binary alone cannot reveal which CDC option was used to build it. The serial MCP is denied in `.claude/settings.json`, because opening the port resets the robot.
- The control sketch and the PC tools use **576000 baud** on the robot's USB port (CH340). `kSerialBaudRate` selects that rate. `navlink`, `remote_status.py`, `run_drive_trial.py` and `sensor_monitor.py` use the same rate on a USB port, and the link on UART1 runs at 2000000 baud. The physical UART rate at 576000 has not yet been measured; treat this as a provisional setting pending a future hardware capture. A 2026-10-07 diagnostic capture at 921600 over `/dev/ttyUSB0` (CH340 `1a86:7523`) showed corrupted reception: over 95.54 seconds of device timestamps, 7,824 valid 17-field rows arrived (81.89/s) while sequence numbers implied 9,555 producer records (100.01/s); 1,714 numeric lines were malformed and 1,731 sequence IDs were missing. Firmware queue-drop and UART-write-failure snapshots remained zero. The capture established a problem on the CH340/host/MCP receive path but did not isolate which component caused it. Read the sensor-only image with `python3 scripts/sensor_monitor.py`, which leaves DTR and RTS deasserted. Asserted RTS can hold the board in reset. If the ROM log shows `DOWNLOAD(USB/UART0)` after DTR is deasserted, reset the board.
- The firmware was rebuilt with USB CDC disabled, flashed through `/dev/ttyUSB0`, and flash hash verification succeeded. The earlier build with USB CDC enabled also flashed successfully, but it routed sketch `Serial` away from the observed CH340 connection.

## Sensor data

- The ICM42688 is read over SPI in `src/ES-02/OllieFOCdrive/ICM42688.cpp`. `ImuUpdate()` in `OllieFOCdrive.ino` updates acceleration, gyro, temperature, and orientation values in `attitude`.
- The robot has no CSV stream. Signals are read through [navlink](../../integration/software/navlink.md) (see [trace capture](trace-capture.md)).
- Firmware text (setup messages, IMU timing, SimpleFOC debug output, the diagnostic boot line) goes out as `Log` frames on both ports. Boot ROM output and panic dumps are not framed.
- The sensor diagnostic build (`SENSOR_DIAGNOSTIC_MODE 1`) skips motor and servo initialization. It no longer streams a CSV. It is a link build: its live signals are listed on the [sensor diagnostic mode](sensor-diagnostic-mode.md#which-signals-are-live) page.
