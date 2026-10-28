# Use the sensor diagnostic mode

The sensor diagnostic mode initializes the IMU and the receiver without initializing motor or servo control. It holds the two motor-driver enable pins low and returns from normal setup before initializing FOC, servos, or balance control. The source currently sets `SENSOR_DIAGNOSTIC_MODE` to `0`. Set it to `1` and rebuild to enable this mode.

Support the robot mechanically during this check. The legs do not move in diagnostic mode. This mode does not establish safe or stable behavior under motor load.

## What the mode reports

The mode sends its setup messages and a once-per-second `IMU_TIMING` line as `Log` frames on both link ports. Read them with `python3 scripts/sensor_monitor.py`, which leaves DTR and RTS deasserted, or with `navlink shell` on the ros2_pc.

The mode no longer streams a sample CSV, and the `K` selectors are gone with the logger. The output of this mode is decided by step 6 of the [logging and control plan](../../integration/plans/logging-and-control-link.md); until then, the mode gives the boot line, the IMU timing and the setup messages only.

## Build and upload the diagnostic image

Set `SENSOR_DIAGNOSTIC_MODE` to `1` in `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`, then build from the repository root:

```bash
python3 scripts/build_firmware.py
```

Upload the resulting `build/flash` image with the configured `navbot_flash` MCP tool. Follow the current project instructions in `AGENTS.md`; close the serial connection before uploading and keep the motor outputs safe. Set the define back to `0` and rebuild when you need the normal motor-control firmware.

If the ROM reports `DOWNLOAD(USB/UART0)` and waits for download, the ESP32 entered its flash loader. Keep DTR off, then pulse RTS or press RESET/EN. A normal firmware boot reports `SPI_FAST_FLASH_BOOT`. Do not run `sensor_monitor.py` and another serial program at the same time.

Sensor readings establish that the sensors and receiver respond with motor control disabled. They do not establish that the robot is safe or stable under motor load.
