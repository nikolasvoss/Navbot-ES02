# Use the sensor diagnostic mode

The sensor diagnostic mode reads the IMU, battery ADC, and SBUS receiver without initializing motor or servo control. It holds the two motor-driver enable pins low and returns from normal setup before initializing BLE, FOC, servos, or balance control. The source currently sets `SENSOR_DIAGNOSTIC_MODE` to `0`, so a diagnostic build requires changing that define and rebuilding.

Support the robot mechanically during this check. The legs do not move in diagnostic mode.

## Read the serial stream

The diagnostic firmware sends 14 numeric CSV fields at 115200 baud, 20 times per second, over the CH340 USB serial connection. Close SerialPlot or another serial monitor before opening the port because the port allows one client at a time.

For plain text output, run:

```bash
python3 scripts/sensor_monitor.py
```

The monitor leaves DTR and RTS deasserted. In SerialPlot, select the current `/dev/ttyUSB*` port, set DTR and RTS off, and choose ASCII, comma delimiter, and 14 channels. Port names can change, so identify the connected device before use.

The first serial lines contain boot and setup text before the CSV rows. The `DIAG,boot` line reports the reset reason and IMU status. The voltage field uses direct ADC conversion and does not have the filtered value's startup delay.

If the ROM reports `DOWNLOAD(USB/UART0)` and waits for download, the ESP32 entered its flash loader. Keep DTR off, then pulse RTS or press RESET/EN. A normal firmware boot reports `SPI_FAST_FLASH_BOOT`. Do not run `sensor_monitor.py` and SerialPlot at the same time.

## Build and upload the diagnostic image

Set `SENSOR_DIAGNOSTIC_MODE` to `1` in `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`, then build from the repository root:

```bash
python3 scripts/build_firmware.py
```

Upload the resulting `build/flash` image with the configured `navbot_flash` MCP tool. Follow the current project instructions in `AGENTS.md`; close the serial connection before uploading and keep the motor outputs safe. Set the define back to `0` and rebuild when you need the normal motor-control firmware.

Sensor readings establish that the sensors and receiver respond with motor control disabled. They do not establish that the robot is safe or stable under motor load.
