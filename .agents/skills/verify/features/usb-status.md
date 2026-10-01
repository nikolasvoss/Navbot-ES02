# Read RC status over USB

The USB status command shows the latest SBUS channel values and estimated switch positions from the robot's CH340 serial connection.

## Sub-features

- `status-snapshot` requests one parsed view of the ten channels.
- `switch-estimates` maps CH5 through CH8 to estimated switch labels.
- `receiver-interval` reports the latest SBUS frame interval.

## How to get to it (user POV)

- Run `python3 scripts/remote_status.py` in a terminal connected to the robot over USB.
- Pass `--port` when the computer sees more than one USB serial device.

## Driving it with the USB serial CLI

Preconditions:

- The robot is supported and its existing firmware is running.
- The configured serial MCP server lists the intended CH340 device. Use the current port name, not a remembered name.
- Use 115200 baud for the diagnostic or live-tuning firmware. Use the script's 2,000,000 baud default for the normal firmware.
- No other serial monitor owns the port.

- **Request the snapshot.** Run `python3 scripts/remote_status.py --port /dev/ttyUSB0 --baud-rate 115200 --timeout 15`. Exit code `0` and one JSON object show `raw_channels`, `switches_estimated`, `sbus_frame_interval_ms`, and `receiver_status`.
- **Check the cleanup.** The script sends `K0` in its `finally` path after the K8 read. Confirm that it exits and releases the serial port before another serial tool uses it.
- **Proof.** Save the command, stdout, stderr, and exit code under the run evidence directory. The result is a point-in-time channel snapshot, not a receiver failsafe report.

## Gotchas

- Opening serial can reset the board. The script sets DTR and RTS low before opening the port.
- The script needs local access to the serial device. If the device appears only through the serial MCP server, use MCP for ad-hoc inspection and record the CLI route as unavailable in that environment.
- The reported receiver status is always `unknown`; K8 does not include receiver age or failsafe state.
- Switch labels are estimates and depend on transmitter calibration.
