#!/usr/bin/env python3
"""Read one remote-control snapshot from the NavBot's USB serial port.

Agent use: python3 scripts/remote_status.py [--port /dev/ttyACM0]
Requires pyserial. Prints one JSON object to stdout and errors to stderr.
The firmware's K8 output has no receiver age/failsafe field; readings can be
stale after radio loss. Keep the robot safely supported while inspecting it.
"""

import argparse
import datetime as dt
import json
import re
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required: python3 -m pip install pyserial")


BAUD_RATE = 576_000
CHANNEL_PATTERN = re.compile(r"\bch:(\d+)")
INTERVAL_PATTERN = re.compile(r"\bsbus_dt_ms:(\d+)")
SWITCH_NAMES = {
    5: ("pid_control", ("off", "on_without_touch", "on_with_touch")),
    6: ("posture_or_mark", ("posture", "mark")),
    7: ("roll_control", ("manual", "auto")),
    8: ("attitude", ("default", "pitching_adjust", "ball_balance")),
}


def parse_line(line):
    """Return (ten raw channels, frame interval) for a complete K8 line."""
    channels = [int(value) for value in CHANNEL_PATTERN.findall(line)]
    interval = INTERVAL_PATTERN.search(line)
    if len(channels) != 10 or interval is None:
        return None
    return channels, int(interval.group(1))


def switch_position(value, labels):
    # Switch positions are estimates; transmitter calibration can differ.
    centers = (192, 992, 1792) if len(labels) == 3 else (192, 1792)
    return labels[min(range(len(centers)), key=lambda i: abs(value - centers[i]))]


def find_port():
    ports = [p.device for p in list_ports.comports() if p.device.startswith(("/dev/ttyACM", "/dev/ttyUSB"))]
    if len(ports) != 1:
        raise RuntimeError(f"Expected one USB serial port, found {ports!r}; pass --port explicitly")
    return ports[0]


def read_snapshot(port, timeout, baud_rate):
    deadline = time.monotonic() + timeout
    latest = None
    first_reading_at = None
    # Configure modem-control lines before opening. On this board RTS can hold
    # EN low, and changing DTR/RTS after opening may reset the ESP32.
    connection = serial.Serial(baudrate=baud_rate, timeout=0.1, write_timeout=1)
    connection.port = port
    connection.dtr = False
    connection.rts = False
    with connection:
        # Firmware setup can take several seconds before its command loop runs.
        next_request_at = 0.0
        try:
            while time.monotonic() < deadline:
                if first_reading_at is None and time.monotonic() >= next_request_at:
                    connection.write(b"K8\n")
                    connection.flush()
                    next_request_at = time.monotonic() + 0.5
                line = connection.readline().decode("ascii", errors="replace")
                reading = parse_line(line)
                if reading is None:
                    continue
                latest = reading
                if first_reading_at is None:
                    first_reading_at = time.monotonic()
                if time.monotonic() - first_reading_at >= 0.2:
                    break
        finally:
            # Stop the high-rate debug stream even if parsing or reading fails.
            try:
                connection.write(b"K0\n")
                connection.flush()
            except (OSError, serial.SerialException):
                # Do not hide the original read/connection error during cleanup.
                pass

    if latest is None:
        raise RuntimeError("No complete K8 channel line received before timeout")
    return latest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="USB serial port; auto-detected if exactly one exists")
    parser.add_argument("--baud-rate", type=int, default=BAUD_RATE,
                        help="serial baud rate (default: 576000)")
    parser.add_argument("--timeout", type=float, default=15.0, help="seconds to wait (default: 15)")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    try:
        port = args.port or find_port()
        channels, interval = read_snapshot(port, args.timeout, args.baud_rate)
    except (OSError, serial.SerialException, RuntimeError) as error:
        print(f"remote_status: {error}", file=sys.stderr)
        return 1

    switches = {
        name: switch_position(channels[number - 1], labels)
        for number, (name, labels) in SWITCH_NAMES.items()
    }
    result = {
        "timestamp_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "port": port,
        "raw_channels": {f"ch{number}": value for number, value in enumerate(channels, 1)},
        "switches_estimated": switches,
        "sbus_frame_interval_ms": interval,
        "receiver_status": "unknown",
    }
    print(json.dumps(result, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
