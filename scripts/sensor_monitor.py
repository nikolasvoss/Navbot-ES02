#!/usr/bin/env python3
"""Show the safe diagnostic stream without holding the ESP32-S3 in reset.

Usage: python3 scripts/sensor_monitor.py [--port /dev/ttyUSB0] [--seconds 10]
Requires pyserial. Press Ctrl-C to stop an unlimited capture.
"""

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required: python3 -m pip install pyserial")


def find_port():
    ports = [
        port.device
        for port in list_ports.comports()
        if (port.vid, port.pid) == (0x1A86, 0x7523)
    ]
    if len(ports) != 1:
        raise RuntimeError(f"Expected one CH340 USB adapter, found {ports!r}; pass --port")
    return ports[0]


def monitor(port, seconds):
    # On this board an asserted RTS holds EN low. Deassert it before opening;
    # opening the port can still cause a brief reset, but must not hold reset.
    connection = serial.Serial(baudrate=115200, timeout=0.2)
    connection.port = port
    connection.dtr = False
    connection.rts = False
    deadline = time.monotonic() + seconds if seconds else None
    with connection:
        while deadline is None or time.monotonic() < deadline:
            line = connection.readline()
            if line:
                print(line.decode("ascii", errors="replace").rstrip(), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="CH340 serial port; auto-detected when unique")
    parser.add_argument("--seconds", type=float, help="capture duration; default until Ctrl-C")
    args = parser.parse_args()
    if args.seconds is not None and args.seconds <= 0:
        parser.error("--seconds must be positive")
    try:
        monitor(args.port or find_port(), args.seconds)
    except KeyboardInterrupt:
        return 0
    except (OSError, serial.SerialException, RuntimeError) as error:
        print(f"sensor_monitor: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
