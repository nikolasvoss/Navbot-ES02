#!/usr/bin/env python3
"""Show the robot's log messages from the USB port without holding the ESP32-S3 in reset.

Usage: python3 scripts/sensor_monitor.py [--port /dev/ttyUSB0] [--seconds 10]
Reads through the navbot_link library. Press Ctrl-C to stop an unlimited capture.
"""

import argparse
import queue
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

try:
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required: python3 -m pip install pyserial")

from navbot_link.capture import describe  # noqa: E402
from navbot_link.framing import Text  # noqa: E402
from navbot_link.link import Disconnected, FramesLost, Link, LogEvent  # noqa: E402
from navbot_link.protocol import LinkError  # noqa: E402

USB_BAUD = 576_000


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
    deadline = time.monotonic() + seconds if seconds else None
    with Link(port, USB_BAUD) as link:
        while deadline is None or time.monotonic() < deadline:
            try:
                event = link.events.get(timeout=0.2)
            except queue.Empty:
                continue
            if isinstance(event, (LogEvent, FramesLost)):
                print(describe(event), flush=True)
            elif isinstance(event, Text):
                print(event.text.rstrip("\r\n"), flush=True)
            elif isinstance(event, Disconnected):
                raise LinkError(f"port closed: {event.reason}")


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
    except (OSError, LinkError, RuntimeError) as error:
        print(f"sensor_monitor: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
