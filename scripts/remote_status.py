#!/usr/bin/env python3
"""Read one remote-control snapshot from the NavBot's USB serial port.

Agent use: python3 scripts/remote_status.py [--port /dev/ttyACM0]
Reads the receiver-derived signals through the navbot_link library and prints one JSON object to stdout;
errors go to stderr. The raw S.BUS channels are not in the signal table, so only the commands the controller
takes from them are reported. Keep the robot safely supported while inspecting it.
"""

import argparse
import datetime as dt
import json
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

from navbot_link.link import Disconnected, Link, Row  # noqa: E402
from navbot_link.protocol import LinkError  # noqa: E402

BAUD_RATE = 576_000
SIGNALS = ["ch5_mode", "ch3_target", "body_turn_command"]


def find_port():
    ports = [
        port.device
        for port in list_ports.comports()
        if (port.vid, port.pid) == (0x1A86, 0x7523)
    ]
    if len(ports) != 1:
        raise RuntimeError(f"Expected one CH340 USB adapter, found {ports!r}; pass --port")
    return ports[0]


def read_snapshot(port, timeout_s, baud):
    deadline = time.monotonic() + timeout_s
    with Link(port, baud) as link:
        sub = link.subscribe(SIGNALS, 10)
        try:
            while time.monotonic() < deadline:
                try:
                    event = link.events.get(timeout=0.2)
                except queue.Empty:
                    continue
                if isinstance(event, Row) and event.slot == sub.slot:
                    return {entry.name: value for entry, value in zip(sub.entries, event.values)}
                if isinstance(event, Disconnected):
                    raise LinkError(f"port closed: {event.reason}")
        finally:
            link.unsubscribe(sub.slot)
    raise RuntimeError("No signal row received before timeout")


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
        signals = read_snapshot(port, args.timeout, args.baud_rate)
    except (OSError, LinkError, RuntimeError) as error:
        print(f"remote_status: {error}", file=sys.stderr)
        return 1

    result = {
        "timestamp_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "port": port,
        "signals": signals,
        "receiver_status": "unknown",
    }
    print(json.dumps(result, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
