#!/usr/bin/env python3
"""Reset the NavBot ESP32-S3 over its CH340 USB-to-serial connection.

Requires pyserial. The reset uses the serial adapter's RTS line and does not
erase flash or alter firmware. Keep the robot supported while resetting it.

Usage:
    python3 scripts/reset_board.py [--port /dev/ttyUSB0]
"""

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required: python3 -m pip install pyserial")


CH340_USB_ID = (0x1A86, 0x7523)


def find_port():
    ports = [
        port.device
        for port in list_ports.comports()
        if (port.vid, port.pid) == CH340_USB_ID
    ]
    if len(ports) != 1:
        raise RuntimeError(
            f"Expected one CH340 adapter ({CH340_USB_ID[0]:04x}:{CH340_USB_ID[1]:04x}), "
            f"found {ports!r}; pass --port explicitly"
        )
    return ports[0]


def reset(port):
    connection = serial.Serial(baudrate=115200, timeout=1)
    connection.port = port
    connection.dtr = False
    connection.rts = False
    with connection:
        # Match esptool's hard-reset pulse for a board with an RTS-to-EN circuit.
        connection.rts = True
        time.sleep(0.1)
        connection.rts = False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="USB serial port; auto-detect the CH340 by default")
    args = parser.parse_args()

    try:
        port = args.port or find_port()
        reset(port)
    except (OSError, serial.SerialException, RuntimeError) as error:
        print(f"reset_board: {error}", file=sys.stderr)
        return 1

    print(f"Reset pulse sent over {port}; flash contents were not changed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
