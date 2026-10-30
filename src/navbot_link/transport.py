"""Byte transports to the robot's link: the serial port itself, or the multiplexer's TCP address."""

import socket


class SerialTransport:
    def __init__(self, port: str, baud: int):
        import serial

        self._ser = serial.Serial()
        self._ser.port = port
        self._ser.baudrate = baud
        self._ser.timeout = 0.05
        # Asserting DTR or RTS on open resets the board through the CH340
        # auto-reset circuit; it can still reset on some adapters.
        self._ser.dtr = False
        self._ser.rts = False
        self._ser.open()

    def read(self) -> bytes:
        return self._ser.read(max(1, self._ser.in_waiting))

    def write(self, data: bytes) -> None:
        self._ser.write(data)

    def close(self) -> None:
        self._ser.close()


class SocketTransport:
    """A connection to the link multiplexer (navbot_link.mux)."""

    def __init__(self, host: str, port: int):
        self._sock = socket.create_connection((host, port), timeout=0.05)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def read(self) -> bytes:
        try:
            data = self._sock.recv(65536)
        except TimeoutError:
            return b""
        if not data:
            raise OSError("multiplexer closed the connection")
        return data

    def write(self, data: bytes) -> None:
        self._sock.sendall(data)

    def close(self) -> None:
        self._sock.close()


def open_transport(port: str, baud: int):
    """`tcp:HOST:PORT` for the multiplexer, anything else is a serial port path."""
    if port.startswith("tcp:"):
        host, _, number = port[len("tcp:"):].rpartition(":")
        return SocketTransport(host, int(number))
    return SerialTransport(port, baud)
