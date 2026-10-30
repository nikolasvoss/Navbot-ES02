"""Link multiplexer: owns the robot's serial port and shares it with local clients
over TCP (127.0.0.1:7650 by default). Clients speak the unchanged frame format,
so Link works through it with `tcp:127.0.0.1:7650` as its port.

Each client has its own sequence numbers, so frames routed to other clients do
not look like losses. Replies go back to the client that asked, and a slot
belongs to the client that subscribed it.

Run: python3 -m navbot_link.mux --port /dev/ttyAMA3 --baud 2000000 --listen 127.0.0.1:7650
"""

import argparse
import selectors
import socket
import sys
import time
from dataclasses import dataclass, field

import serial
from google.protobuf.message import DecodeError

from navbot_link import navbot_link_pb2 as pb
from navbot_link.framing import FrameDecoder, Text, encode_frame
from navbot_link.link import IDLE_FLUSH_S
from navbot_link.protocol import SeqTracker

LISTEN = "127.0.0.1:7650"
PENDING_S = 5.0  # a reply later than this is dropped
SEND_TIMEOUT_S = 1.0  # a client that does not take a frame in this time is disconnected
SELECT_S = 0.05


@dataclass(eq=False)
class _Client:
    sock: socket.socket
    decoder: FrameDecoder = field(default_factory=FrameDecoder)
    seq: int = 0  # the last Message.seq sent to this client


@dataclass
class _Pending:
    """A request forwarded to the robot. `client` None: the multiplexer's own request, whose Ack is dropped."""
    client: _Client | None
    client_seq: int
    body: str
    sent_s: float
    slot: int = 0  # unsubscribe: the slot it names


def _open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0
    # As SerialTransport: DTR and RTS low on open, so the board does not reset.
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


class Multiplexer:
    """One thread runs `serve()`; `close()` stops it from another thread."""

    def __init__(self, port: str, baud: int, listener: socket.socket):
        self._ser = _open_serial(port, baud)
        self._listener = listener
        self._listener.setblocking(False)
        self._sel = selectors.DefaultSelector()
        self._decoder = FrameDecoder()  # robot bytes
        self._seq_in = SeqTracker()  # robot Message.seq
        self._last_rx = time.monotonic()
        self._mux_seq = 0  # Message.seq of every request sent to the robot
        self._pending: dict[int, _Pending] = {}  # mux seq -> request waiting for its Ack
        self._catalog_waits: dict[int, list[_Client]] = {}  # catalog index -> clients waiting for it
        self._slots: dict[int, _Client] = {}  # robot slot -> owning client
        self._clients: list[_Client] = []
        self._stop = False

    def close(self) -> None:
        self._stop = True

    def serve(self) -> None:
        """Runs until close(). Raises OSError when the serial port fails."""
        self._sel.register(self._ser, selectors.EVENT_READ)
        self._sel.register(self._listener, selectors.EVENT_READ)
        try:
            while not self._stop:
                for key, _ in self._sel.select(SELECT_S):
                    if key.fileobj is self._ser:
                        self._read_robot()
                    elif key.fileobj is self._listener:
                        self._accept()
                    else:
                        self._read_client(key.data)
                self._expire_pending()
                if time.monotonic() - self._last_rx >= IDLE_FLUSH_S:
                    for item in self._decoder.flush():
                        self._on_robot_item(item)
        finally:
            for client in list(self._clients):
                self._close(client)
            self._sel.close()
            self._ser.close()
            self._listener.close()

    def _accept(self) -> None:
        try:
            conn, _ = self._listener.accept()
        except BlockingIOError:
            return
        conn.setblocking(True)
        conn.settimeout(SEND_TIMEOUT_S)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        client = _Client(conn)
        self._clients.append(client)
        self._sel.register(conn, selectors.EVENT_READ, client)

    def _read_client(self, client: _Client) -> None:
        try:
            data = client.sock.recv(65536)
        except OSError:
            data = b""
        if not data:
            self._disconnect(client)
            return
        for item in client.decoder.feed(data):
            if isinstance(item, bytes):  # Text from a client is not a request
                self._on_client_frame(client, item)

    def _read_robot(self) -> None:
        chunk = self._ser.read(max(1, self._ser.in_waiting))
        if not chunk:
            return
        self._last_rx = time.monotonic()
        for item in self._decoder.feed(chunk):
            self._on_robot_item(item)

    def _on_robot_item(self, item) -> None:
        if isinstance(item, Text):
            data = item.text.encode("ascii") + b"\x00"
            for client in list(self._clients):
                self._send_raw(client, data)
        else:
            self._on_robot_frame(item)

    def _on_robot_frame(self, payload: bytes) -> None:
        msg = pb.Message()
        try:
            msg.ParseFromString(payload)
        except DecodeError:
            return
        step = self._seq_in.observe(msg.seq)
        if step.restarted:
            self._on_restart()
        for client in self._clients:
            client.seq += step.lost  # a real gap on the robot is a gap for every client

        body = msg.WhichOneof("body")
        if body == "ack":
            self._on_ack(msg)
        elif body == "catalog_entry":
            for client in dict.fromkeys(self._catalog_waits.pop(msg.catalog_entry.index, [])):
                self._send_message(client, msg)
        elif body == "signal_batch":
            owner = self._slots.get(msg.signal_batch.slot)
            if owner is not None:
                self._send_message(owner, msg)
        else:
            for client in list(self._clients):
                self._send_message(client, msg)

    def _on_ack(self, msg) -> None:
        ack = msg.ack
        pending = self._pending.pop(ack.command_id, None)
        if pending is None:
            return
        if pending.body == "subscribe" and ack.accepted and pending.client is not None and len(ack.value) == 4:
            self._slots[int.from_bytes(ack.value, "little")] = pending.client
        elif pending.body == "unsubscribe" and ack.accepted:
            self._slots.pop(pending.slot, None)
        if pending.client is None:
            return
        ack.command_id = pending.client_seq
        self._send_message(pending.client, msg)

    def _on_client_frame(self, client: _Client, payload: bytes) -> None:
        msg = pb.Message()
        try:
            msg.ParseFromString(payload)
        except DecodeError:
            return
        body = msg.WhichOneof("body")
        slot = 0  # the slot a request names, when it names one
        if body == "subscribe":
            slot = msg.subscribe.slot
        elif body == "unsubscribe":
            slot = msg.unsubscribe.slot
        owner = self._slots.get(slot)
        if owner is not None and owner is not client:
            refusal = pb.Message(t_ms=msg.t_ms, ack=pb.Ack(command_id=msg.seq, accepted=False, error="slot in use"))
            self._send_message(client, refusal)
            return
        if body == "get_catalog":
            self._catalog_waits.setdefault(msg.get_catalog.index, []).append(client)
        self._mux_seq += 1
        self._pending[self._mux_seq] = _Pending(client, msg.seq, body, time.monotonic(), slot=slot)
        msg.seq = self._mux_seq
        self._write_robot(msg)

    def _on_restart(self) -> None:
        self._slots.clear()
        for client in self._clients:
            client.seq = 0  # the next frame to each client is seq 1, so every client sees the restart

    def _expire_pending(self) -> None:
        if self._pending:
            now = time.monotonic()
            self._pending = {seq: p for seq, p in self._pending.items() if now - p.sent_s < PENDING_S}

    def _write_robot(self, msg) -> None:
        # The leading 0x00 ends any partial bytes the robot still holds, as in Link.
        self._ser.write(b"\x00" + encode_frame(msg.SerializeToString()))

    def _send_message(self, client: _Client, msg) -> None:
        client.seq += 1
        msg.seq = client.seq
        self._send_raw(client, encode_frame(msg.SerializeToString()))

    def _send_raw(self, client: _Client, data: bytes) -> None:
        if client not in self._clients:
            return
        try:
            client.sock.sendall(data)
        except OSError:
            self._disconnect(client)

    def _disconnect(self, client: _Client) -> None:
        if client not in self._clients:
            return
        self._close(client)
        owned = [slot for slot, owner in self._slots.items() if owner is client]
        for slot in owned:
            del self._slots[slot]
        self._pending = {seq: p for seq, p in self._pending.items() if p.client is not client}
        for index in list(self._catalog_waits):
            waiting = [c for c in self._catalog_waits[index] if c is not client]
            if waiting:
                self._catalog_waits[index] = waiting
            else:
                del self._catalog_waits[index]
        for slot in owned:
            self._mux_seq += 1
            self._pending[self._mux_seq] = _Pending(None, 0, "unsubscribe", time.monotonic(), slot=slot)
            self._write_robot(pb.Message(seq=self._mux_seq, unsubscribe=pb.Unsubscribe(slot=slot)))

    def _close(self, client: _Client) -> None:
        self._sel.unregister(client.sock)
        self._clients.remove(client)
        client.sock.close()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Own the robot's link port and share it with local clients over TCP.")
    parser.add_argument("--port", required=True, help="the robot's serial port, e.g. /dev/ttyAMA3")
    parser.add_argument("--baud", type=int, default=2_000_000)
    parser.add_argument("--listen", default=LISTEN, metavar="HOST:PORT", help="where clients connect")
    args = parser.parse_args(argv)
    host, _, number = args.listen.rpartition(":")
    try:
        listener = socket.create_server((host, int(number)))
        Multiplexer(args.port, args.baud, listener).serve()
    except OSError as exc:
        print(f"navbot-link-mux: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
