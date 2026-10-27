"""A robot-side link endpoint for tests: answers GetCatalog, Subscribe,
Unsubscribe, GetParam, SetParam and Call with Ack, and streams SignalBatch rows
on a virtual control-tick clock. The clock advances one tick per produce() call
while a subscription is active and stands still otherwise, so rows and their
t_us do not depend on PC timing. It is written from the protocol page, not from
the PC library, and shares only framing and the generated schema with it."""

import os
import select
import struct
import threading
import time
from dataclasses import dataclass, field

from navbot_link import navbot_link_pb2 as pb
from navbot_link.framing import FrameDecoder, encode_frame

E = pb.CatalogEntry
_FMT = {E.F32: "<f", E.I32: "<i", E.U32: "<I", E.BOOL: "<I"}


@dataclass
class FakeEntry:
    name: str
    kind: int
    type: int = 0
    unit: str = ""
    value: object = 0  # a number, or a function of t_us for a signal
    args: tuple = ()  # (name, min, max) for an action


def speed_loop_table() -> list[FakeEntry]:
    """The plan's speed-loop example plus one entry of each remaining type and kind."""
    tick = lambda t: t // 10_000  # noqa: E731
    return [
        FakeEntry("speed_target", E.SIGNAL, E.F32, "m/s", lambda t: 0.5),
        FakeEntry("speed_measured", E.SIGNAL, E.F32, "m/s", lambda t: tick(t) % 4 * 0.125),
        FakeEntry("speed_p", E.SIGNAL, E.F32, "rad", lambda t: -0.03125),
        FakeEntry("speed_i", E.SIGNAL, E.F32, "rad", lambda t: tick(t) % 4 * 0.001),
        FakeEntry("tilt_ref", E.SIGNAL, E.F32, "rad", lambda t: 0.0),
        FakeEntry("tilt_clipped", E.SIGNAL, E.BOOL, "", lambda t: tick(t) % 2),
        FakeEntry("loop_count", E.SIGNAL, E.U32, "", tick),
        FakeEntry("battery_min", E.SIGNAL_MIN, E.F32, "V", lambda t: 7.5),
        FakeEntry("wheel_offset", E.SIGNAL, E.I32, "", lambda t: -3),
        FakeEntry("speed_kp", E.PARAM, E.F32, "rad/(m/s)", 0.08),
        FakeEntry("speed_ki", E.PARAM, E.F32, "rad/(m/s*s)", 0.5),
        FakeEntry("speed_i_on", E.PARAM, E.BOOL, "", 1),
        FakeEntry("motor_reset", E.ACTION),
        FakeEntry("motor_current", E.ACTION, args=(("left_a", -2.0, 2.0), ("right_a", -2.0, 2.0),
                                                   ("lease_ms", 1.0, 1000.0))),
    ]


@dataclass
class _Sub:
    ids: list
    period_us: int
    deadline_us: int
    renewed_s: float


@dataclass
class FakeFirmware:
    entries: list = field(default_factory=speed_loop_table)
    build: str = "fake-build-1"
    tick_us: int = 10_000
    lease_s: float = 5.0
    usb: bool = False  # USB writer: 0x00 before each frame
    t_us: int = 1_000_000

    def __post_init__(self):
        self.received = []  # every decoded PC payload, in order
        self.calls = []  # (name, args) of started actions
        self._decoder = FrameDecoder()
        self._out = bytearray()
        self._seq = 0
        self._subs: dict[int, _Sub] = {}
        self._scheduled_logs = []  # (t_us, text, tag)
        self._lock = threading.Lock()


    def receive(self, data: bytes) -> None:
        with self._lock:
            for payload in self._decoder.feed(data):
                self.received.append(payload)
                msg = pb.Message()
                msg.ParseFromString(payload)
                self._handle(msg)

    def produce(self) -> bytes:
        """Run one control tick and return everything sent since the last call."""
        with self._lock:
            if self._subs:
                self.t_us = (self.t_us + self.tick_us) & 0xFFFFFFFF
            for item in [x for x in self._scheduled_logs if x[0] <= self.t_us]:
                self._scheduled_logs.remove(item)
                self._log(item[1], item[2])
            now_s = time.monotonic()
            for slot, sub in list(self._subs.items()):
                if now_s - sub.renewed_s > self.lease_s:
                    del self._subs[slot]
                elif self.t_us >= sub.deadline_us:
                    late = self.t_us - sub.deadline_us
                    sub.deadline_us = (self.t_us if late >= sub.period_us else sub.deadline_us) + sub.period_us
                    row = struct.pack("<I", self.t_us) + b"".join(self._read(i) for i in sub.ids)
                    self._send(pb.Message(signal_batch=pb.SignalBatch(slot=slot, rows=row)))
            out, self._out = bytes(self._out), bytearray()
            return out

    def write_text(self, text: bytes) -> None:
        with self._lock:
            self._out += text

    def log_at(self, t_us: int, text: str, tag: str) -> None:
        """Send a Log at the first tick at or after t_us."""
        with self._lock:
            self._scheduled_logs.append((t_us, text, tag))

    def drop_frames(self, count: int) -> None:
        """The next frame's seq skips `count` values, as if they were lost."""
        with self._lock:
            self._seq += count

    def restart(self) -> None:
        with self._lock:
            self._seq = 0
            self._subs.clear()

    def subscribed_slots(self) -> list[int]:
        with self._lock:
            return sorted(self._subs)


    def _handle(self, msg) -> None:
        body = msg.WhichOneof("body")
        req = getattr(msg, body)
        if body == "get_catalog":
            if req.index >= len(self.entries):
                return self._ack(msg.seq, error=f"no entry {req.index}")
            self._send(pb.Message(catalog_entry=self._catalog_entry(req.index)))
        elif body == "subscribe":
            bad = [i for i in req.ids if i >= len(self.entries) or self.entries[i].kind == E.ACTION]
            if bad:
                return self._ack(msg.seq, error=f"not a signal: {bad[0]}")
            old = self._subs.get(req.slot)
            same = old and old.ids == list(req.ids) and old.period_us == req.period_us
            deadline = old.deadline_us if same else self.t_us + self.tick_us
            self._subs[req.slot] = _Sub(list(req.ids), req.period_us, deadline, time.monotonic())
            self._ack(msg.seq)
        elif body == "unsubscribe":
            self._subs.pop(req.slot, None)
            self._ack(msg.seq)
        elif body == "get_param":
            self._ack(msg.seq, value=self._read(req.id))
        elif body == "set_param":
            entry = self.entries[req.id]
            if entry.kind != E.PARAM:
                return self._ack(msg.seq, error=f"{entry.name} is not a parameter")
            old = self._read(req.id)
            (entry.value,) = struct.unpack(_FMT[entry.type], req.value)
            self._ack(msg.seq, value=self._read(req.id))
            self._log(f"{entry.name}: {_show(entry, old)} -> {_show(entry, req.value)}", "param")
        elif body == "call":
            entry = self.entries[req.id]
            self.calls.append((entry.name, [round(a, 6) for a in req.args]))
            self._ack(msg.seq)
            self._log(f"{entry.name} started", "action")

    def _catalog_entry(self, index: int):
        e = self.entries[index]
        msg = E(index=index, count=len(self.entries), name=e.name, unit=e.unit, kind=e.kind, type=e.type,
                args=[E.Arg(name=n, min=lo, max=hi) for n, lo, hi in e.args])
        if index == 0:
            msg.build = self.build
        return msg

    def _read(self, index: int) -> bytes:
        e = self.entries[index]
        value = e.value(self.t_us) if callable(e.value) else e.value
        return struct.pack(_FMT[e.type], value)

    def _ack(self, seq: int, error: str = "", value: bytes = b"") -> None:
        self._send(pb.Message(ack=pb.Ack(command_id=seq, accepted=not error, error=error, value=value)))

    def log(self, text: str, tag: str) -> None:
        with self._lock:
            self._log(text, tag)

    def _log(self, text: str, tag: str) -> None:
        self._send(pb.Message(log=pb.Log(level=pb.Log.INFO, text=text, tag=tag)))

    def _send(self, msg) -> None:
        self._seq += 1
        msg.seq = self._seq
        msg.t_ms = self.t_us // 1000
        self._out += (b"\x00" if self.usb else b"") + encode_frame(msg.SerializeToString())


def _show(entry: FakeEntry, data: bytes) -> str:
    (value,) = struct.unpack(_FMT[entry.type], data)
    return f"{value:.7g}" if entry.type == E.F32 else str(value)


class MemoryTransport:
    """The Link's side of an in-memory wire to a FakeFirmware."""

    def __init__(self, firmware: FakeFirmware):
        self.firmware = firmware
        self.closed = False

    def read(self) -> bytes:
        time.sleep(0.001)
        return self.firmware.produce()

    def write(self, data: bytes) -> None:
        self.firmware.receive(data)

    def close(self) -> None:
        self.closed = True


class PtyFirmware:
    """Serves a FakeFirmware on a pseudo-terminal; `path` is the port to open."""

    def __init__(self, firmware: FakeFirmware):
        self.firmware = firmware
        self._master, slave = os.openpty()
        self.path = os.ttyname(slave)
        self._slave = slave
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            ready, _, _ = select.select([self._master], [], [], 0.001)
            if ready:
                try:
                    self.firmware.receive(os.read(self._master, 4096))
                except OSError:
                    return
            out = self.firmware.produce()
            if out:
                os.write(self._master, out)

    def close(self) -> None:
        self._stop.set()
        self._thread.join()
        os.close(self._master)
        os.close(self._slave)
