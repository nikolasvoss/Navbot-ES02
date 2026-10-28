"""A robot for tests: answers GetCatalog, Subscribe, Unsubscribe, GetParam,
SetParam and Call with Ack, and streams SignalBatch rows on a virtual
control-tick clock. It has one or more ports; slots are shared by all ports and
owned by the port that subscribed them. The clock advances one tick per produce() call
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
    limits: tuple | None = None  # (min, max) a PARAM accepts; SetParam outside it is refused


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


MAX_SLOTS = 4
MAX_PERIOD_US = 60_000_000  # the protocol page's limit; the firmware and the PC library refuse longer periods too


@dataclass
class _Port:
    usb: bool
    decoder: FrameDecoder = field(default_factory=FrameDecoder)
    out: bytearray = field(default_factory=bytearray)
    seq: int = 0


@dataclass
class _Sub:
    port: int
    ids: list
    period_us: int
    deadline_us: int
    renewed_s: float
    batch: bytes = b""  # rows not sent yet


@dataclass
class FakeFirmware:
    entries: list = field(default_factory=speed_loop_table)
    build: str = "fake-build-1"
    tick_us: int = 10_000
    lease_s: float = 5.0
    usb: bool = False  # port 0 is a USB writer: 0x00 before each frame
    t_us: int = 1_000_000
    stop_us: int | None = None  # the clock does not pass this time
    batch_rows: int = 1  # rows per SignalBatch

    def __post_init__(self):
        self.received = []  # every decoded PC payload, in order
        self.calls = []  # (name, args) of started actions
        self._ports = [_Port(self.usb)]
        self._subs: dict[int, _Sub] = {}
        self._scheduled = []  # (t_us, action) run at the first tick at or after t_us
        self._lock = threading.Lock()

    def add_port(self, usb: bool = False) -> int:
        with self._lock:
            self._ports.append(_Port(usb))
            return len(self._ports) - 1

    def receive(self, data: bytes, port: int = 0) -> None:
        with self._lock:
            for payload in self._ports[port].decoder.feed(data):
                self.received.append(payload)
                msg = pb.Message()
                msg.ParseFromString(payload)
                self._handle(msg, port)

    def produce(self, port: int = 0) -> bytes:
        """Run one control tick and return everything sent on `port` since the last call."""
        with self._lock:
            if self._subs and self.t_us != self.stop_us:
                self.t_us = (self.t_us + self.tick_us) & 0xFFFFFFFF
            for item in [x for x in self._scheduled if x[0] <= self.t_us]:
                self._scheduled.remove(item)
                item[1]()
            now_s = time.monotonic()
            for slot, sub in list(self._subs.items()):
                if now_s - sub.renewed_s > self.lease_s:
                    del self._subs[slot]
                elif self.t_us >= sub.deadline_us:
                    late = self.t_us - sub.deadline_us
                    sub.deadline_us = (self.t_us if late >= sub.period_us else sub.deadline_us) + sub.period_us
                    sub.batch += struct.pack("<I", self.t_us) + b"".join(self._read(i) for i in sub.ids)
                    if len(sub.batch) == self.batch_rows * (4 + 4 * len(sub.ids)):
                        self._send(pb.Message(signal_batch=pb.SignalBatch(slot=slot, rows=sub.batch)), sub.port)
                        sub.batch = b""
            out = bytes(self._ports[port].out)
            self._ports[port].out.clear()
            return out

    def write_text(self, text: bytes, port: int = 0) -> None:
        with self._lock:
            self._ports[port].out += text

    def log_at(self, t_us: int, text: str, tag: str) -> None:
        """Send a Log at the first tick at or after t_us."""
        with self._lock:
            self._scheduled.append((t_us, lambda: self._log(text, tag)))

    def text_at(self, t_us: int, text: bytes, port: int = 0) -> None:
        """Write raw text at the first tick at or after t_us."""
        with self._lock:
            self._scheduled.append((t_us, lambda: self._ports[port].out.extend(text)))

    def drop_frames(self, count: int) -> None:
        """The next frame's seq skips `count` values, as if they were lost."""
        with self._lock:
            for p in self._ports:
                p.seq += count

    def restart(self) -> None:
        with self._lock:
            for p in self._ports:
                p.seq = 0
            self._subs.clear()

    def subscribed_slots(self) -> list[int]:
        with self._lock:
            return sorted(self._subs)

    def _handle(self, msg, port: int) -> None:
        body = msg.WhichOneof("body")
        req = getattr(msg, body)
        ack = lambda **kw: self._ack(msg.seq, port, **kw)  # noqa: E731
        if body == "get_catalog":
            if req.index >= len(self.entries):
                return ack(error=f"no entry {req.index}")
            self._send(pb.Message(catalog_entry=self._catalog_entry(req.index)), port)
        elif body == "subscribe":
            bad = [i for i in req.ids if i >= len(self.entries) or self.entries[i].kind == E.ACTION]
            if bad:
                return ack(error=f"not a signal: {bad[0]}")
            if req.period_us > MAX_PERIOD_US:
                return ack(error=f"period_us must be at most {MAX_PERIOD_US}")
            slot = req.slot
            if slot == 0:
                free = [s for s in range(1, MAX_SLOTS + 1) if s not in self._subs]
                if not free:
                    return ack(error=f"all {MAX_SLOTS} slots are in use")
                slot = free[0]
            elif slot not in self._subs:
                return ack(error=f"no subscription in slot {slot}")
            old = self._subs.get(slot)
            if old and old.port != port:
                return ack(error="slot in use")
            same = old and old.ids == list(req.ids) and old.period_us == req.period_us
            deadline = old.deadline_us if same else self.t_us + self.tick_us
            self._subs[slot] = _Sub(port, list(req.ids), req.period_us, deadline, time.monotonic(),
                                    old.batch if same else b"")
            ack(value=struct.pack("<I", slot))
        elif body == "unsubscribe":
            old = self._subs.get(req.slot)
            if old and old.port != port:
                return ack(error="slot in use")
            self._subs.pop(req.slot, None)
            ack()
        elif body == "get_param":
            ack(value=self._read(req.id))
        elif body == "set_param":
            entry = self.entries[req.id]
            if entry.kind != E.PARAM:
                return ack(error=f"{entry.name} is not a parameter")
            old = self._read(req.id)
            (new,) = struct.unpack(_FMT[entry.type], req.value)
            if entry.limits and not entry.limits[0] <= new <= entry.limits[1]:
                return ack(error=f"{entry.name} must be within [{entry.limits[0]:g}, {entry.limits[1]:g}]")
            entry.value = new
            ack(value=self._read(req.id))
            self._log_uart1(f"{entry.name}: {_show(entry, old)} -> {_show(entry, req.value)}", "param")
        elif body == "call":
            entry = self.entries[req.id]
            self.calls.append((entry.name, [round(a, 6) for a in req.args]))
            ack()
            shown = "".join(f" {arg[0]}={value:g}" for arg, value in zip(entry.args, req.args))
            self._log_uart1(f"{entry.name} started{shown}", "action")

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

    def _ack(self, seq: int, port: int, error: str = "", value: bytes = b"") -> None:
        self._send(pb.Message(ack=pb.Ack(command_id=seq, accepted=not error, error=error, value=value)), port)

    def emit(self, msg, port: int = 0) -> None:
        """Send any message, such as Imu or WheelOdometry, with the port's next seq."""
        with self._lock:
            self._send(msg, port)

    def log(self, text: str, tag: str, level: int = pb.Log.INFO) -> None:
        with self._lock:
            self._log(text, tag, level)

    def _log(self, text: str, tag: str, level: int = pb.Log.INFO) -> None:
        for port in range(len(self._ports)):
            self._send(pb.Message(log=pb.Log(level=level, text=text, tag=tag)), port)

    def _log_uart1(self, text: str, tag: str, level: int = pb.Log.INFO) -> None:
        """Log frames go to UART1 only until migration step 5 (the firmware's LinkLogReachesPort)."""
        self._send(pb.Message(log=pb.Log(level=level, text=text, tag=tag)), 0)

    def _send(self, msg, port: int) -> None:
        p = self._ports[port]
        p.seq += 1
        msg.seq = p.seq
        msg.t_ms = self.t_us // 1000
        p.out += (b"\x00" if p.usb else b"") + encode_frame(msg.SerializeToString())


def _show(entry: FakeEntry, data: bytes) -> str:
    (value,) = struct.unpack(_FMT[entry.type], data)
    return f"{value:.6g}" if entry.type == E.F32 else str(value)


class MemoryTransport:
    """The Link's side of an in-memory wire to one port of a FakeFirmware."""

    def __init__(self, firmware: FakeFirmware, port: int = 0):
        self.firmware = firmware
        self.port = port
        self.closed = False

    def read(self) -> bytes:
        time.sleep(0.001)
        return self.firmware.produce(self.port)

    def write(self, data: bytes) -> None:
        self.firmware.receive(data, self.port)

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
