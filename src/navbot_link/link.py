"""Link: one serial port to the robot, with catalog, subscriptions, parameters,
actions and an ordered event stream."""

import queue
import threading
import time
from dataclasses import dataclass
from typing import Protocol

from google.protobuf.message import DecodeError

from navbot_link import navbot_link_pb2 as pb
from navbot_link.framing import FrameDecoder, Text, encode_frame
from navbot_link.protocol import (
    Catalog,
    Entry,
    LinkError,
    SeqTracker,
    entry_from_pb,
    pack_value,
    unpack_rows,
    unpack_value,
)

MAX_SLOTS = 4
MAX_SUBSCRIBED = 48
LEASE_RENEW_S = 2.0  # firmware lease is 5 s


class Transport(Protocol):
    def read(self) -> bytes:
        """Return the bytes received so far, or b"" after a short idle timeout."""

    def write(self, data: bytes) -> None: ...

    def close(self) -> None: ...


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


@dataclass(frozen=True)
class Row:
    slot: int
    t_us: int
    values: tuple


@dataclass(frozen=True)
class LogEvent:
    t_ms: int
    level: str
    code: int
    text: str
    tag: str


@dataclass(frozen=True)
class FramesLost:
    count: int
    seq: int  # first frame after the gap


@dataclass(frozen=True)
class Restarted:
    """The robot's seq started over: catalog and subscriptions are gone."""


@dataclass(frozen=True)
class Disconnected:
    reason: str


@dataclass(frozen=True)
class SubscriptionEnded:
    slot: int
    reason: str


@dataclass(frozen=True)
class Subscription:
    slot: int
    entries: tuple[Entry, ...]
    period_us: int

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(e.name for e in self.entries)


class _Waiter:
    def __init__(self):
        self._event = threading.Event()
        self.reply = None

    def resolve(self, reply) -> None:
        self.reply = reply
        self._event.set()

    def wait(self, timeout_s: float) -> bool:
        return self._event.wait(timeout_s)


class Link:
    """Owns the port: one I/O thread decodes frames, answers waiters, renews leases
    and puts Row, LogEvent, Text, FramesLost, Restarted, SubscriptionEnded and
    Disconnected on
    `events` in stream order."""

    def __init__(self, port, baud: int = 2_000_000, *, timeout_s: float = 0.5,
                 retries: int = 3, renew_s: float = LEASE_RENEW_S, clock=time.monotonic):
        self._transport: Transport = SerialTransport(port, baud) if isinstance(port, str) else port
        self._timeout_s = timeout_s
        self._retries = retries
        self._renew_s = renew_s
        self.events: queue.Queue = queue.Queue()
        self._decoder = FrameDecoder()
        self._seq_in = SeqTracker()
        self._lock = threading.Lock()
        self._seq_out = 0
        self._clock = clock  # seconds; sets Message.t_ms of requests
        self._started = clock()
        self._waiters: dict[tuple, _Waiter] = {}
        self._renewals: dict[int, int] = {}  # renewal request seq -> slot
        self._subs: dict[int, Subscription] = {}
        self._catalog: Catalog | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="navbot-link", daemon=True)
        self._thread.start()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    @property
    def lost_frames(self) -> int:
        return self._seq_in.lost_frames

    @property
    def bad_frames(self) -> int:
        return self._decoder.bad_frames

    def close(self) -> None:
        self._stop.set()
        self._thread.join()
        self._transport.close()

    def catalog(self) -> Catalog:
        """Read once per connection and again after a robot restart."""
        if self._catalog is None:
            first = self._get_entry(0)
            entries = [entry_from_pb(first)] if first.count else []
            entries += [entry_from_pb(self._get_entry(i)) for i in range(1, first.count)]
            self._catalog = Catalog(first.build, tuple(entries))
        return self._catalog

    def get(self, name: str):
        entry = self.catalog().signal(name)
        ack = self._request(pb.Message(get_param=pb.GetParam(id=entry.index)))
        return unpack_value(entry.type, ack.value)

    def set(self, name: str, value):
        """Returns the value the robot applied."""
        entry = self.catalog().param(name)
        ack = self._request(pb.Message(
            set_param=pb.SetParam(id=entry.index, value=pack_value(entry.type, value))))
        return unpack_value(entry.type, ack.value)

    def call(self, name: str, **args) -> None:
        entry = self.catalog().action(name)
        values = check_args(entry, args)
        # Not retried: a lost Ack must not start a motor test twice.
        self._request(pb.Message(call=pb.Call(id=entry.index, args=values)), retries=1)

    def subscribe(self, names, rate_hz: float | None = None) -> Subscription:
        """rate_hz None or 0 streams every control tick."""
        catalog = self.catalog()
        entries = tuple(catalog.signal(n) for n in names)
        if not entries or len(entries) > MAX_SUBSCRIBED:
            raise LinkError(f"a subscription takes 1 to {MAX_SUBSCRIBED} signals, got {len(entries)}")
        with self._lock:
            free = [s for s in range(1, MAX_SLOTS + 1) if s not in self._subs]
            if not free:
                raise LinkError(f"all {MAX_SLOTS} slots are in use")
            sub = Subscription(free[0], entries, round(1e6 / rate_hz) if rate_hz else 0)
            self._subs[sub.slot] = sub  # before the request: rows may beat the Ack
        try:
            self._request(_subscribe_message(sub))
        except LinkError:
            with self._lock:
                self._subs.pop(sub.slot, None)
            raise
        return sub

    def unsubscribe(self, slot: int) -> None:
        with self._lock:
            self._subs.pop(slot, None)
        self._request(pb.Message(unsubscribe=pb.Unsubscribe(slot=slot)))

    def _get_entry(self, index: int):
        return self._request(pb.Message(get_catalog=pb.GetCatalog(index=index)),
                             reply_key=("catalog", index)).catalog_entry

    def _request(self, msg, *, reply_key=None, retries=None):
        what = msg.WhichOneof("body")
        for _ in range(retries or self._retries):
            waiter = _Waiter()
            with self._lock:
                seq = self._send_locked(msg)
                keys = [("ack", seq)] + ([reply_key] if reply_key else [])
                for key in keys:
                    self._waiters[key] = waiter
            try:
                if not waiter.wait(self._timeout_s):
                    continue
            finally:
                with self._lock:
                    for key in keys:
                        self._waiters.pop(key, None)
            reply = waiter.reply
            if reply.WhichOneof("body") == "ack":
                if not reply.ack.accepted:
                    raise LinkError(f"{what} refused: {reply.ack.error}")
                return reply.ack
            return reply
        raise LinkError(f"no reply to {what}")

    def _send_locked(self, msg) -> int:
        self._seq_out += 1
        msg.seq = self._seq_out
        msg.t_ms = int((self._clock() - self._started) * 1000) & 0xFFFFFFFF
        self._transport.write(encode_frame(msg.SerializeToString()))
        return msg.seq

    def _run(self) -> None:
        next_renew = time.monotonic() + self._renew_s
        while not self._stop.is_set():
            try:
                chunk = self._transport.read()
            except OSError as exc:  # pyserial's SerialException is an OSError
                self.events.put(Disconnected(str(exc)))
                return
            for item in self._decoder.feed(chunk) if chunk else self._decoder.flush():
                if isinstance(item, Text):
                    self.events.put(item)
                else:
                    self._dispatch(item)
            if time.monotonic() >= next_renew:
                next_renew = time.monotonic() + self._renew_s
                with self._lock:
                    for sub in self._subs.values():
                        self._renewals[self._send_locked(_subscribe_message(sub))] = sub.slot

    def _dispatch(self, payload: bytes) -> None:
        msg = pb.Message()
        try:
            msg.ParseFromString(payload)
        except DecodeError:
            self._decoder.bad_frames += 1
            return
        step = self._seq_in.observe(msg.seq)
        if step.restarted:
            self._on_restart()
        elif step.lost:
            self.events.put(FramesLost(step.lost, msg.seq))

        body = msg.WhichOneof("body")
        if body == "ack":
            self._on_ack(msg)
        elif body == "catalog_entry":
            self._resolve(("catalog", msg.catalog_entry.index), msg)
        elif body == "signal_batch":
            sub = self._subs.get(msg.signal_batch.slot)
            if sub is not None:
                try:
                    rows = unpack_rows(tuple(e.type for e in sub.entries), msg.signal_batch.rows)
                except LinkError:
                    self._decoder.bad_frames += 1
                    return
                for t_us, values in rows:
                    self.events.put(Row(sub.slot, t_us, values))
        elif body == "log":
            log = msg.log
            self.events.put(LogEvent(msg.t_ms, pb.Log.Level.Name(log.level), log.code, log.text, log.tag))

    def _on_ack(self, msg) -> None:
        ack = msg.ack
        with self._lock:
            slot = self._renewals.pop(ack.command_id, None)
            ended = slot is not None and not ack.accepted and self._subs.pop(slot, None) is not None
        if slot is None:
            self._resolve(("ack", ack.command_id), msg)
        elif ended:
            self.events.put(SubscriptionEnded(slot, f"renewal refused: {ack.error}"))

    def _on_restart(self) -> None:
        with self._lock:
            ended = sorted(self._subs)
            self._subs.clear()
            self._renewals.clear()
            self._catalog = None
        self.events.put(Restarted())
        for slot in ended:
            self.events.put(SubscriptionEnded(slot, "robot restarted"))

    def _resolve(self, key, msg) -> None:
        with self._lock:
            waiter = self._waiters.get(key)
        if waiter is not None:
            waiter.resolve(msg)


def check_args(entry: Entry, args: dict) -> list[float]:
    """Argument values in catalog order, checked against the catalog bounds."""
    names = [a.name for a in entry.args]
    unknown = sorted(set(args) - set(names))
    missing = [n for n in names if n not in args]
    if unknown or missing:
        raise LinkError(f"{entry.name} takes {', '.join(names) or 'no arguments'}"
                        f"{'; unknown: ' + ', '.join(unknown) if unknown else ''}"
                        f"{'; missing: ' + ', '.join(missing) if missing else ''}")
    values = []
    for arg in entry.args:
        value = float(args[arg.name])
        if not arg.min <= value <= arg.max:
            raise LinkError(f"{arg.name}={value:g} is outside [{arg.min:g}, {arg.max:g}]")
        values.append(value)
    return values


def _subscribe_message(sub: Subscription):
    return pb.Message(subscribe=pb.Subscribe(
        slot=sub.slot, ids=[e.index for e in sub.entries], period_us=sub.period_us))
