"""Catalog entries, 4-byte values and sequence tracking. No I/O."""

import struct
from dataclasses import dataclass
from enum import IntEnum

from navbot_link import navbot_link_pb2 as pb

_Entry = pb.CatalogEntry


class Kind(IntEnum):
    SIGNAL = _Entry.SIGNAL
    SIGNAL_MIN = _Entry.SIGNAL_MIN
    SIGNAL_MAX = _Entry.SIGNAL_MAX
    PARAM = _Entry.PARAM
    ACTION = _Entry.ACTION


class Type(IntEnum):
    F32 = _Entry.F32
    I32 = _Entry.I32
    U32 = _Entry.U32
    BOOL = _Entry.BOOL


_FORMAT = {Type.F32: "<f", Type.I32: "<i", Type.U32: "<I", Type.BOOL: "<I"}


class LinkError(Exception):
    pass


class UnknownName(LinkError):
    pass


@dataclass(frozen=True)
class Arg:
    name: str
    min: float
    max: float


@dataclass(frozen=True)
class Entry:
    index: int
    name: str
    unit: str
    kind: Kind
    type: Type | None  # None for an action
    args: tuple[Arg, ...] = ()


@dataclass(frozen=True)
class Catalog:
    build: str
    entries: tuple[Entry, ...]

    def _find(self, name: str, kinds: set[Kind], what: str) -> Entry:
        for entry in self.entries:
            if entry.name == name and entry.kind in kinds:
                return entry
        raise UnknownName(f"unknown {what} {name}")

    def signal(self, name: str) -> Entry:
        """Anything that can be streamed or read: signals and parameters."""
        return self._find(name, set(Kind) - {Kind.ACTION}, "signal")

    def param(self, name: str) -> Entry:
        return self._find(name, {Kind.PARAM}, "parameter")

    def action(self, name: str) -> Entry:
        return self._find(name, {Kind.ACTION}, "action")

    @property
    def params(self) -> tuple[Entry, ...]:
        return tuple(e for e in self.entries if e.kind == Kind.PARAM)


def entry_from_pb(msg) -> Entry:
    kind = Kind(msg.kind)
    return Entry(
        index=msg.index,
        name=msg.name,
        unit=msg.unit,
        kind=kind,
        type=None if kind == Kind.ACTION else Type(msg.type),
        args=tuple(Arg(a.name, a.min, a.max) for a in msg.args),
    )


def pack_value(type_: Type, value) -> bytes:
    if type_ == Type.F32:
        return struct.pack("<f", float(value))
    if type_ == Type.BOOL:
        return struct.pack("<I", 1 if value else 0)
    return struct.pack(_FORMAT[type_], int(value))


def unpack_value(type_: Type, data: bytes):
    (value,) = struct.unpack(_FORMAT[type_], data)
    return bool(value) if type_ == Type.BOOL else value


def format_value(type_: Type, value) -> str:
    if type_ == Type.F32:
        return f"{value:.7g}"  # float32 carries about 7 significant digits
    if type_ == Type.BOOL:
        return "1" if value else "0"
    return str(value)


def parse_value(type_: Type, text: str):
    if type_ == Type.BOOL:
        if text.lower() in ("1", "true", "on"):
            return True
        if text.lower() in ("0", "false", "off"):
            return False
        raise ValueError(f"not a bool: {text}")
    return float(text) if type_ == Type.F32 else int(text, 0)


def unpack_rows(types: tuple[Type, ...], data: bytes) -> list[tuple[int, tuple]]:
    """SignalBatch.rows: [t_us u32][4 B per entry] repeated."""
    size = 4 + 4 * len(types)
    if len(data) % size:
        raise LinkError(f"SignalBatch of {len(data)} bytes does not fit rows of {size}")
    rows = []
    for off in range(0, len(data), size):
        (t_us,) = struct.unpack_from("<I", data, off)
        values = tuple(
            unpack_value(t, data[off + 4 + 4 * i:off + 8 + 4 * i]) for i, t in enumerate(types)
        )
        rows.append((t_us, values))
    return rows


@dataclass(frozen=True)
class SeqStep:
    lost: int  # frames missing before this one
    restarted: bool  # the sender started counting again


class SeqTracker:
    """Per-port Message.seq: a gap is a lost frame, a non-increasing value a restart.

    A sender counts from 1 and a uint32 takes years to wrap, so wrap is not handled.
    """

    def __init__(self):
        self._prev = None
        self.lost_frames = 0
        self.restarts = 0

    def observe(self, seq: int) -> SeqStep:
        prev, self._prev = self._prev, seq
        if prev is None:
            return SeqStep(0, False)
        if seq <= prev:
            self.restarts += 1
            return SeqStep(0, True)
        lost = seq - prev - 1
        self.lost_frames += lost
        return SeqStep(lost, False)
