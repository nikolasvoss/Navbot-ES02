"""CSV captures and signal presets for the host tools."""

from dataclasses import dataclass
from pathlib import Path

from navbot_link.link import FramesLost, LogEvent, Row, SubscriptionEnded
from navbot_link.protocol import Entry, format_value


@dataclass(frozen=True)
class Preset:
    name: str
    signals: tuple[str, ...]
    rate_hz: float  # 0 = every control tick


def load_presets(path: Path) -> dict[str, Preset]:
    import yaml

    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    presets = {}
    for name, body in data.items():
        signals = body.get("signals") if isinstance(body, dict) else None
        rate = body.get("rate_hz", 0) if isinstance(body, dict) else None
        if (not signals or not isinstance(signals, list) or not all(isinstance(s, str) for s in signals)
                or not isinstance(rate, (int, float)) or rate < 0):
            raise ValueError(f"{path}: preset {name} needs a list 'signals' and a number 'rate_hz' >= 0")
        presets[name] = Preset(name, tuple(signals), float(rate))
    return presets


def rate_text(rate_hz: float) -> str:
    return f"{rate_hz:g} Hz" if rate_hz else "every tick"


class CsvWriter:
    """Header comments (build, rate, every parameter), the `t_us,<names>` column
    header, one line per row, and a comment line for each log event or gap."""

    def __init__(self, f, entries: tuple[Entry, ...], build: str, rate_hz: float,
                 params: list[tuple[Entry, object]]):
        self._f = f
        self._entries = entries
        lines = [f"# build: {build}", f"# rate: {rate_text(rate_hz)}"]
        lines += [f"# param {e.name}={format_value(e.type, v)}" for e, v in params]
        lines.append(",".join(["t_us", *(e.name for e in entries)]))
        f.write("\n".join(lines) + "\n")

    def row(self, row: Row) -> None:
        values = (format_value(e.type, v) for e, v in zip(self._entries, row.values))
        self._f.write(",".join([str(row.t_us), *values]) + "\n")

    def event(self, event) -> None:
        self._f.write(f"# {describe(event)}\n")

    def close(self) -> None:
        self._f.close()


def describe(event) -> str:
    if isinstance(event, LogEvent):
        tag = f" [{event.tag}]" if event.tag else ""
        code = f" (code {event.code})" if event.code else ""
        return f"log t_ms={event.t_ms} {event.level}{tag} {event.text}{code}"
    if isinstance(event, FramesLost):
        return f"lost {event.count} frames before seq {event.seq}"
    if isinstance(event, SubscriptionEnded):
        return f"slot {event.slot} ended: {event.reason}"
    raise TypeError(event)
