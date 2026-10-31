"""Turns health rows into the values of the diagnostics and battery topics. No ROS imports."""

from dataclasses import dataclass

from navbot_link.health import BALANCE_STATES, FAULT_REASONS
from navbot_link_bridge.bridge import HealthRow


@dataclass(frozen=True)
class HealthReport:
    level: str  # "OK", "WARN" or "ERROR"
    message: str
    values: tuple[tuple[str, str], ...]  # (signal, shown value); state and fault are shown by name
    voltage_v: float


def _name(names: tuple[str, ...], value) -> str:
    index = int(value)
    return names[index] if 0 <= index < len(names) else f"unknown ({index})"


class HealthMonitor:
    """Keeps the link counters of the previous row, to see when one rose."""

    def __init__(self):
        self._counters: tuple | None = None

    def reset(self) -> None:
        """A new boot or a new link: the counters start again, so the next row is the baseline."""
        self._counters = None

    def report(self, row: HealthRow) -> HealthReport:
        signals = dict(zip(row.names, row.values))
        state = _name(BALANCE_STATES, signals["balance_state"])
        fault = int(signals["balance_fault"])
        fault_name = _name(FAULT_REASONS, fault)
        counters = tuple(v for n, v in signals.items() if n.startswith("link_"))
        rose = self._counters is not None and any(now > before for now, before in zip(counters, self._counters))
        self._counters = counters

        if fault or signals["overspeed_tripped"]:
            level = "ERROR"
        elif rose:
            level = "WARN"
        else:
            level = "OK"
        shown = {"balance_state": state, "balance_fault": fault_name}
        return HealthReport(
            level=level,
            message=state + (f": {fault_name}" if fault else ""),
            values=tuple((name, shown.get(name, str(value))) for name, value in zip(row.names, row.values)),
            voltage_v=float(signals["voltage_filtered_v"]),
        )
