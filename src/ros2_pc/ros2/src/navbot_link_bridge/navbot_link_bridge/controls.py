"""Firmware parameters and the signal subscription behind ROS parameters. No ROS imports."""

import struct
from dataclasses import dataclass

from navbot_link.link import Link, Subscription
from navbot_link.protocol import Catalog, LinkError, Type


@dataclass(frozen=True)
class ParamInfo:
    name: str
    type: Type
    value: object  # bool, int or float, as read from the robot


class FirmwareControls:
    """What the catalog makes possible. Without a catalog (firmware that does not
    answer GetCatalog) every operation refuses and the bridge carries telemetry only."""

    def __init__(self, link: Link):
        self._link = link
        self._catalog: Catalog | None = None
        self.subscription: Subscription | None = None

    @property
    def has_catalog(self) -> bool:
        return self._catalog is not None

    def attach(self) -> bool:
        """Read the catalog; False when the robot does not answer or refuses."""
        try:
            self._catalog = self._link.catalog()
        except LinkError:
            self._catalog = None
        return self._catalog is not None

    def params(self) -> list[ParamInfo]:
        return [ParamInfo(e.name, e.type, self._link.get(e.name)) for e in self._catalog.params]

    def set_param(self, name: str, value) -> str:
        """Returns "" when the robot applied the value, else why it did not."""
        if self._catalog is None:
            return "the robot sent no catalog"
        try:
            self._link.set(name, value)
        except LinkError as exc:
            return str(exc)
        except struct.error:
            return f"{name}: {value!r} does not fit {self._catalog.param(name).type.name.lower()}"
        return ""

    def set_subscription(self, names: list[str], rate_hz: float) -> str:
        """Streams `names` (none stops the stream) at `rate_hz`, 0 for every control tick.
        Returns "" on success. On failure the previous subscription keeps running."""
        if rate_hz < 0:
            return f"signals_rate_hz must not be negative, got {rate_hz:g}"
        if not names:
            old, self.subscription = self.subscription, None
            return self._release(old)
        if self._catalog is None:
            return "the robot sent no catalog"
        try:
            new = self._link.subscribe(names, rate_hz)
        except LinkError as exc:
            return str(exc)
        old, self.subscription = self.subscription, new
        return self._release(old)

    def _release(self, sub: Subscription | None) -> str:
        if sub is None:
            return ""
        try:
            self._link.unsubscribe(sub.slot)
        except LinkError as exc:
            return f"unsubscribe slot {sub.slot}: {exc}"  # its lease runs out by itself
        return ""
