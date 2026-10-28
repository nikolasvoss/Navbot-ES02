"""Firmware parameters and the signal subscription behind ROS parameters. No ROS imports."""

import struct
from dataclasses import dataclass

from navbot_link.link import Link, Subscription
from navbot_link.protocol import Catalog, LinkError, NoReply, Type


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
        self._unanswered = False  # the robot ignored GetCatalog on this connection
        self.subscription: Subscription | None = None

    @property
    def has_catalog(self) -> bool:
        return self._catalog is not None

    def attach(self) -> bool:
        """Read the catalog; False when the robot does not answer or refuses. A robot that
        ignored GetCatalog is not asked again until restarted() (each ask costs the whole
        retry time)."""
        if self._unanswered:
            return False
        try:
            self._catalog = self._link.catalog()
        except NoReply:
            self._catalog = None
            self._unanswered = True
        except LinkError:
            self._catalog = None
        return self._catalog is not None

    def restarted(self) -> None:
        """The robot restarted: its catalog may differ, so ask again."""
        self._catalog = None
        self._unanswered = False

    def params(self) -> list[ParamInfo]:
        return [ParamInfo(e.name, e.type, self._link.get(e.name)) for e in self._catalog.params]

    def get_param(self, name: str):
        """The robot's current value; raises LinkError."""
        return self._link.get(name)

    def set_param(self, name: str, value) -> str:
        """Returns "" when the robot applied the value, else why it did not."""
        return self.apply_param(name, value)[0]

    def apply_param(self, name: str, value) -> tuple[str, object]:
        """(reason, applied): "" and the value the robot kept (it may clamp), or why it did not apply it."""
        if self._catalog is None:
            return "the robot sent no catalog", None
        try:
            return "", self._link.set(name, value)
        except LinkError as exc:
            return str(exc), None
        except struct.error:
            return f"{name}: {value!r} does not fit {self._catalog.param(name).type.name.lower()}", None

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
        old = self.subscription

        def adopt(sub: Subscription) -> None:  # before the first row reaches poll(), which drops unknown slots
            self.subscription = sub

        try:
            self._link.subscribe(names, rate_hz, on_subscribed=adopt)
        except LinkError as exc:
            return str(exc)
        return self._release(old)

    def _release(self, sub: Subscription | None) -> str:
        if sub is None:
            return ""
        try:
            self._link.unsubscribe(sub.slot)
        except LinkError as exc:
            return f"unsubscribe slot {sub.slot}: {exc}"  # its lease runs out by itself
        return ""
