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


_TYPE_RULE = {Type.F32: "a number", Type.I32: "an integer", Type.U32: "an unsigned integer", Type.BOOL: "a bool"}


def _as_type(type_: Type, value):
    """The value the robot takes for `type_`, or None when the type does not fit. An int is taken
    as a float by an f32 parameter; a bool is never a number."""
    if type_ == Type.BOOL:
        return value if isinstance(value, bool) else None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    if type_ == Type.F32:
        return float(value)
    return value if isinstance(value, int) else None


class FirmwareControls:
    """What the catalog makes possible. Without one (its read failed on this connection) every
    operation refuses and the bridge carries telemetry only."""

    def __init__(self, link: Link):
        self._link = link
        self._catalog: Catalog | None = None
        self.subscription: Subscription | None = None

    def attach(self) -> None:
        """Read the catalog. Raises LinkError when the robot does not answer or refuses."""
        self._catalog = self._link.catalog()

    def restarted(self) -> None:
        """The robot restarted: its catalog may differ, so the old one goes until attach() reads the new one."""
        self._catalog = None

    def params(self) -> list[ParamInfo]:
        return [ParamInfo(e.name, e.type, self._link.get(e.name)) for e in self._catalog.params]

    def get_param(self, name: str):
        """The robot's current value; raises LinkError."""
        return self._link.get(name)

    def set_param(self, name: str, value) -> str:
        """Returns "" when the robot applied the value, else why it did not."""
        return self.apply_param(name, value)[0]

    def apply_param(self, name: str, value) -> tuple[str, object]:
        """(reason, applied): "" and the value the robot kept (it may clamp), or why it did not apply it.
        A value of the wrong type is refused here and never sent."""
        if self._catalog is None:
            return "the robot sent no catalog", None
        try:
            type_ = self._catalog.param(name).type
        except LinkError as exc:
            return str(exc), None
        taken = _as_type(type_, value)
        if taken is None:
            return f"{name} takes {_TYPE_RULE[type_]}, got {value!r}", None
        try:
            return "", self._link.set(name, taken)
        except LinkError as exc:
            return str(exc), None
        except struct.error:
            return f"{name}: {value!r} does not fit {type_.name.lower()}", None

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
