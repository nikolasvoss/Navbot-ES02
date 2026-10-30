"""Turns Link events into odometry states, IMU samples, robot log lines and signal rows.
No ROS imports."""

from dataclasses import dataclass

from navbot_link import navbot_link_pb2
from navbot_link.link import (
    CommandRefused, Disconnected, Link, LogEvent, Restarted, Row, SubscriptionEnded, Telemetry,
)
from navbot_link_bridge.controls import FirmwareControls
from navbot_link_bridge.odometry import DiffDriveOdometry, OdometryState

# rcl_interfaces/msg/Log levels
ROS_SEVERITY = {"DEBUG": 10, "INFO": 20, "WARN": 30, "ERROR": 40, "FATAL": 50}
DEFAULT_LOGGER = "robot"


@dataclass(frozen=True)
class ImuSample:
    linear_acceleration: tuple[float, float, float]  # m/s^2
    angular_velocity: tuple[float, float, float]  # rad/s
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    t_us: int  # robot micros() at the IMU read; use only differences (wraps at 2**32)


@dataclass(frozen=True)
class RobotLog:
    severity: int  # ROS_SEVERITY value
    logger: str  # the Log tag
    text: str


@dataclass(frozen=True)
class SignalRow:
    t_us: int
    names: tuple[str, ...]
    values: tuple

    @property
    def column_header(self) -> str:
        return ",".join(("t_us",) + self.names)

    @property
    def data(self) -> list[float]:
        return [float(self.t_us)] + [float(v) for v in self.values]


@dataclass(frozen=True)
class HealthRow:
    t_us: int
    names: tuple[str, ...]  # HEALTH_SIGNALS, in subscription order
    values: tuple


@dataclass(frozen=True)
class HealthEnded:
    reason: str


class RobotClock:
    """Maps the robot's micros() (32 bit, wraps) to ROS time, so that every sample is stamped
    with its own time and not with the time the bridge got round to reading it.

    The offset follows the lowest `now - t` seen: no stamp lies in the future, and it
    moves up slowly to follow clock drift. Stamps never go back."""

    _RELAX = 0.001  # share of the excess a later sample pulls the offset up by

    def __init__(self):
        self.rebase()

    def rebase(self) -> None:
        """The robot's clock started over (restart, new connection)."""
        self._last_us = None
        self._unwrapped_us = 0
        self._offset_ns = None
        self._last_ns = 0

    def stamp_ns(self, t_us: int, now_ns: int) -> int:
        if self._last_us is None:
            self._unwrapped_us = t_us
        else:
            step = (t_us - self._last_us) & 0xFFFFFFFF
            self._unwrapped_us += step if step < 2**31 else step - 2**32
        self._last_us = t_us
        sample_ns = now_ns - self._unwrapped_us * 1000
        if self._offset_ns is None or sample_ns < self._offset_ns:
            self._offset_ns = sample_ns
        else:
            self._offset_ns += round((sample_ns - self._offset_ns) * self._RELAX)
        self._last_ns = max(self._unwrapped_us * 1000 + self._offset_ns, self._last_ns + 1)
        return self._last_ns


class LinkBridge:
    """One connection: the Link, the pose integration and the firmware controls."""

    def __init__(self, link: Link, odometry: DiffDriveOdometry):
        self.link = link
        self.controls = FirmwareControls(link)
        self._odometry = odometry

    def poll(self) -> list:
        """Everything that arrived since the last call, in order: OdometryState, ImuSample,
        RobotLog, SignalRow, HealthRow, plus Restarted, SubscriptionEnded, HealthEnded, CommandRefused
        and Disconnected for the caller."""
        out = []
        while not self.link.events.empty():
            event = self.link.events.get_nowait()
            if isinstance(event, Telemetry):
                self._on_telemetry(event.message, out)
            elif isinstance(event, LogEvent):
                out.append(_robot_log(event))
            elif isinstance(event, Row):
                sub = self.controls.subscription
                health = self.controls.health
                if sub is not None and sub.slot == event.slot:
                    out.append(SignalRow(event.t_us, sub.names, event.values))
                elif health is not None and health.slot == event.slot:
                    out.append(HealthRow(event.t_us, health.names, event.values))
            elif isinstance(event, Restarted):
                self._odometry.rebase()  # keep the pose
                self.controls.subscription = None  # the robot's slots are gone
                self.controls.health = None
                out.append(event)
            elif isinstance(event, SubscriptionEnded):
                sub = self.controls.subscription
                if sub is not None and sub.slot == event.slot:
                    self.controls.subscription = None
                    out.append(event)
                elif self.controls.health is not None and self.controls.health.slot == event.slot:
                    self.controls.health = None
                    out.append(HealthEnded(event.reason))
            elif isinstance(event, CommandRefused):
                out.append(event)
            elif isinstance(event, Disconnected):
                out.append(event)
        return out

    def _on_telemetry(self, msg, out: list) -> None:
        if msg.WhichOneof("body") == "odom":
            state = self._odometry.update(msg.odom.t_us, msg.odom.left_angle_rad, msg.odom.right_angle_rad)
            if state is not None:
                out.append(state)
        else:
            self._odometry.add_gyro(msg.imu.t_us, msg.imu.gz)
            out.append(_imu_sample(msg.imu))


def _robot_log(event: LogEvent) -> RobotLog:
    text = f"{event.text} (code {event.code})" if event.code else event.text
    # An unnamed level (LEVEL_UNSPECIFIED, a number from newer firmware) is shown, not dropped.
    return RobotLog(ROS_SEVERITY.get(event.level, ROS_SEVERITY["INFO"]), event.tag or DEFAULT_LOGGER, text)


def _imu_sample(imu: navbot_link_pb2.Imu) -> ImuSample:
    return ImuSample(
        linear_acceleration=(imu.ax, imu.ay, imu.az),
        angular_velocity=(imu.gx, imu.gy, imu.gz),
        roll_rad=imu.roll_rad,
        pitch_rad=imu.pitch_rad,
        yaw_rad=imu.yaw_rad,
        t_us=imu.t_us,
    )
