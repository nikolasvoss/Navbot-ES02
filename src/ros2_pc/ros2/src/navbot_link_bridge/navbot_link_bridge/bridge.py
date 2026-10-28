"""Turns Link events into odometry states, IMU samples, robot log lines and signal rows.
No ROS imports."""

from dataclasses import dataclass

from navbot_link import navbot_link_pb2
from navbot_link.link import (
    Disconnected, Link, LogEvent, Restarted, Row, SubscriptionEnded, Telemetry,
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


class LinkBridge:
    """One connection: the Link, the pose integration and the firmware controls."""

    def __init__(self, link: Link, odometry: DiffDriveOdometry):
        self.link = link
        self.controls = FirmwareControls(link)
        self._odometry = odometry

    def poll(self) -> list:
        """Everything that arrived since the last call, in order: OdometryState, ImuSample,
        RobotLog, SignalRow, plus Restarted, SubscriptionEnded and Disconnected for the caller."""
        out = []
        while not self.link.events.empty():
            event = self.link.events.get_nowait()
            if isinstance(event, Telemetry):
                self._on_telemetry(event.message, out)
            elif isinstance(event, LogEvent):
                out.append(_robot_log(event))
            elif isinstance(event, Row):
                sub = self.controls.subscription
                if sub is not None and sub.slot == event.slot:
                    out.append(SignalRow(event.t_us, sub.names, event.values))
            elif isinstance(event, Restarted):
                self._odometry.rebase()  # keep the pose
                self.controls.subscription = None  # the robot's slots are gone
                out.append(event)
            elif isinstance(event, SubscriptionEnded):
                sub = self.controls.subscription
                if sub is not None and sub.slot == event.slot:
                    self.controls.subscription = None
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
