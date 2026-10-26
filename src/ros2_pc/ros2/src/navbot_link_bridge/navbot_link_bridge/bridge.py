"""Turns link bytes into odometry states and IMU samples. No ROS imports."""

from dataclasses import dataclass

from google.protobuf.message import DecodeError

from navbot_link_bridge import navbot_link_pb2
from navbot_link_bridge.framing import FrameDecoder
from navbot_link_bridge.odometry import DiffDriveOdometry, OdometryState


@dataclass(frozen=True)
class ImuSample:
    linear_acceleration: tuple[float, float, float]  # m/s^2
    angular_velocity: tuple[float, float, float]  # rad/s
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    t_us: int  # robot micros() at the IMU read; use only differences (wraps at 2**32)


class LinkBridge:
    def __init__(self, odometry: DiffDriveOdometry):
        self._odometry = odometry
        self._decoder = FrameDecoder()
        self._prev_seq = None
        self.decode_errors = 0
        self.lost_frames = 0
        self.restarts = 0

    @property
    def bad_frames(self) -> int:
        return self._decoder.bad_frames

    def feed(self, chunk: bytes) -> list[OdometryState | ImuSample]:
        results = []
        for payload in self._decoder.feed(chunk):
            msg = navbot_link_pb2.Message()
            try:
                msg.ParseFromString(payload)
            except DecodeError:
                self.decode_errors += 1
                continue
            self._track_seq(msg.seq)

            body = msg.WhichOneof("body")
            if body == "odom":
                state = self._odometry.update(
                    msg.odom.t_us, msg.odom.left_angle_rad, msg.odom.right_angle_rad
                )
                if state is not None:
                    results.append(state)
            elif body == "imu":
                self._odometry.add_gyro(msg.imu.t_us, msg.imu.gz)
                results.append(_imu_sample(msg.imu))
        return results

    def _track_seq(self, seq: int) -> None:
        # A sender counts from 1 and a uint32 would take years to wrap, so a
        # non-increasing seq means the robot restarted: start a new baseline
        # and keep the pose.
        if self._prev_seq is not None:
            if seq <= self._prev_seq:
                self.restarts += 1
                self._odometry.rebase()
            elif seq - self._prev_seq > 1:
                self.lost_frames += seq - self._prev_seq - 1
        self._prev_seq = seq


def _imu_sample(imu) -> ImuSample:
    return ImuSample(
        linear_acceleration=(imu.ax, imu.ay, imu.az),
        angular_velocity=(imu.gx, imu.gy, imu.gz),
        roll_rad=imu.roll_rad,
        pitch_rad=imu.pitch_rad,
        yaw_rad=imu.yaw_rad,
        t_us=imu.t_us,
    )
