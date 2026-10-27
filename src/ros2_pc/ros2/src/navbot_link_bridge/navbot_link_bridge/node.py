"""rclpy node: polls the NavBot link serial port and publishes odometry, TF and IMU."""

import math
import time

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from navbot_link_bridge.bridge import ImuSample, LinkBridge
from navbot_link_bridge.odometry import DiffDriveOdometry, OdometryState, gyro_yaw_sign_for

POLL_HZ = 200
WARN_INTERVAL_S = 5.0
REOPEN_INTERVAL_S = 1.0


def _quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) for intrinsic ZYX (yaw, pitch, roll), the ROS REP-103 convention."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _set_quaternion(target, quat: tuple[float, float, float, float]) -> None:
    target.x, target.y, target.z, target.w = quat


class LinkBridgeNode(Node):
    def __init__(self):
        super().__init__("navbot_link_bridge")
        self.declare_parameter("port", "")
        self.declare_parameter("baud", 2000000)
        self.declare_parameter("wheel_radius_m", 0.0)
        self.declare_parameter("wheel_separation_m", 0.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("heading_source", "wheels")
        self.declare_parameter("gyro_yaw_sign", 0)

        self._port = self.get_parameter("port").value
        if not self._port:
            self.get_logger().fatal("parameter 'port' is required, e.g. -p port:=/dev/ttyXXX")
            raise SystemExit(2)
        self._baud = self.get_parameter("baud").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._imu_frame = self.get_parameter("imu_frame").value
        self._publish_tf = self.get_parameter("publish_tf").value

        try:
            gyro_yaw_sign = gyro_yaw_sign_for(
                self.get_parameter("heading_source").value, self.get_parameter("gyro_yaw_sign").value
            )
        except ValueError as exc:
            self.get_logger().fatal(f"{exc}, e.g. -p heading_source:=gyro -p gyro_yaw_sign:=1")
            raise SystemExit(2)
        self._odometry = DiffDriveOdometry(
            self.get_parameter("wheel_radius_m").value,
            self.get_parameter("wheel_separation_m").value,
            gyro_yaw_sign,
        )
        self._bridge = LinkBridge(self._odometry)
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self._imu_pub = self.create_publisher(Imu, "imu/data", 10)
        self._tf = TransformBroadcaster(self)

        self._ser = None
        self._next_open_s = 0.0
        self._try_open()

        self._last_warn_s = -WARN_INTERVAL_S
        self._warned_counts = (0, 0, 0)
        self._last_fallback_warn_s = -WARN_INTERVAL_S
        self._warned_fallbacks = 0
        self._restarts_logged = 0
        self.create_timer(1.0 / POLL_HZ, self._on_timer)

    def _try_open(self) -> None:
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=0)
            self.get_logger().info(f"opened {self._port} at {self._baud} baud")
        except (serial.SerialException, OSError) as exc:
            self._ser = None
            self._next_open_s = time.monotonic() + REOPEN_INTERVAL_S
            self.get_logger().error(f"cannot open {self._port}: {exc}; retrying every 1 s")

    def _on_timer(self) -> None:
        if self._ser is None:
            if time.monotonic() >= self._next_open_s:
                self._try_open()
            return
        try:
            waiting = self._ser.in_waiting
            chunk = self._ser.read(waiting) if waiting else b""
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f"serial read failed: {exc}; reopening")
            self.close()
            self._next_open_s = time.monotonic() + REOPEN_INTERVAL_S
            return
        if chunk:
            stamp = self.get_clock().now().to_msg()
            for result in self._bridge.feed(chunk):
                if isinstance(result, OdometryState):
                    self._publish_odometry(result, stamp)
                elif isinstance(result, ImuSample):
                    self._publish_imu(result, stamp)
        self._report_link_health()

    def _publish_odometry(self, state: OdometryState, stamp) -> None:
        quat = _quaternion(0.0, 0.0, state.yaw_rad)
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = state.x_m
        msg.pose.pose.position.y = state.y_m
        _set_quaternion(msg.pose.pose.orientation, quat)
        msg.twist.twist.linear.x = state.linear_mps
        msg.twist.twist.angular.z = state.angular_rad_s
        self._odom_pub.publish(msg)

        if self._publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = state.x_m
            tf.transform.translation.y = state.y_m
            _set_quaternion(tf.transform.rotation, quat)
            self._tf.sendTransform(tf)

    def _publish_imu(self, sample: ImuSample, stamp) -> None:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self._imu_frame
        _set_quaternion(msg.orientation, _quaternion(sample.roll_rad, sample.pitch_rad, sample.yaw_rad))
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = sample.angular_velocity
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
            sample.linear_acceleration
        )
        self._imu_pub.publish(msg)

    def _report_link_health(self) -> None:
        bridge = self._bridge
        if bridge.restarts > self._restarts_logged:
            self.get_logger().info("robot restart detected (seq reset); pose kept, odometry rebased")
            self._restarts_logged = bridge.restarts

        counts = (bridge.bad_frames, bridge.decode_errors, bridge.lost_frames)
        now = time.monotonic()
        if counts != self._warned_counts and now - self._last_warn_s >= WARN_INTERVAL_S:
            self.get_logger().warning(
                f"link frames: bad_frames={counts[0]} decode_errors={counts[1]} lost_frames={counts[2]}"
            )
            self._warned_counts = counts
            self._last_warn_s = now

        fallbacks = self._odometry.gyro_fallbacks
        if fallbacks != self._warned_fallbacks and now - self._last_fallback_warn_s >= WARN_INTERVAL_S:
            self.get_logger().warning(
                f"gyro heading: {fallbacks} odometry steps used wheel yaw (no fresh IMU sample)"
            )
            self._warned_fallbacks = fallbacks
            self._last_fallback_warn_s = now

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None


def main(args=None):
    rclpy.init(args=args)
    node = LinkBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
