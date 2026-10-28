"""rclpy node: reads the NavBot link through navbot_link.Link and publishes odometry, TF, IMU
and the robot's log lines."""

import math
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import rclpy
import serial
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.exceptions import ParameterException
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log, ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from tf2_ros import TransformBroadcaster

from navbot_link.link import Disconnected, Link, Restarted, SubscriptionEnded
from navbot_link.protocol import LinkError
from navbot_link_bridge.bridge import ImuSample, LinkBridge, RobotClock, RobotLog, SignalRow
from navbot_link_bridge.odometry import DiffDriveOdometry, OdometryState, gyro_yaw_sign_for

POLL_HZ = 200
WARN_INTERVAL_S = 5.0
REOPEN_INTERVAL_S = 1.0
# The profile rcl uses for /rosout, which rclpy does not export.
ROSOUT_QOS = QoSProfile(
    depth=1000,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    lifespan=Duration(seconds=10),
)


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


def _open_link(port: str, baud: int) -> Link:
    return Link(port, baud, telemetry=True)


class LinkBridgeNode(Node):
    def __init__(self, open_link=_open_link, **node_args):
        super().__init__("navbot_link_bridge", **node_args)
        self._open_link = open_link
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
        # Dynamic typing: rclpy reads an empty list as a byte array, and `signals: []` must clear the stream.
        self.declare_parameter("signals", [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter("signals_rate_hz", 10.0)

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
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self._imu_pub = self.create_publisher(Imu, "imu/data", 10)
        self._tf = TransformBroadcaster(self)
        # rcl publishes a logger's lines to /rosout only for node loggers, so the
        # robot's lines (logger name = Log tag) go out as Log messages directly.
        self._rosout_pub = self.create_publisher(Log, "/rosout", ROSOUT_QOS)
        self._signals_pub = self.create_publisher(Float64MultiArray, "/navbot/signals", 1000)

        self._bridge = None
        self._opened_before = False
        self._next_open_s = 0.0
        self._last_warn_s = -WARN_INTERVAL_S
        self._warned_counts = (0, 0)
        self._last_fallback_warn_s = -WARN_INTERVAL_S
        self._warned_fallbacks = 0
        self._firmware_params: set[str] = set()
        self._local = threading.local()  # .mirroring: this thread copies a robot value into a ROS parameter
        self._warned_no_catalog = False
        self._robot_clock = RobotClock()
        # Anything that waits for the robot (catalog, parameters, subscriptions) runs here, never in the
        # poll timer. One thread keeps those exchanges in order.
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="navbot-link-control")
        self.attached = threading.Event()  # set when the latest attach job has finished, with or without a catalog
        self._robot_lock = threading.RLock()  # serializes parameter sets and attach jobs against each other
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.add_post_set_parameters_callback(self._after_set_parameters)
        self._connect()
        self.create_timer(1.0 / POLL_HZ, self._poll_once, callback_group=MutuallyExclusiveCallbackGroup())

    @property
    def _mirroring(self) -> bool:
        return getattr(self._local, "mirroring", False)

    @_mirroring.setter
    def _mirroring(self, value: bool) -> None:
        self._local.mirroring = value

    def _connect(self) -> None:
        try:
            link = self._open_link(self._port, self._baud)
        except (serial.SerialException, OSError) as exc:
            self._next_open_s = time.monotonic() + REOPEN_INTERVAL_S
            self.get_logger().error(f"cannot open {self._port}: {exc}; retrying every 1 s")
            return
        self.get_logger().info(f"opened {self._port} at {self._baud} baud")
        if self._opened_before:
            self._odometry.rebase()  # wheel angles may have jumped while the port was closed
        self._opened_before = True
        self._bridge = LinkBridge(link, self._odometry)
        self._robot_clock.rebase()
        self._warned_counts = (0, 0)
        self.attached.clear()
        self._submit(self._attach, self._bridge, False)

    def _submit(self, job, bridge, *args) -> None:
        def run():
            try:
                with self._robot_lock:
                    if self._bridge is bridge:  # not closed or replaced since
                        job(bridge, *args)
            except Exception as exc:  # noqa: BLE001 - a failed job must not vanish with its future
                self.get_logger().error(f"{job.__name__} failed: {type(exc).__name__}: {exc}")

        self._worker.submit(run)

    def _attach(self, bridge: LinkBridge, restarted: bool) -> None:
        """Read the catalog, expose its parameters and start the configured signal stream."""
        controls = bridge.controls
        if restarted:
            self.attached.clear()
            controls.restarted()
        try:
            self._attach_catalog(bridge)
        finally:
            self.attached.set()

    def _attach_catalog(self, bridge: LinkBridge) -> None:
        controls = bridge.controls
        if not controls.attach():
            self._forget_firmware_parameters()  # no catalog, no firmware parameters
            if not self._warned_no_catalog:
                self._warned_no_catalog = True
                self.get_logger().warning(
                    "the robot did not answer GetCatalog (firmware without the signal table?); "
                    "continuing with odometry and IMU only, no firmware parameters or signals"
                )
            return
        self._warned_no_catalog = False
        try:
            self._expose_firmware_parameters(bridge)
        except LinkError as exc:
            self.get_logger().warning(f"reading firmware parameters failed: {exc}")
        self._subscribe_configured_signals(bridge)

    def _forget_firmware_parameters(self) -> None:
        for name in self._firmware_params:
            self.undeclare_parameter(name)
        self._firmware_params.clear()

    def _expose_firmware_parameters(self, bridge: LinkBridge) -> None:
        self._forget_firmware_parameters()
        for info in bridge.controls.params():
            if self.has_parameter(info.name):
                self.get_logger().warning(f"firmware parameter {info.name} clashes with a bridge parameter; not exposed")
                continue
            if not self._declare_firmware_parameter(info.name, info.value):
                continue
            self._firmware_params.add(info.name)
            wanted = self.get_parameter(info.name).value
            if type(wanted) is type(info.value) and wanted == info.value:
                continue
            # A launch-file or command-line value for a firmware parameter, sent once the catalog is read.
            reason, applied = bridge.controls.apply_param(info.name, wanted)
            if reason:
                self.get_logger().warning(f"startup value of {info.name} not applied: {reason}")
                applied = info.value
            self._mirror(info.name, applied)

    def _declare_firmware_parameter(self, name: str, value) -> bool:
        """Dynamically typed, so that an integer startup value for an f32 parameter is accepted and
        stored as the float the robot takes (rclpy would refuse it against a double). False and a warning
        when rclpy refuses the name."""
        self._mirroring = True
        try:
            self.declare_parameter(name, value, ParameterDescriptor(dynamic_typing=True))
            return True
        except ParameterException as exc:
            self.get_logger().warning(f"firmware parameter {name!r} not exposed: {str(exc).strip()}")
            return False
        finally:
            self._mirroring = False

    def _mirror(self, name: str, value) -> None:
        self._mirroring = True
        try:
            self.set_parameters([Parameter(name, value=value)])
        except ParameterException as exc:
            self.get_logger().warning(f"{name} not stored as {value!r}: {str(exc).strip()}")
        finally:
            self._mirroring = False

    def _subscribe_configured_signals(self, bridge: LinkBridge) -> None:
        reason = bridge.controls.set_subscription(
            list(self.get_parameter("signals").value), self.get_parameter("signals_rate_hz").value
        )
        if reason:
            self.get_logger().warning(f"signal stream not started: {reason}")

    def _on_set_parameters(self, params) -> SetParametersResult:
        if self._mirroring:
            return SetParametersResult(successful=True)
        changed = {p.name: p.value for p in params}
        wanted = [name for name in changed if name in self._firmware_params]
        streaming = "signals" in changed or "signals_rate_hz" in changed
        if not (wanted or streaming):
            return SetParametersResult(successful=True)
        self._local.kept = {}
        with self._robot_lock:
            bridge = self._bridge
            if bridge is None:
                return SetParametersResult(successful=False, reason="the link to the robot is down")
            reason = self._apply_to_robot(bridge.controls, changed, wanted, streaming)
        return SetParametersResult(successful=not reason, reason=reason)

    def _apply_to_robot(self, controls, changed: dict, wanted: list, streaming: bool) -> str:
        """All or nothing: when the robot refuses one value, those it took before are put back.
        The values it keeps go to self._local.kept for _after_set_parameters."""
        try:
            before = {name: controls.get_param(name) for name in wanted} if len(wanted) + streaming > 1 else {}
        except LinkError as exc:
            return str(exc)
        kept = {}
        reason = ""
        for name in wanted:
            reason, kept[name] = controls.apply_param(name, changed[name])
            if reason:
                break
        if not reason and streaming:
            reason = controls.set_subscription(
                list(changed.get("signals", self.get_parameter("signals").value)),
                changed.get("signals_rate_hz", self.get_parameter("signals_rate_hz").value),
            )
        if reason:
            for name in [n for n in before if n in kept and kept[n] is not None]:
                failure = controls.set_param(name, before[name])
                if failure:
                    reason += f"; could not restore {name}: {failure}"
            return reason
        self._local.kept = kept
        return ""

    def _after_set_parameters(self, params) -> None:
        """ROS has stored the requested values. Where the robot kept another (a clamp, float32 rounding), ROS takes
        the robot's."""
        kept, self._local.kept = getattr(self._local, "kept", {}), {}
        for param in params:
            if param.name in kept and (kept[param.name] != param.value or type(kept[param.name]) is not type(param.value)):
                self._mirror(param.name, kept[param.name])

    def _poll_once(self) -> None:
        if self._bridge is None:
            if time.monotonic() >= self._next_open_s:
                self._connect()
            return
        bridge = self._bridge
        for result in bridge.poll():
            if isinstance(result, OdometryState):
                self._publish_odometry(result, self._robot_stamp(result.t_us))
            elif isinstance(result, ImuSample):
                self._publish_imu(result, self._robot_stamp(result.t_us))
            elif isinstance(result, RobotLog):
                self._publish_robot_log(result, self.get_clock().now().to_msg())
            elif isinstance(result, SignalRow):
                self._publish_signals(result)
            elif isinstance(result, Restarted):
                self.get_logger().info("robot restart detected (seq reset); pose kept, odometry rebased")
                self._robot_clock.rebase()
                self._submit(self._attach, bridge, True)  # table indices and slots may differ in the new build
            elif isinstance(result, SubscriptionEnded):
                self.get_logger().warning(f"signal stream ended: {result.reason}; resubscribing")
                self._submit(self._subscribe_configured_signals, bridge)
            elif isinstance(result, Disconnected):
                self.get_logger().error(f"link failed: {result.reason}; reopening")
                self.close()
                self._next_open_s = time.monotonic() + REOPEN_INTERVAL_S
                return
        self._report_link_health()

    def _robot_stamp(self, t_us: int):
        return Time(nanoseconds=self._robot_clock.stamp_ns(t_us, self.get_clock().now().nanoseconds)).to_msg()

    def _publish_robot_log(self, line: RobotLog, stamp) -> None:
        msg = Log()
        msg.stamp = stamp
        msg.level = line.severity
        msg.name = line.logger
        msg.msg = line.text
        self._rosout_pub.publish(msg)

    def _publish_signals(self, row: SignalRow) -> None:
        # One dimension whose label is the column header (as in the navlink CSV): data[i] belongs to label i.
        msg = Float64MultiArray()
        msg.layout.dim = [MultiArrayDimension(label=row.column_header, size=len(row.data), stride=len(row.data))]
        msg.data = row.data
        self._signals_pub.publish(msg)

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
        counts = (bridge.link.bad_frames, bridge.link.lost_frames)
        now = time.monotonic()
        if counts != self._warned_counts and now - self._last_warn_s >= WARN_INTERVAL_S:
            self.get_logger().warning(
                f"link frames: bad_frames={counts[0]} lost_frames={counts[1]}"
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
        if self._bridge is not None:
            self._bridge.link.close()
            self._bridge = None

    def destroy_node(self) -> None:
        self._worker.shutdown(wait=False, cancel_futures=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LinkBridgeNode()
    executor = MultiThreadedExecutor(num_threads=3)  # the poll timer must not wait for a parameter set
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
