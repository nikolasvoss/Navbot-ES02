from __future__ import annotations

import math
import threading
import time

import rclpy.time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from .localization import LocalizationTracker
from .motion_gate import MotionGate
from .mount import SENSOR_MOUNT_X_M, SENSOR_MOUNT_Y_M, SENSOR_MOUNT_YAW_RAD
from .runtime import LidarRuntime
from .slam import SlamState, grid_to_dict
from .source import SourceConfig
from .tilt_gate import TiltGate, config_from_mount, roll_from_quaternion

MAP_FRAME = "map"
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
SENSOR_FRAME = "lds08rr"

BIN_COUNT = 360
RANGE_MIN_M = 0.01
RANGE_MAX_M = 30.0

PLAN_MAX_POINTS = 200
SUPERVISOR_TICK_SEC = 0.2  # 5 Hz


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def with_ranges(message: LaserScan, ranges) -> LaserScan:
    """A copy of the scan with other ranges."""
    gated = LaserScan()
    gated.header = message.header
    gated.angle_min, gated.angle_max, gated.angle_increment = message.angle_min, message.angle_max, message.angle_increment
    gated.time_increment, gated.scan_time = message.time_increment, message.scan_time
    gated.range_min, gated.range_max = message.range_min, message.range_max
    gated.ranges = ranges
    gated.intensities = message.intensities
    return gated


def to_laserscan(scan, stamp) -> LaserScan:
    ranges = [math.inf] * BIN_COUNT
    intensities = [0.0] * BIN_COUNT
    for point in scan.points:
        distance = point.dist_mm / 1000.0
        if not point.valid or not math.isfinite(distance):
            continue
        if not RANGE_MIN_M <= distance <= RANGE_MAX_M:
            continue
        index = int(round(-point.angle_deg % 360.0)) % BIN_COUNT
        if distance < ranges[index]:
            ranges[index] = distance
            intensities[index] = float(point.quality)

    message = LaserScan()
    message.header.stamp = stamp
    message.header.frame_id = "lds08rr"
    message.angle_min = 0.0
    message.angle_max = math.radians(BIN_COUNT - 1)
    message.angle_increment = math.tau / BIN_COUNT
    message.time_increment = 0.0
    message.scan_time = 0.0 if scan.scan_freq_hz <= 0 else 1.0 / scan.scan_freq_hz
    message.range_min = RANGE_MIN_M
    message.range_max = RANGE_MAX_M
    message.ranges = ranges
    message.intensities = intensities
    return message


def sensor_mount_transform(stamp) -> TransformStamped:
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = BASE_FRAME
    transform.child_frame_id = SENSOR_FRAME
    transform.transform.translation.x = SENSOR_MOUNT_X_M
    transform.transform.translation.y = SENSOR_MOUNT_Y_M
    transform.transform.rotation.z = math.sin(SENSOR_MOUNT_YAW_RAD / 2.0)
    transform.transform.rotation.w = math.cos(SENSOR_MOUNT_YAW_RAD / 2.0)
    return transform


def firmware_values(statuses) -> dict[str, str] | None:
    """The KeyValues of the navbot_firmware status in a DiagnosticArray's status list, or None when it is absent."""
    for status in statuses:
        if status.name == "navbot_firmware":
            return {item.key: item.value for item in status.values}
    return None


def reduce_plan(points: list[list[float]], limit: int = PLAN_MAX_POINTS) -> list[list[float]]:
    """Every k-th pose, then the last pose if the stride missed it, so that at most limit points remain."""
    if len(points) <= limit:
        return [list(point) for point in points]
    step = -(-(len(points) - 1) // (limit - 1))  # ceiling division
    reduced = [list(point) for point in points[::step]]
    if (len(points) - 1) % step:
        reduced.append(list(points[-1]))
    return reduced


def nav_outcome(status: int, error_code: int, error_msg: str) -> tuple[str, str]:
    """The outcome of a finished NavigateToPose goal. A failed goal gets Nav2's error text, else its error code."""
    if status == GoalStatus.STATUS_SUCCEEDED:
        return "reached", ""
    if status == GoalStatus.STATUS_CANCELED:
        return "canceled", ""
    if error_msg:
        return "failed", error_msg
    if error_code:
        return "failed", f"Nav2 error code {error_code}"
    return "failed", "Nav2 aborted the goal"


class _ActiveGoal:
    """A goal sent to Nav2. handle is set once Nav2 accepts it; a cancel asked before that is sent on acceptance."""

    def __init__(self) -> None:
        self.handle = None
        self.cancel_requested = False
        self.finish = None  # finish(outcome, error), set by send_nav_goal


class LidarNode(Node):
    def __init__(
        self,
        config: SourceConfig,
        localization: LocalizationTracker,
        supervisor=None,
    ) -> None:
        super().__init__("lds08rr_lidar")
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.scan_publisher = self.create_publisher(LaserScan, "/scan", scan_qos)
        # /scan stays raw; slam_toolbox reads the tilt- and motion-gated /scan_slam, because mapping keeps every scan for good.
        # AMCL and Nav2 read /scan_nav, which only loses floor hits: the gates drop whole scans in exactly the moves Nav2
        # commands, and AMCL's map -> odom would expire and the collision monitor and supervisor stop for want of scans.
        self.slam_scan_publisher = self.create_publisher(LaserScan, "/scan_slam", scan_qos)
        self.nav_scan_publisher = self.create_publisher(LaserScan, "/scan_nav", scan_qos)
        self.tilt_gate = TiltGate(config_from_mount())
        self.motion_gate = MotionGate()
        self._logged_gate_state: str | None = None
        self._logged_motion_state: str | None = None
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.status_publisher = self.create_publisher(DiagnosticArray, "/lidar/status", 10)
        self.runtime = LidarRuntime(config, on_scan=self.publish_scan)
        self._ros_publish_error: str | None = None
        self.create_timer(1.0, self.publish_status)

        # slam_toolbox publishes /map transient-local; the web page reads it from here.
        self.slam = SlamState()
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.localization = localization
        amcl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl_pose, amcl_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # odom -> base_link comes from navbot_link_bridge (ESP32 wheel odometry); only the sensor mount is ours.
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(sensor_mount_transform(self.get_clock().now().to_msg()))
        self.create_timer(0.5, self.update_pose)
        self._serialize_client = None
        self._service_lock = threading.Lock()
        self._service_clients: dict[str, object] = {}
        self._last_seen: dict[str, float] = {}
        self.supervisor = supervisor
        self._plan: list[list[float]] = []
        self._nav_goal: _ActiveGoal | None = None
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        if supervisor is not None:
            self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diagnostics, 10)
            self.create_subscription(BatteryState, "/battery_state", self.on_battery, 10)
            self.create_subscription(LaserScan, "/scan_nav", self.on_scan_nav, qos_profile_sensor_data)
            self.create_subscription(Path, "plan", self.on_plan, 10)
            self.create_timer(SUPERVISOR_TICK_SEC, self.on_supervisor_tick)

    def on_diagnostics(self, message: DiagnosticArray) -> None:
        values = firmware_values(message.status)
        if values is not None:
            self.supervisor.set_health(values)

    def on_battery(self, message: BatteryState) -> None:
        self.supervisor.set_voltage(message.voltage)

    def on_scan_nav(self, message: LaserScan) -> None:
        self.supervisor.scan_seen()

    def on_plan(self, message: Path) -> None:
        self._plan = reduce_plan([[pose.pose.position.x, pose.pose.position.y] for pose in message.poses])

    def on_supervisor_tick(self) -> None:
        self.supervisor.set_localization(self.localization.snapshot()["state"])
        self.supervisor.set_nav_ready(self.nav_server_ready())
        if self.supervisor.check():
            self.cancel_nav_goal()

    def on_map(self, message: OccupancyGrid) -> None:
        info = message.info
        self.slam.set_grid(grid_to_dict(
            info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y, message.data,
        ))

    def topic_age(self, topic: str) -> float | None:
        seen = self._last_seen.get(topic)
        return None if seen is None else time.monotonic() - seen

    def publisher_count(self, topic: str) -> int:
        return self.count_publishers(topic)

    def node_exists(self, name: str) -> bool:
        return name in self.get_node_names()

    def on_odom(self, message: Odometry) -> None:
        self._last_seen["/odom"] = time.monotonic()
        p, twist = message.pose.pose, message.twist.twist
        self.slam.set_odom(p.position.x, p.position.y, yaw_from_quaternion(p.orientation), twist.linear.x, twist.angular.z)
        stamp = rclpy.time.Time.from_msg(message.header.stamp).nanoseconds / 1e9
        self.motion_gate.add_odom(stamp, twist.linear.x, twist.angular.z)

    def on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        p, covariance = message.pose.pose, message.pose.covariance
        self.localization.add_pose(
            p.position.x, p.position.y, yaw_from_quaternion(p.orientation),
            covariance[0], covariance[7], covariance[35],
        )

    def update_pose(self) -> None:
        try:
            correction = self.tf_buffer.lookup_transform(MAP_FRAME, ODOM_FRAME, rclpy.time.Time()).transform
            self.slam.set_correction(correction.translation.x, correction.translation.y, yaw_from_quaternion(correction.rotation))
        except TransformException:
            pass
        try:
            transform = self.tf_buffer.lookup_transform(MAP_FRAME, BASE_FRAME, rclpy.time.Time())
        except TransformException:
            return
        t, q = transform.transform.translation, transform.transform.rotation
        yaw = yaw_from_quaternion(q)
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
        self.slam.set_pose(t.x, t.y, yaw, age_sec=max(age, 0.0))

    def call_service(self, name: str, timeout: float) -> str | None:
        """Calls a std_srvs/Empty service from a non-ROS thread; None on success, else the error text."""
        from std_srvs.srv import Empty

        with self._service_lock:
            client = self._service_clients.get(name)
            if client is None:
                client = self.create_client(Empty, name)
                self._service_clients[name] = client
        if not client.wait_for_service(timeout_sec=timeout):
            return "service unavailable"
        # Called from the localizer thread while the main thread spins the node.
        finished = threading.Event()
        future = client.call_async(Empty.Request())
        future.add_done_callback(lambda _: finished.set())
        if not finished.wait(timeout):
            client.remove_pending_request(future)
            return "call timed out"
        if future.exception() is not None:
            return f"call failed: {future.exception()}"
        return None

    def nav_server_ready(self) -> bool:
        return self._nav_client.server_is_ready()

    def send_nav_goal(self, x: float, y: float, yaw: float, on_feedback, on_done) -> str | None:
        """NavigateToPose in map; on_feedback(distance_remaining), on_done(outcome, error)."""
        if not self._nav_client.server_is_ready():
            return "Nav2 is not ready"
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = MAP_FRAME
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Callbacks run on the ROS thread. A goal that Nav2 replaced reports nothing more.
        goal = _ActiveGoal()
        self._nav_goal = goal

        def finish(outcome: str, error: str) -> None:
            if self._nav_goal is goal:
                self._nav_goal = None
                on_done(outcome, error)

        goal.finish = finish

        def on_feedback_message(message) -> None:
            if self._nav_goal is goal:
                on_feedback(message.feedback.distance_remaining)

        def on_result(future) -> None:
            if future.exception() is not None:
                finish("failed", str(future.exception()))
                return
            result = future.result()
            finish(*nav_outcome(result.status, result.result.error_code, result.result.error_msg))

        def on_goal_response(future) -> None:
            if future.exception() is not None:
                finish("failed", str(future.exception()))
                return
            handle = future.result()
            if not handle.accepted:
                finish("failed", "goal rejected by Nav2")
                return
            goal.handle = handle
            if goal.cancel_requested:
                handle.cancel_goal_async()
            handle.get_result_async().add_done_callback(on_result)

        self._nav_client.send_goal_async(goal_msg, feedback_callback=on_feedback_message).add_done_callback(on_goal_response)
        return None

    def cancel_nav_goal(self) -> None:
        goal = self._nav_goal
        if goal is None:
            return
        goal.cancel_requested = True
        if not self._nav_client.server_is_ready():
            # A Nav2 that died or restarted never reports this goal again; end it here so it does not stay running.
            goal.finish("canceled", "Nav2 is not running")
            return
        if goal.handle is not None:
            goal.handle.cancel_goal_async()

    def nav_plan(self) -> list[list[float]]:
        """At most 200 points of the newest plan, the last pose included."""
        return self._plan

    def serialize_map(self, filename: str, timeout: float = 15.0) -> str | None:
        try:
            from slam_toolbox.srv import SerializePoseGraph
        except ImportError as exc:
            return f"slam_toolbox is unavailable: {exc}"
        if self._serialize_client is None:
            self._serialize_client = self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")
        if not self._serialize_client.wait_for_service(timeout_sec=2.0):
            return "slam_toolbox serialize service unavailable"
        request = SerializePoseGraph.Request()
        request.filename = filename
        # Called from the HTTP thread while the main thread spins the node.
        finished = threading.Event()
        future = self._serialize_client.call_async(request)
        future.add_done_callback(lambda _: finished.set())
        if not finished.wait(timeout):
            return "serialize_map timed out"
        if future.exception() is not None:
            return f"serialize_map failed: {future.exception()}"
        result = future.result().result
        if result != 0:
            return f"serialize_map failed with result {result}"
        return None

    def publish_scan(self, scan, completed_at: float) -> None:
        stamp = self.get_clock().now()
        if scan.scan_freq_hz > 0:
            scan_start = stamp.nanoseconds - int(1e9 / scan.scan_freq_hz)
            sec, nsec = divmod(scan_start, 1_000_000_000)
            ros_stamp = Time(sec=int(sec), nanosec=int(nsec))
        else:
            ros_stamp = stamp.to_msg()
        try:
            message = to_laserscan(scan, ros_stamp)
            self.scan_publisher.publish(message)
            self.publish_slam_scan(message)
        except Exception as exc:
            self._ros_publish_error = f"{type(exc).__name__}: {exc}"
        else:
            self._ros_publish_error = None

    def on_imu(self, message: Imu) -> None:
        self._last_seen["/imu/data"] = time.monotonic()
        q = message.orientation
        stamp = rclpy.time.Time.from_msg(message.header.stamp).nanoseconds / 1e9
        self.tilt_gate.add_imu(stamp, roll_from_quaternion(q.x, q.y, q.z, q.w))

    def publish_slam_scan(self, message: LaserScan) -> None:
        start = rclpy.time.Time.from_msg(message.header.stamp).nanoseconds / 1e9
        ranges = self.tilt_gate.apply(message.ranges, message.angle_min, message.angle_increment, start, message.scan_time)
        keep_for_motion = self.motion_gate.apply(start, message.scan_time)
        if self.motion_gate.state != self._logged_motion_state:
            self._logged_motion_state = self.motion_gate.state
            self.get_logger().info({
                "no_odom": "motion gate paused: no /odom covers the scan; scans kept",
                "ok": "motion gate active on /scan_slam",
                "limiting": "motion gate dropping scans: " + str(self.motion_gate.snapshot()["reason"]),
            }[self._logged_motion_state])
        if self.tilt_gate.state != self._logged_gate_state:
            self._logged_gate_state = self.tilt_gate.state
            text = {
                "unconfigured": "tilt gating off: mount.py lacks scan height, IMU roll sign or level roll; /scan_slam = /scan",
                "no_imu": "tilt gating paused: no fresh /imu/data covers the scan; /scan_slam = /scan",
                "active": "tilt gating active on /scan_slam",
            }[self._logged_gate_state]
            self.get_logger().info(text)
        nav_ranges = self.tilt_gate.apply_floor_only(message.ranges, message.angle_min, message.angle_increment, start, message.scan_time)
        self.nav_scan_publisher.publish(with_ranges(message, nav_ranges))
        if ranges is None or not keep_for_motion:
            return
        self.slam_scan_publisher.publish(with_ranges(message, ranges))

    def publish_status(self) -> None:
        status = self.runtime.snapshot()
        diagnostic = DiagnosticStatus()
        diagnostic.name = "lds08rr"
        diagnostic.hardware_id = "LDS08RR"
        diagnostic.level = {
            "live": DiagnosticStatus.OK,
            "demo": DiagnosticStatus.WARN,
            "waiting": DiagnosticStatus.WARN,
            "invalid": DiagnosticStatus.WARN,
            "stale": DiagnosticStatus.WARN,
            "error": DiagnosticStatus.ERROR,
        }[status.state]
        diagnostic.message = status.state
        fields = {
            "mode": status.mode,
            "device": status.device or "",
            "protocol": status.protocol or "",
            "bytes_received": str(status.bytes_received),
            "scans_received": str(status.scans_received),
            "last_scan_age_sec": "" if status.last_scan_age_sec is None else f"{status.last_scan_age_sec:.3f}",
            "scan_rate_hz": "" if status.scan_rate_hz is None else f"{status.scan_rate_hz:.3f}",
            "valid_points": str(status.valid_points),
            "last_error": status.last_error or "",
            "ros_publish_error": self._ros_publish_error or "",
            "orientation": "native and uncalibrated",
            "stamp": "estimated scan start from receive completion and reported rate",
        }
        diagnostic.values = [KeyValue(key=key, value=value) for key, value in fields.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [diagnostic]
        self.status_publisher.publish(array)
