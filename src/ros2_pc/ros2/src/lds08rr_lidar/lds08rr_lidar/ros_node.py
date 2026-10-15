from __future__ import annotations

import math

import rclpy.time

from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .runtime import LidarRuntime
from .slam import SlamState, grid_to_dict
from .source import SourceConfig

MAP_FRAME = "map"
SENSOR_FRAME = "lds08rr"

BIN_COUNT = 360
RANGE_MIN_M = 0.01
RANGE_MAX_M = 30.0


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


class LidarNode(Node):
    def __init__(self, config: SourceConfig) -> None:
        super().__init__("lds08rr_lidar")
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.scan_publisher = self.create_publisher(LaserScan, "/scan", scan_qos)
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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.5, self.update_pose)

    def on_map(self, message: OccupancyGrid) -> None:
        info = message.info
        self.slam.set_grid(grid_to_dict(
            info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y, message.data,
        ))

    def update_pose(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(MAP_FRAME, SENSOR_FRAME, rclpy.time.Time())
        except TransformException:
            return
        t, q = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.slam.set_pose(t.x, t.y, yaw)

    def publish_scan(self, scan, completed_at: float) -> None:
        stamp = self.get_clock().now()
        if scan.scan_freq_hz > 0:
            scan_start = stamp.nanoseconds - int(1e9 / scan.scan_freq_hz)
            sec, nsec = divmod(scan_start, 1_000_000_000)
            ros_stamp = Time(sec=int(sec), nanosec=int(nsec))
        else:
            ros_stamp = stamp.to_msg()
        try:
            self.scan_publisher.publish(to_laserscan(scan, ros_stamp))
        except Exception as exc:
            self._ros_publish_error = f"{type(exc).__name__}: {exc}"
        else:
            self._ros_publish_error = None

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
