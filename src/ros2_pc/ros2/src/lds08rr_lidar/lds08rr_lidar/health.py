from __future__ import annotations

import socket
from dataclasses import asdict, dataclass

from .slam import MAP_STALE_AFTER_SEC

ODOM_TOPIC = "/odom"
IMU_TOPIC = "/imu/data"
RADAR_NODE = "hmmd_sensor"
ROSBRIDGE_PORT = 9090
TOPIC_STALE_AFTER_SEC = 2.0

# One severity scale for every page: ok (green), warn (amber), error (red). The pages add grey for "unknown".
# A service that delivers nothing is an error; a degraded, optional or recovering one is a warning.
LIDAR_VIEW = {
    "live": ("ok", "Live"),
    "demo": ("warn", "Demo"),
    "waiting": ("error", "No data"),
    "invalid": ("error", "Invalid data"),
    "stale": ("error", "Stale"),
    "error": ("error", "Error"),
}


@dataclass(frozen=True)
class ServiceHealth:
    key: str
    name: str
    level: str  # "ok", "warn" or "error"
    message: str
    # Short state word shown in the status chips and page badges, for example "No data".
    short: str = ""


def lidar_view(state: str) -> tuple[str, str]:
    return LIDAR_VIEW[state]


def tcp_port_open(port: int, host: str = "127.0.0.1", timeout: float = 0.3) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


class ServiceMonitor:
    """Turns runtime, SLAM and ROS graph facts into one list of per-service health rows."""

    def __init__(self, runtime, control=None, ros=None, slam=None, port_open=tcp_port_open) -> None:
        self.runtime = runtime
        self.control = control
        self.ros = ros
        self.slam = slam
        self.port_open = port_open

    def snapshot(self) -> list[dict]:
        rows = [self._lidar(), self._robot_link(), self._slam(), self._radar()]
        return [asdict(row) for row in rows if row is not None]

    def _lidar(self) -> ServiceHealth:
        status = self.runtime.snapshot()
        name = "LiDAR (navbot-lidar)"
        level, short = lidar_view(status.state)
        message = {
            "live": "scans arriving",
            "demo": "synthetic scan, not from the LiDAR",
            "waiting": "No data from the LiDAR sensor yet",
            "invalid": "The LiDAR sensor sends bytes that are not valid scans",
            "stale": "The LiDAR sensor stopped sending scans",
            "error": f"LiDAR sensor error: {status.last_error}",
        }[status.state]
        return ServiceHealth("lidar", name, level, message, short)

    def _robot_link(self) -> ServiceHealth | None:
        if self.ros is None:
            return None
        name = "Robot link (navbot-link-bridge)"
        odom_age = self.ros.topic_age(ODOM_TOPIC)
        imu_age = self.ros.topic_age(IMU_TOPIC)
        missing = [
            topic for topic, age in ((ODOM_TOPIC, odom_age), (IMU_TOPIC, imu_age))
            if age is None or age > TOPIC_STALE_AFTER_SEC
        ]
        if not missing:
            return ServiceHealth("robot_link", name, "ok", "odometry and IMU arriving", "OK")
        if self.ros.publisher_count(ODOM_TOPIC) == 0 and self.ros.publisher_count(IMU_TOPIC) == 0:
            return ServiceHealth(
                "robot_link", name, "error",
                "navbot-link-bridge is not running (nothing publishes /odom or /imu/data)", "Not running",
            )
        return ServiceHealth(
            "robot_link", name, "error",
            f"navbot-link-bridge is running, but no fresh {' or '.join(missing)} from the robot; "
            "check the robot's power and the UART wiring",
            "No data",
        )

    def _slam(self) -> ServiceHealth:
        name = "SLAM / localization"
        if self.control is None:
            return ServiceHealth("slam", name, "warn", "SLAM is not available on this service", "Unavailable")
        status = self.control.status()
        process = status["process"]
        localizing = status.get("mode") == "localization"
        launch = "AMCL localization" if localizing else "slam_toolbox"
        if process["state"] == "running":
            if localizing:
                return self._localization(status["localization"])
            if self.slam is not None and self.slam.snapshot()["state"] == "stale":
                return ServiceHealth(
                    "slam", name, "error",
                    f"slam_toolbox is running but published no map for over {MAP_STALE_AFTER_SEC:.0f} s", "Stale",
                )
            return ServiceHealth("slam", name, "ok", "running", "Running")
        detail = process["error"] or (
            None if process["exit_code"] is None else f"last exit code {process['exit_code']}"
        )
        text = f"{launch} is {process['state']}" + (f" ({detail})" if detail else "")
        # The supervisor restarts the process by itself, so "restarting" is a warning, not yet a failure.
        if process["state"] == "restarting":
            return ServiceHealth("slam", name, "warn", text, "Restarting")
        return ServiceHealth("slam", name, "error", text, "Not running")

    def _localization(self, localization: dict) -> ServiceHealth:
        name = "SLAM / localization"
        state = localization["state"]
        if state == "localized":
            return ServiceHealth("slam", name, "ok", "localized on the saved map", "Localized")
        if state == "searching":
            return ServiceHealth("slam", name, "warn", "searching for the pose on the saved map", "Searching")
        if state == "lost":
            if localization["error"]:
                return ServiceHealth("slam", name, "error", f"localization failed: {localization['error']}", "Lost")
            return ServiceHealth("slam", name, "warn", localization["hint"] or "pose lost", "Lost")
        return ServiceHealth("slam", name, "warn", "localization is starting", "Starting")

    def _radar(self) -> ServiceHealth | None:
        if self.ros is None:
            return None
        name = "Radar (hmmd_sensor, rosbridge)"
        missing = []
        if not self.ros.node_exists(RADAR_NODE):
            missing.append("the hmmd_sensor ROS node")
        if not self.port_open(ROSBRIDGE_PORT):
            missing.append(f"rosbridge on port {ROSBRIDGE_PORT}")
        if not missing:
            return ServiceHealth("radar", name, "ok", "running", "Running")
        return ServiceHealth(
            "radar", name, "warn", f"Not running: {' and '.join(missing)} (started on demand)", "Not running",
        )
