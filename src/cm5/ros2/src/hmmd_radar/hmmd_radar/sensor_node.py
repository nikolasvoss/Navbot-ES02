import math
import time

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from hmmd_interfaces.msg import RangeDopplerMap
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .protocol import DOPPLER_BINS, RANGE_GATES
from .serial_session import SerialSession


class HmmdSensorNode(Node):
    def __init__(self, *, serial_factory=None, **node_kwargs):
        super().__init__("hmmd_sensor", **node_kwargs)
        readonly = ParameterDescriptor(read_only=True)
        self.declare_parameter("port", "", readonly)
        self.declare_parameter("baud_rate", 115200, readonly)
        self.declare_parameter("poll_period_sec", 0.01, readonly)
        self.declare_parameter("stale_timeout_sec", 1.0, readonly)
        self.declare_parameter("diagnostics_period_sec", 1.0, readonly)
        port = self.get_parameter("port").value
        baud_rate = self.get_parameter("baud_rate").value
        self.poll_period = self._positive_float("poll_period_sec")
        self.stale_timeout = self._positive_float("stale_timeout_sec")
        self.diagnostics_period = self._positive_float("diagnostics_period_sec")
        if not isinstance(port, str):
            raise ValueError("port must be a string")
        if isinstance(baud_rate, bool) or not isinstance(baud_rate, int) or baud_rate <= 0:
            raise ValueError("baud_rate must be a positive integer")
        self.port = port
        self.session = SerialSession(port, baud_rate, serial_factory or serial.Serial, self.stale_timeout) if port else None
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._last_status_time = time.monotonic()
        self._last_status_frame_count = 0
        self.map_publisher = self.create_publisher(
            RangeDopplerMap,
            "/hmmd/rdmap",
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.status_publisher = self.create_publisher(DiagnosticArray, "/hmmd/status", 5)
        self.poll_timer = self.create_timer(self.poll_period, self._poll, clock=self._steady_clock)
        self.status_timer = self.create_timer(self.diagnostics_period, self._publish_status, clock=self._steady_clock)

    def _positive_float(self, name: str) -> float:
        value = self.get_parameter(name).value
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
        return float(value)

    def _poll(self):
        if self.session is None:
            return
        for frame in self.session.poll():
            message = RangeDopplerMap()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "hmmd_sensor"
            message.doppler_bins = DOPPLER_BINS
            message.range_gates = RANGE_GATES
            message.amplitude_squared = frame.amplitude_squared
            self.map_publisher.publish(message)

    def _publish_status(self):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "hmmd_sensor"
        if self.session is None:
            status.level = DiagnosticStatus.WARN
            status.message = "serial port not configured"
            connected = False
            stale = True
            age = "unknown"
            last_io_error = ""
            counters = (0, 0, 0, 0, 0)
            frame_rate = 0.0
        else:
            session_status = self.session.status()
            connected = session_status.connected
            stale = session_status.stale
            age = "unknown" if session_status.last_frame_age_seconds is None else f"{session_status.last_frame_age_seconds:.3f}"
            counters = (
                self.session.frames_received,
                self.session.parser.malformed_candidates,
                self.session.parser.discarded_bytes,
                self.session.reconnects,
                self.session.input_backlog_overflows,
            )
            last_io_error = self.session.last_io_error
            status_time = time.monotonic()
            elapsed = max(status_time - self._last_status_time, 1e-9)
            frame_rate = (self.session.frames_received - self._last_status_frame_count) / elapsed
            self._last_status_time = status_time
            self._last_status_frame_count = self.session.frames_received
            if not connected or stale:
                status.level = DiagnosticStatus.WARN
                status.message = "disconnected" if not connected else "no recent frames"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "receiving frames"
        status.hardware_id = self.port or "unconfigured"
        status.values = [
            KeyValue(key="connected", value=str(connected).lower()),
            KeyValue(key="stale", value=str(stale).lower()),
            KeyValue(key="last_frame_age_sec", value=age),
            KeyValue(key="frames_received", value=str(counters[0])),
            KeyValue(key="frame_rate_hz", value=f"{frame_rate:.3f}"),
            KeyValue(key="malformed_candidates", value=str(counters[1])),
            KeyValue(key="discarded_bytes", value=str(counters[2])),
            KeyValue(key="reconnects", value=str(counters[3])),
            KeyValue(key="input_backlog_overflows", value=str(counters[4])),
            KeyValue(key="last_io_error", value=last_io_error),
        ]
        message.status = [status]
        self.status_publisher.publish(message)

    def destroy_node(self):
        if self.session is not None:
            self.session.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HmmdSensorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
