import math
import time

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from hmmd_interfaces.msg import RangeDopplerMap
from hmmd_interfaces.srv import GetRadarConfig, SetRadarSetting
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
        self.get_radar_config_service = self.create_service(
            GetRadarConfig,
            "/hmmd_sensor/get_radar_config",
            self._get_radar_config,
        )
        self.set_radar_setting_service = self.create_service(
            SetRadarSetting,
            "/hmmd_sensor/set_radar_setting",
            self._set_radar_setting,
        )
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
        self._publish_maps(self.session.poll())

    def _publish_maps(self, frames):
        for frame in frames:
            message = RangeDopplerMap()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "hmmd_sensor"
            message.doppler_bins = DOPPLER_BINS
            message.range_gates = RANGE_GATES
            message.amplitude_squared = frame.amplitude_squared
            self.map_publisher.publish(message)

    def _get_radar_config(self, request, response):
        del request
        if self.session is None:
            outcome, stage, detail = "not_connected", "enter_config", "UART is not configured."
            result = None
        else:
            result = self.session.read_radar_config()
            outcome, stage, detail = result.outcome, result.stage, result.detail
            self._publish_maps(self.session.poll())
        response.success = bool(result and result.success)
        response.maximum_distance_gate = result.maximum_distance_gate if result else 0
        response.target_disappearance_delay_seconds = result.target_disappearance_delay_seconds if result else 0
        response.outcome = self._outcome_code(GetRadarConfig.Response, outcome)
        response.stage = self._read_stage_code(stage)
        response.detail = detail
        return response

    def _set_radar_setting(self, request, response):
        if (
            isinstance(request.setting, bool)
            or not isinstance(request.setting, int)
            or isinstance(request.value, bool)
            or not isinstance(request.value, int)
        ):
            result = None
            outcome, stage, detail = "invalid_request", "none", "Setting and value must be integers."
        elif request.setting not in (SetRadarSetting.Request.SETTING_MAXIMUM_DISTANCE_GATE, SetRadarSetting.Request.SETTING_TARGET_DISAPPEARANCE_DELAY):
            result = None
            outcome, stage, detail = "invalid_request", "none", "Unknown radar setting selector."
        elif request.setting == SetRadarSetting.Request.SETTING_MAXIMUM_DISTANCE_GATE and not 0 <= request.value <= 15:
            result = None
            outcome, stage, detail = "invalid_request", "none", "Maximum distance gate must be an integer from 0 to 15."
        elif request.setting == SetRadarSetting.Request.SETTING_TARGET_DISAPPEARANCE_DELAY and not 0 <= request.value <= 65535:
            result = None
            outcome, stage, detail = "invalid_request", "none", "Target disappearance delay must be an integer from 0 to 65535."
        elif self.session is None:
            result = None
            outcome, stage, detail = "not_connected", "enter_config", "UART is not configured."
        else:
            result = self.session.set_radar_setting(request.setting, request.value)
            outcome, stage, detail = result.outcome, result.stage, result.detail
            self._publish_maps(self.session.poll())
        response.success = bool(result and result.success)
        response.write_acknowledged = result.write_acknowledged if result else False
        response.has_observed_value = result.has_observed_value if result else False
        response.observed_value = result.observed_value if result else 0
        response.readback_matched = result.readback_matched if result else False
        response.save_acknowledged = result.save_acknowledged if result else False
        response.outcome = self._outcome_code(SetRadarSetting.Response, outcome)
        response.stage = self._write_stage_code(stage)
        response.detail = detail
        return response

    @staticmethod
    def _outcome_code(response_type, outcome):
        names = {
            "ok": "OUTCOME_OK",
            "invalid_request": "OUTCOME_INVALID_REQUEST",
            "not_connected": "OUTCOME_NOT_CONNECTED",
            "timeout": "OUTCOME_TIMEOUT",
            "device_rejected": "OUTCOME_DEVICE_REJECTED",
            "readback_mismatch": "OUTCOME_READBACK_MISMATCH",
            "protocol_error": "OUTCOME_PROTOCOL_ERROR",
            "io_error": "OUTCOME_IO_ERROR",
            "refresh_required": "OUTCOME_REFRESH_REQUIRED",
        }
        return getattr(response_type, names[outcome], response_type.OUTCOME_PROTOCOL_ERROR)

    @staticmethod
    def _read_stage_code(stage):
        return {
            "none": GetRadarConfig.Response.STAGE_NONE,
            "enter_config": GetRadarConfig.Response.STAGE_ENTER_CONFIG,
            "read_maximum_distance_gate": GetRadarConfig.Response.STAGE_READ_MAXIMUM_DISTANCE_GATE,
            "read_target_disappearance_delay": GetRadarConfig.Response.STAGE_READ_TARGET_DISAPPEARANCE_DELAY,
            "exit_config": GetRadarConfig.Response.STAGE_EXIT_CONFIG,
        }.get(stage, GetRadarConfig.Response.STAGE_NONE)

    @staticmethod
    def _write_stage_code(stage):
        return {
            "none": SetRadarSetting.Response.STAGE_NONE,
            "enter_config": SetRadarSetting.Response.STAGE_ENTER_CONFIG,
            "write": SetRadarSetting.Response.STAGE_WRITE,
            "readback": SetRadarSetting.Response.STAGE_READBACK,
            "save": SetRadarSetting.Response.STAGE_SAVE,
        }.get(stage, SetRadarSetting.Response.STAGE_NONE)

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
            counters = (0, 0, 0, 0, 0, 0, 0)
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
                self.session.deferred_map_overflows,
                self.session.late_command_replies,
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
            KeyValue(key="deferred_map_overflows", value=str(counters[5])),
            KeyValue(key="late_command_replies", value=str(counters[6])),
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
        if rclpy.ok():
            rclpy.shutdown()
