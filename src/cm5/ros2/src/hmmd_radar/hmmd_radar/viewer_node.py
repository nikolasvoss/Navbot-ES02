import math
import time

import matplotlib.pyplot as plt
import rclpy
from hmmd_interfaces.msg import RangeDopplerMap
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .heatmap import RangeDopplerHeatmap
from .view_model import RangeDopplerViewModel


class HmmdHeatmapNode(Node):
    def __init__(self):
        super().__init__("hmmd_heatmap")
        readonly = ParameterDescriptor(read_only=True)
        self.declare_parameter("stale_timeout_sec", 1.0, readonly)
        self.declare_parameter("render_hz", 10.0, readonly)
        stale_after = self._positive_float("stale_timeout_sec")
        render_hz = self._positive_float("render_hz")
        self.render_period = 1.0 / render_hz
        self.model = RangeDopplerViewModel(stale_after)
        self.plot = RangeDopplerHeatmap(self.model)
        self.subscription = self.create_subscription(
            RangeDopplerMap,
            "/hmmd/rdmap",
            self._on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._last_warning_time = 0.0
        self.plot.figure.canvas.mpl_connect("key_press_event", self._on_key)

    def _positive_float(self, name: str) -> float:
        value = self.get_parameter(name).value
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
        return float(value)

    def _on_map(self, message):
        try:
            self.model.update(message.doppler_bins, message.range_gates, message.amplitude_squared)
        except (TypeError, ValueError) as error:
            now = time.monotonic()
            if now - self._last_warning_time >= 5.0:
                self.get_logger().warning(f"discarding invalid range-Doppler map: {error}")
                self._last_warning_time = now

    def _on_key(self, event):
        if event.key == "l":
            self.plot.toggle_scale()


def main(args=None):
    rclpy.init(args=args)
    node = HmmdHeatmapNode()
    next_render = time.monotonic()
    try:
        while rclpy.ok() and plt.fignum_exists(node.plot.figure.number):
            rclpy.spin_once(node, timeout_sec=0)
            now = time.monotonic()
            if now >= next_render:
                node.plot.render(node.model.is_stale(now))
                next_render = now + node.render_period
            plt.pause(0.001)
            delay = next_render - time.monotonic()
            if delay > 0.002:
                time.sleep(min(delay, 0.02))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
