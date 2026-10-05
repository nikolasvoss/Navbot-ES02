import importlib.util
import struct
import time
import unittest

ROS_AVAILABLE = importlib.util.find_spec("rclpy") is not None

if ROS_AVAILABLE:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.parameter import Parameter
    from hmmd_radar.sensor_node import HmmdSensorNode
    from hmmd_interfaces.msg import RangeDopplerMap
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy


def _frame(values):
    return b"\xAA\xBF\x10\x14" + struct.pack("<320I", *values) + b"\xFD\xFC\xFB\xFA"


class FakeSerial:
    def __init__(self, data):
        self.data = bytearray(data)
        self.is_open = False

    @property
    def in_waiting(self):
        return len(self.data)

    def open(self):
        self.is_open = True

    def write(self, data):
        return len(data)

    def read(self, size):
        result = bytes(self.data[:size])
        del self.data[:size]
        return result

    def reset_input_buffer(self):
        self.data.clear()

    def close(self):
        self.is_open = False


@unittest.skipUnless(ROS_AVAILABLE, "ROS 2 runtime is unavailable")
class RosGraphTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_sensor_publishes_raw_map_and_status(self):
        values = list(range(320))
        fake = FakeSerial(_frame(values))
        sensor = HmmdSensorNode(
            serial_factory=lambda: fake,
            parameter_overrides=[
                Parameter("port", value="fake"),
                Parameter("diagnostics_period_sec", value=0.05),
            ],
        )
        subscriber = Node("hmmd_test_subscriber")
        maps = []
        statuses = []
        subscription = subscriber.create_subscription(
            RangeDopplerMap,
            "/hmmd/rdmap",
            maps.append,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        subscriber.create_subscription(DiagnosticArray, "/hmmd/status", statuses.append, 5)
        try:
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and sensor.map_publisher.get_subscription_count() == 0:
                rclpy.spin_once(subscriber, timeout_sec=0.01)
            while time.monotonic() < deadline and (not maps or not statuses):
                rclpy.spin_once(sensor, timeout_sec=0.01)
                rclpy.spin_once(subscriber, timeout_sec=0.01)

            self.assertTrue(maps)
            self.assertEqual(maps[-1].doppler_bins, 20)
            self.assertEqual(maps[-1].range_gates, 16)
            self.assertEqual(list(maps[-1].amplitude_squared), values)
            self.assertEqual(maps[-1].header.frame_id, "hmmd_sensor")
            self.assertTrue(statuses)
            self.assertEqual(sensor.map_publisher.topic_name, "/hmmd/rdmap")
            self.assertEqual(subscription.topic_name, "/hmmd/rdmap")
            self.assertEqual(statuses[-1].status[0].name, "hmmd_sensor")
        finally:
            sensor.destroy_node()
            subscriber.destroy_node()


if __name__ == "__main__":
    unittest.main()
