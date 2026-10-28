"""The node against the in-memory firmware. Needs a sourced ROS 2; skipped otherwise."""

import time

import pytest

rclpy = pytest.importorskip("rclpy")

from rcl_interfaces.msg import Log  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402

from fake_firmware import FakeFirmware, MemoryTransport  # noqa: E402
from navbot_link import navbot_link_pb2 as pb  # noqa: E402
from navbot_link.link import Link  # noqa: E402
from navbot_link_bridge.node import ROSOUT_QOS, LinkBridgeNode  # noqa: E402


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class Harness:
    def __init__(self, firmware, link_args=None, ros_args=()):
        self.firmware = firmware
        link_args = link_args or {}
        self.node = LinkBridgeNode(
            open_link=lambda port, baud: Link(MemoryTransport(firmware), telemetry=True, **link_args),
            parameter_overrides=[
                rclpy.parameter.Parameter("port", value="fake"),
                rclpy.parameter.Parameter("wheel_radius_m", value=0.05),
                rclpy.parameter.Parameter("wheel_separation_m", value=0.2),
                *ros_args,
            ],
        )
        self.listener = rclpy.create_node("listener")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.listener)
        self.rosout = []
        self.listener.create_subscription(Log, "/rosout", self.rosout.append, ROSOUT_QOS)

    def spin(self, seconds: float = 0.5, until=lambda: False) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not until():
            self.executor.spin_once(timeout_sec=0.01)

    def close(self) -> None:
        self.node.close()
        self.executor.shutdown()
        self.node.destroy_node()
        self.listener.destroy_node()

    def robot_lines(self):
        return [(m.level, m.name, m.msg) for m in self.rosout if m.name != "navbot_link_bridge"]


@pytest.fixture
def harness(ros):
    made = []

    def make(*args, **kwargs):
        made.append(Harness(*args, **kwargs))
        return made[-1]

    yield make
    for h in made:
        h.close()


def test_robot_log_frames_reach_rosout_with_their_severity_and_tag(harness):
    fw = FakeFirmware()
    h = harness(fw)
    fw.log("gyro bias loaded", "imu", pb.Log.INFO)
    fw.log("overspeed", "motor", pb.Log.ERROR)
    h.spin(until=lambda: len(h.robot_lines()) == 2)
    assert h.robot_lines() == [(20, "imu", "gyro bias loaded"), (40, "motor", "overspeed")]
