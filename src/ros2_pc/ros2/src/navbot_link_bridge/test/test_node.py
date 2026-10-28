"""The node against the in-memory firmware. Needs a sourced ROS 2; skipped otherwise."""

import time

import pytest

rclpy = pytest.importorskip("rclpy")

from rcl_interfaces.msg import Log  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402

from fake_firmware import FakeFirmware, MemoryTransport, speed_loop_table  # noqa: E402
from helpers import WireTransport  # noqa: E402
from navbot_link import navbot_link_pb2 as pb  # noqa: E402
from navbot_link.framing import encode_frame  # noqa: E402
from navbot_link.link import Link  # noqa: E402
from navbot_link_bridge.node import ROSOUT_QOS, LinkBridgeNode  # noqa: E402


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class Harness:
    def __init__(self, firmware, transport=None, link_args=None, ros_args=()):
        self.firmware = firmware
        link_args = link_args or {}
        self.node = LinkBridgeNode(
            open_link=lambda port, baud: Link(transport or MemoryTransport(firmware), telemetry=True, **link_args),
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
        self.signals = []
        self.listener.create_subscription(Float64MultiArray, "/navbot/signals", self.signals.append, 1000)
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

    def node_warnings(self):
        return [m.msg for m in self.rosout if m.name == "navbot_link_bridge" and m.level == 30]

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


def limited_table():
    table = speed_loop_table()
    next(e for e in table if e.name == "speed_kp").limits = (0.0, 0.5)
    return table


def test_firmware_params_become_ros_parameters_and_set_goes_through_the_link(harness):
    fw = FakeFirmware(entries=limited_table())
    h = harness(fw)
    assert [(n, h.node.get_parameter(n).value) for n in ("speed_kp", "speed_ki", "speed_i_on")] == [
        ("speed_kp", 0.07999999821186066), ("speed_ki", 0.5), ("speed_i_on", True)]

    ok, bad, flag = h.node.set_parameters([
        Parameter("speed_kp", value=0.25),
        Parameter("speed_kp", value=0.75),
        Parameter("speed_i_on", value=False),
    ])
    assert (ok.successful, ok.reason) == (True, "")
    assert (bad.successful, bad.reason) == (False, "set_param refused: speed_kp must be within [0, 0.5]")
    assert flag.successful
    assert h.node.get_parameter("speed_kp").value == 0.25
    assert [e.value for e in fw.entries if e.name in ("speed_kp", "speed_i_on")] == [0.25, False]


def test_actions_and_signals_are_not_ros_parameters(harness):
    h = harness(FakeFirmware())
    assert not h.node.has_parameter("motor_reset")
    assert not h.node.has_parameter("speed_target")


def test_signals_parameter_starts_and_changes_the_signals_topic(harness):
    fw = FakeFirmware()
    h = harness(fw)
    result = h.node.set_parameters_atomically([
        Parameter("signals", value=["speed_target", "tilt_clipped"]),
        Parameter("signals_rate_hz", value=100.0),
    ])
    assert result.successful
    h.spin(until=lambda: len(h.signals) >= 2)
    first = h.signals[0]
    assert [(d.label, d.size, d.stride) for d in first.layout.dim] == [("t_us,speed_target,tilt_clipped", 3, 3)]
    assert list(first.data) == [1_010_000.0, 0.5, 1.0]
    assert list(h.signals[1].data) == [1_020_000.0, 0.5, 0.0]

    refused = h.node.set_parameters([Parameter("signals", value=["speed_target", "nope"])])[0]
    assert (refused.successful, refused.reason) == (False, "unknown signal nope")
    assert h.node.get_parameter("signals").value == ["speed_target", "tilt_clipped"]
    assert fw.subscribed_slots() == [1]

    assert h.node.set_parameters([Parameter("signals", value=[])])[0].successful
    assert fw.subscribed_slots() == []


def test_signals_from_the_launch_file_start_at_startup(harness):
    fw = FakeFirmware()
    h = harness(fw, ros_args=[Parameter("signals", value=["loop_count"]),
                              Parameter("signals_rate_hz", value=100.0)])
    h.spin(until=lambda: len(h.signals) >= 1)
    assert list(h.signals[0].data) == [1_010_000.0, 101.0]


def test_startup_value_of_a_firmware_parameter_is_applied_or_reported(harness):
    fw = FakeFirmware(entries=limited_table())
    h = harness(fw, ros_args=[Parameter("speed_ki", value=0.75), Parameter("speed_kp", value=0.9)])
    h.spin(0.3)
    values = {e.name: e.value for e in fw.entries}
    assert (values["speed_ki"], values["speed_kp"]) == (0.75, 0.08)
    assert h.node.get_parameter("speed_kp").value == 0.07999999821186066
    assert h.node_warnings() == [
        "startup value of speed_kp not applied: set_param refused: speed_kp must be within [0, 0.5]"]


def test_firmware_without_a_catalog_gives_one_warning_and_keeps_publishing_telemetry(harness):
    imu = pb.Message(seq=1, imu=pb.Imu(t_us=5, gz=0.5))
    h = harness(FakeFirmware(), transport=WireTransport(encode_frame(imu.SerializeToString())),
                link_args={"timeout_s": 0.02, "retries": 2})
    got = []
    h.listener.create_subscription(Imu, "imu/data", got.append, 10)
    h.spin(1.0, until=lambda: got and h.node_warnings())
    h.spin(0.3)
    assert len(got) == 1 and got[0].angular_velocity.z == 0.5
    assert h.node_warnings() == [
        "the robot did not answer GetCatalog (firmware without the signal table?); "
        "continuing with odometry and IMU only, no firmware parameters or signals"]
    assert not h.node.has_parameter("speed_kp")
    result = h.node.set_parameters([Parameter("signals", value=["speed_target"])])[0]
    assert (result.successful, result.reason) == (False, "the robot sent no catalog")


def test_a_robot_restart_reads_the_catalog_again_and_resumes_the_stream(harness):
    fw = FakeFirmware()
    h = harness(fw, ros_args=[Parameter("signals", value=["loop_count"]),
                              Parameter("signals_rate_hz", value=100.0)])
    h.spin(until=lambda: len(h.signals) >= 1)
    assert h.node.has_parameter("speed_ki")

    fw.restart()
    fw.entries[:] = [e for e in fw.entries if e.name != "speed_ki"]  # a new build without speed_ki
    fw.log("boot", "system")
    seen = len(h.signals)
    h.spin(2.0, until=lambda: fw.subscribed_slots() == [1] and len(h.signals) > seen)
    assert not h.node.has_parameter("speed_ki")
    assert h.node.has_parameter("speed_kp")
    assert fw.subscribed_slots() == [1]
    assert len(h.signals) > seen
