"""The node against the in-memory firmware. Needs a sourced ROS 2; skipped otherwise."""

import os
import struct
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from rcl_interfaces.msg import Log  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402

from fake_firmware import FakeEntry, FakeFirmware, MemoryTransport, speed_loop_table  # noqa: E402
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
    def __init__(self, firmware, transport=None, link_args=None, ros_args=(), wait_for_attach=True):
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
        self.executor = MultiThreadedExecutor(num_threads=8)
        self.executor.add_node(self.node)
        self.executor.add_node(self.listener)
        self._spinner = threading.Thread(target=self.executor.spin, daemon=True)
        self._spinner.start()
        self.on_close = []
        self.rosout = []
        self.signals = []
        self.listener.create_subscription(Float64MultiArray, "/navbot/signals", self.signals.append, 1000)
        self.listener.create_subscription(Log, "/rosout", self.rosout.append, ROSOUT_QOS)
        if wait_for_attach:
            assert self.node.attached.wait(5.0)

    def spin(self, seconds: float = 0.5, until=lambda: False) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not until():
            time.sleep(0.005)

    def close(self) -> None:
        for close in self.on_close:
            close()
        self.node.close()
        self.executor.shutdown()
        self._spinner.join(2.0)
        self.node.destroy_node()
        self.listener.destroy_node()

    def node_warnings(self):
        return [m.msg for m in self.rosout if m.name == "navbot_link_bridge" and m.level == 30]

    def node_errors(self):
        return [m.msg for m in self.rosout if m.name == "navbot_link_bridge" and m.level == 40]

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


def test_a_robot_log_line_carries_the_time_the_bridge_forwarded_it(harness):
    fw = FakeFirmware()
    h = harness(fw)
    before = h.node.get_clock().now().nanoseconds
    fw.log("gyro bias loaded", "imu", pb.Log.INFO)
    h.spin(until=lambda: h.robot_lines())
    after = h.node.get_clock().now().nanoseconds
    (line,) = [m for m in h.rosout if m.name == "imu"]
    assert before <= line.stamp.sec * 10**9 + line.stamp.nanosec <= after


def test_rosout_lines_are_kept_for_ten_seconds(harness):
    h = harness(FakeFirmware())
    assert h.node._rosout_pub.qos_profile.lifespan.nanoseconds == 10_000_000_000


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


def test_a_refused_value_in_a_multi_parameter_set_puts_back_what_the_robot_took_before(harness):
    fw = FakeFirmware(entries=limited_table())
    h = harness(fw)
    result = h.node.set_parameters_atomically([
        Parameter("speed_ki", value=0.75),
        Parameter("speed_kp", value=0.75),
    ])
    assert (result.successful, result.reason) == (False, "set_param refused: speed_kp must be within [0, 0.5]")
    assert [(e.name, e.value) for e in fw.entries if e.name in ("speed_ki", "speed_kp")] == [
        ("speed_kp", 0.08), ("speed_ki", 0.5)]
    assert (h.node.get_parameter("speed_ki").value, h.node.get_parameter("speed_kp").value) == (
        0.5, 0.07999999821186066)


class ClampingFirmware(FakeFirmware):
    """Keeps a value inside the entry's limits instead of refusing it."""

    def _handle(self, msg, port):
        if msg.WhichOneof("body") == "set_param" and self.entries[msg.set_param.id].limits:
            low, high = self.entries[msg.set_param.id].limits
            (value,) = struct.unpack("<f", msg.set_param.value)
            msg.set_param.value = struct.pack("<f", min(max(value, low), high))
        super()._handle(msg, port)


def test_ros_stores_the_value_the_robot_kept_not_the_one_requested(harness):
    fw = ClampingFirmware(entries=limited_table())
    h = harness(fw)
    results = h.node.set_parameters([Parameter("speed_kp", value=0.75), Parameter("speed_ki", value=0.1)])
    assert [r.successful for r in results] == [True, True]
    assert [e.value for e in fw.entries if e.name == "speed_kp"] == [0.5]
    assert h.node.get_parameter("speed_kp").value == 0.5
    assert h.node.get_parameter("speed_ki").value == 0.10000000149011612


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


def test_an_integer_startup_value_of_an_f32_parameter_is_taken_as_a_float(harness):
    fw = FakeFirmware(entries=limited_table())
    h = harness(fw, ros_args=[Parameter("speed_kp", value=0)])
    h.spin(0.3)
    assert [e.value for e in fw.entries if e.name == "speed_kp"] == [0.0]
    assert (h.node.get_parameter("speed_kp").value, type(h.node.get_parameter("speed_kp").value)) == (0.0, float)
    assert h.node_errors() == []
    assert h.node_warnings() == []


def test_an_integer_set_of_an_f32_parameter_is_stored_as_a_float(harness):
    fw = FakeFirmware(entries=limited_table())
    h = harness(fw)
    result = h.node.set_parameters([Parameter("speed_kp", value=0)])[0]
    assert (result.successful, result.reason) == (True, "")
    assert [e.value for e in fw.entries if e.name == "speed_kp"] == [0.0]
    assert (h.node.get_parameter("speed_kp").value, type(h.node.get_parameter("speed_kp").value)) == (0.0, float)


def test_a_text_set_of_a_firmware_parameter_is_refused_with_the_type_rule(harness):
    fw = FakeFirmware(entries=limited_table())
    h = harness(fw)
    result = h.node.set_parameters([Parameter("speed_kp", value="fast")])[0]
    assert (result.successful, result.reason) == (False, "speed_kp takes a number, got 'fast'")
    assert h.node.get_parameter("speed_kp").value == 0.07999999821186066


def test_a_firmware_parameter_with_an_empty_name_is_skipped_with_a_warning(harness):
    entries = speed_loop_table()
    entries.insert(9, FakeEntry("", pb.CatalogEntry.PARAM, pb.CatalogEntry.F32, "", 1.0))
    h = harness(FakeFirmware(entries=entries), ros_args=[Parameter("signals", value=["loop_count"]),
                                                         Parameter("signals_rate_hz", value=100.0)])
    h.spin(until=lambda: len(h.signals) >= 1 and h.node_warnings())
    assert h.node_warnings() == ["firmware parameter '' not exposed: Invalid parameter name:"]
    assert (h.node.get_parameter("speed_kp").value, h.node.get_parameter("speed_ki").value) == (
        0.07999999821186066, 0.5)
    assert h.node_errors() == []
    assert list(h.signals[0].data) == [1_010_000.0, 101.0]


def test_a_restart_with_an_unusable_firmware_parameter_name_keeps_the_others(harness):
    fw = FakeFirmware()
    h = harness(fw)
    fw.entries.insert(9, FakeEntry("", pb.CatalogEntry.PARAM, pb.CatalogEntry.F32, "", 1.0))  # the new build
    fw.restart()
    fw.log("boot", "system")
    h.spin(2.0, until=lambda: h.node_warnings())
    assert h.node_warnings() == ["firmware parameter '' not exposed: Invalid parameter name:"]
    assert h.node.has_parameter("speed_kp") and h.node.has_parameter("speed_ki")
    assert h.node_errors() == []


def test_a_restart_whose_catalog_read_fails_takes_the_firmware_parameters_away(harness):
    fw = DeafFirmware()
    h = harness(fw)
    assert h.node.has_parameter("speed_ki")
    fw.ignored = frozenset({"get_catalog"})
    fw.restart()
    fw.log("boot", "system")
    h.spin(3.0, until=lambda: h.node_warnings())
    assert h.node_warnings() == [
        "catalog read failed (no reply to get_catalog); "
        "continuing with odometry and IMU only, no firmware parameters or signals"]
    assert not h.node.has_parameter("speed_ki")
    assert not h.node.has_parameter("speed_kp")


class PulledCable:
    """Reads nothing, then fails the way a pulled USB cable does."""

    def __init__(self, after_s: float = 0.1):
        self._fail_at = time.monotonic() + after_s

    def read(self) -> bytes:
        time.sleep(0.001)
        if time.monotonic() >= self._fail_at:
            raise OSError("cable pulled")
        return b""

    def write(self, data: bytes) -> None:
        pass

    def close(self) -> None:
        pass


def test_a_link_that_dies_before_the_catalog_is_read_reports_the_dead_link_not_the_robot(harness):
    h = harness(FakeFirmware(), transport=PulledCable(), wait_for_attach=False)
    assert h.node.attached.wait(3.0)  # the catalog wait ends at once with the dead link
    h.spin(1.0, until=lambda: h.node_errors())
    assert h.node_errors()[0] == "link failed: cable pulled; reopening"
    assert h.node_warnings() == []


def test_a_silent_catalog_read_gives_one_warning_and_keeps_publishing_telemetry(harness):
    imu = pb.Message(seq=1, imu=pb.Imu(t_us=5, gz=0.5))
    h = harness(FakeFirmware(), transport=WireTransport(encode_frame(imu.SerializeToString()), start_after_s=0.3),
                link_args={"timeout_s": 0.02, "retries": 2})
    got = []
    h.listener.create_subscription(Imu, "imu/data", got.append, 10)
    h.spin(1.0, until=lambda: got and h.node_warnings())
    h.spin(0.3)
    assert len(got) == 1 and got[0].angular_velocity.z == 0.5
    assert h.node_warnings() == [
        "catalog read failed (no reply to get_catalog); "
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


def test_tests_run_in_their_own_ros_graph(harness):
    h = harness(FakeFirmware())
    h.spin(1.0)
    assert (os.environ["ROS_DOMAIN_ID"], os.environ["ROS_LOCALHOST_ONLY"]) == ("77", "1")
    # Only the node under test publishes here; a bridge running on this machine must not show up.
    assert (h.listener.count_publishers("imu/data"), h.listener.count_publishers("odom")) == (1, 1)


class DeafFirmware(FakeFirmware):
    """Ignores the named request kinds, as a stuck robot does."""

    ignored = frozenset()

    def _handle(self, msg, port):
        if msg.WhichOneof("body") not in self.ignored:
            super()._handle(msg, port)


class TelemetryFeeder:
    """The robot's 50 Hz wheel odometry (robot clock 20 ms per frame), emitted in real time."""

    def __init__(self, firmware):
        self._firmware = firmware
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        n, start = 0, time.monotonic()
        while not self._stop.is_set():
            n += 1
            self._firmware.emit(pb.Message(odom=pb.WheelOdometry(
                t_us=n * 20_000, left_angle_rad=n * 0.01, right_angle_rad=n * 0.01)))
            self._stop.wait(max(0.0, start + n * 0.02 - time.monotonic()))

    def stop(self):
        self._stop.set()
        self._thread.join()


class UnpluggablePort(MemoryTransport):
    """The in-memory robot, whose port fails with OSError while `unplugged` is set."""

    unplugged = False

    def read(self) -> bytes:
        if self.unplugged:
            time.sleep(0.001)
            raise OSError("cable pulled")
        return super().read()


def wheel_angles(t_us: int, left: float, right: float) -> pb.Message:
    return pb.Message(odom=pb.WheelOdometry(t_us=t_us, left_angle_rad=left, right_angle_rad=right))


class OdomLog:
    """Receives /odom on an executor of its own, so that a busy test executor cannot add delay."""

    def __init__(self, h):
        self.arrivals, self.stamps, self.xs = [], [], []
        self._node = rclpy.create_node("odom_log")
        self._node.create_subscription(Odometry, "odom", self._on_odom, 100)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        h.on_close.append(self.close)

    def close(self):
        self._executor.shutdown()
        self._thread.join(2.0)
        self._node.destroy_node()

    def _on_odom(self, msg):
        self.arrivals.append(time.monotonic())
        self.stamps.append(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)
        self.xs.append(msg.pose.pose.position.x)

    def worst_gap_s(self, after=0.0, before=float("inf")):
        times = [t for t in self.arrivals if after <= t < before]
        return max(b - a for a, b in zip(times, times[1:]))

    def stamps_strictly_increase(self):
        return all(b > a for a, b in zip(self.stamps, self.stamps[1:]))


def test_a_silent_catalog_read_does_not_stall_odometry(harness):
    fw = DeafFirmware()
    fw.ignored = frozenset({"get_catalog"})
    feeder = TelemetryFeeder(fw)
    try:
        started = time.monotonic()
        h = harness(fw, wait_for_attach=False)  # three tries of 0.5 s: the catalog wait is 1.5 s
        odom = OdomLog(h)
        h.spin(0.5)
        assert not h.node.attached.is_set()  # the odometry below is published during the catalog wait
        h.spin(1.8)
    finally:
        feeder.stop()
    assert len(odom.arrivals) >= 90
    assert odom.worst_gap_s(after=started, before=started + 1.5) < 0.04
    assert odom.worst_gap_s() < 0.04
    assert odom.stamps_strictly_increase()


def test_a_catalog_read_that_fails_after_a_restart_does_not_stall_odometry(harness):
    fw = DeafFirmware()
    h = harness(fw)
    odom = OdomLog(h)
    feeder = TelemetryFeeder(fw)
    try:
        h.spin(0.5)
        fw.ignored = frozenset({"get_catalog"})
        fw.restart()
        restarted_at = time.monotonic()
        h.spin(2.3)
    finally:
        feeder.stop()
    assert odom.worst_gap_s(after=restarted_at) < 0.045  # 40 ms are the baseline frame after a restart
    assert odom.stamps_strictly_increase()


def test_a_firmware_parameter_set_that_the_robot_never_answers_does_not_stall_odometry(harness):
    fw = DeafFirmware(entries=limited_table())
    h = harness(fw)
    odom = OdomLog(h)
    h.spin(0.3, until=lambda: h.node.has_parameter("speed_kp"))
    fw.ignored = frozenset({"set_param"})
    feeder = TelemetryFeeder(fw)
    try:
        client = h.listener.create_client(SetParameters, "/navbot_link_bridge/set_parameters")
        assert client.wait_for_service(2.0)
        request = SetParameters.Request(parameters=[Parameter("speed_ki", value=0.75).to_parameter_msg()])
        started = time.monotonic()
        future = client.call_async(request)
        h.spin(4.0, until=future.done)
        result = future.result().results[0]
        answered_s = time.monotonic() - started
    finally:
        feeder.stop()
    assert (result.successful, result.reason) == (False, "no reply to set_param")
    assert answered_s > 1.4  # the set did wait for the robot
    assert odom.worst_gap_s(after=started) < 0.04
    assert odom.stamps_strictly_increase()


def test_a_reconnect_keeps_the_pose_and_takes_the_wheel_angles_as_a_new_baseline(harness):
    fw = FakeFirmware()
    port = UnpluggablePort(fw)
    h = harness(fw, transport=port)
    odom = OdomLog(h)
    fw.emit(wheel_angles(0, 0.0, 0.0))
    fw.emit(wheel_angles(1_000_000, 20.0, 20.0))
    h.spin(2.0, until=lambda: len(odom.xs) >= 1)
    port.unplugged = True
    h.spin(2.0, until=lambda: h.node_errors())
    fw.emit(wheel_angles(2_000_000, 500.0, 500.0))  # the wheels turned while the port was closed
    fw.emit(wheel_angles(3_000_000, 501.0, 501.0))
    port.unplugged = False
    h.spin(3.0, until=lambda: len(odom.xs) >= 2)
    assert odom.xs == pytest.approx([1.0, 1.05])
