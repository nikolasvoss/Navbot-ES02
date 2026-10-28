import time

import pytest

from fake_firmware import FakeEntry, FakeFirmware, MemoryTransport, speed_loop_table
from navbot_link import navbot_link_pb2 as pb
from navbot_link.link import Link
from navbot_link.protocol import Type
from navbot_link_bridge.bridge import LinkBridge, SignalRow
from navbot_link_bridge.controls import ParamInfo
from navbot_link_bridge.odometry import DiffDriveOdometry
from helpers import WireTransport, poll_until


class CountingTransport(WireTransport):
    def __init__(self):
        super().__init__()
        self.written = 0

    def write(self, data: bytes) -> None:
        self.written += 1


def limited_table():
    table = speed_loop_table()
    next(e for e in table if e.name == "speed_kp").limits = (0.0, 0.5)
    return table


def new_bridge(firmware, **link_args) -> LinkBridge:
    link = Link(MemoryTransport(firmware), telemetry=True, clock=lambda: 0.0, **link_args)
    bridge = LinkBridge(link, DiffDriveOdometry(0.05, 0.2))
    assert bridge.controls.attach()
    return bridge


def test_catalog_params_are_listed_with_the_robots_values():
    bridge = new_bridge(FakeFirmware())
    assert bridge.controls.params() == [
        ParamInfo("speed_kp", Type.F32, 0.07999999821186066),
        ParamInfo("speed_ki", Type.F32, 0.5),
        ParamInfo("speed_i_on", Type.BOOL, True),
    ]
    bridge.link.close()


def test_set_param_reaches_the_robot_and_a_refusal_carries_its_text():
    fw = FakeFirmware(entries=limited_table())
    bridge = new_bridge(fw)
    controls = bridge.controls
    assert controls.set_param("speed_kp", 0.25) == ""
    assert controls.set_param("speed_i_on", False) == ""
    assert controls.set_param("speed_kp", 0.75) == "set_param refused: speed_kp must be within [0, 0.5]"
    assert controls.set_param("loop_count", 1) == "loop_count is not a parameter (kind signal)"
    assert [(p.name, p.value) for p in controls.params()] == [
        ("speed_kp", 0.25), ("speed_ki", 0.5), ("speed_i_on", False)]
    bridge.link.close()


def test_a_value_that_does_not_fit_the_wire_type_is_rejected_before_sending():
    fw = FakeFirmware(entries=limited_table())
    fw.entries.append(FakeEntry("window_ms", pb.CatalogEntry.PARAM, pb.CatalogEntry.U32, "ms", 20))
    bridge = new_bridge(fw)
    assert bridge.controls.set_param("window_ms", -1) == "window_ms: -1 does not fit u32"
    assert bridge.controls.set_param("window_ms", 40) == ""
    bridge.link.close()


def test_signal_rows_carry_names_and_values_in_subscription_order():
    fw = FakeFirmware()
    bridge = new_bridge(fw)
    assert bridge.controls.set_subscription(["speed_target", "tilt_clipped", "wheel_offset"], 100.0) == ""
    rows = poll_until(bridge, 2)[:2]
    assert rows == [
        SignalRow(1_010_000, ("speed_target", "tilt_clipped", "wheel_offset"), (0.5, True, -3)),
        SignalRow(1_020_000, ("speed_target", "tilt_clipped", "wheel_offset"), (0.5, False, -3)),
    ]
    assert rows[0].column_header == "t_us,speed_target,tilt_clipped,wheel_offset"
    assert rows[0].data == [1010000.0, 0.5, 1.0, -3.0]
    bridge.link.close()


def test_changing_the_signal_list_swaps_the_slot_and_keeps_the_old_one_on_failure():
    fw = FakeFirmware()
    bridge = new_bridge(fw)
    controls = bridge.controls
    controls.set_subscription(["speed_target"], 100.0)
    assert fw.subscribed_slots() == [1]

    assert controls.set_subscription(["speed_target", "nope"], 100.0) == "unknown signal nope"
    assert fw.subscribed_slots() == [1]

    assert controls.set_subscription(["loop_count"], 100.0) == ""
    assert fw.subscribed_slots() == [2]
    assert controls.set_subscription([], 100.0) == ""
    assert fw.subscribed_slots() == []
    assert controls.set_subscription(["loop_count"], -1.0) == "signals_rate_hz must not be negative, got -1"
    bridge.link.close()


def test_the_subscription_lease_is_renewed_while_the_bridge_runs():
    fw = FakeFirmware(lease_s=0.3)
    bridge = new_bridge(fw, renew_s=0.05)
    bridge.controls.set_subscription(["speed_target"], 100.0)
    time.sleep(0.8)  # 2.6 leases
    assert fw.subscribed_slots() == [1]
    rows = poll_until(bridge, 1)
    assert rows and isinstance(rows[0], SignalRow)
    bridge.link.close()


def test_a_robot_restart_ends_the_stream_and_the_old_slot_is_not_reported_again():
    fw = FakeFirmware()
    bridge = new_bridge(fw)
    bridge.controls.set_subscription(["speed_target"], 100.0)
    poll_until(bridge, 1)
    fw.restart()
    fw.log("boot", "system")
    out = poll_until(bridge, 2)
    time.sleep(0.05)
    out += bridge.poll()
    kinds = [type(x).__name__ for x in out if not isinstance(x, SignalRow)]  # rows sent before the restart
    assert kinds == ["Restarted", "RobotLog"]
    assert bridge.controls.subscription is None
    bridge.link.close()


def test_firmware_without_a_catalog_leaves_the_controls_off_and_telemetry_running():
    link = Link(WireTransport(), telemetry=True, timeout_s=0.02, retries=2)
    bridge = LinkBridge(link, DiffDriveOdometry(0.05, 0.2))
    controls = bridge.controls
    assert controls.attach() is False
    assert controls.has_catalog is False
    assert controls.set_param("speed_kp", 0.1) == "the robot sent no catalog"
    assert controls.set_subscription(["speed_target"], 10.0) == "the robot sent no catalog"
    assert controls.set_subscription([], 10.0) == ""
    link.close()


def test_a_robot_that_ignored_get_catalog_is_not_asked_again_until_it_restarts():
    transport = CountingTransport()
    link = Link(transport, telemetry=True, timeout_s=0.02, retries=2)
    controls = LinkBridge(link, DiffDriveOdometry(0.05, 0.2)).controls
    assert controls.attach() is False
    assert transport.written == 2  # the retries of one GetCatalog
    assert controls.attach() is False
    assert transport.written == 2
    controls.restarted()
    assert controls.attach() is False
    assert transport.written == 4
    link.close()
