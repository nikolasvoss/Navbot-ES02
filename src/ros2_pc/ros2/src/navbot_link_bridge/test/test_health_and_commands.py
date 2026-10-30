"""Health rows, command results and their conversion, against the in-memory firmware. No ROS needed."""

import time

import pytest

from fake_firmware import FakeFirmware, MemoryTransport, speed_loop_table
from helpers import with_health, poll_until
from navbot_link.health import HEALTH_SIGNALS
from navbot_link.link import CommandRefused, Link
from navbot_link_bridge.bridge import HealthRow, LinkBridge
from navbot_link_bridge.health import HealthMonitor
from navbot_link_bridge.odometry import DiffDriveOdometry


def new_bridge(transport) -> LinkBridge:
    link = Link(transport, telemetry=True, clock=lambda: 0.0)
    return LinkBridge(link, DiffDriveOdometry(0.05, 0.2))


def health_row(**values) -> HealthRow:
    """A row with every HEALTH_SIGNALS name; `values` overrides the calm defaults."""
    calm = {name: 0 for name in HEALTH_SIGNALS}
    calm.update(voltage_filtered_v=7.4, voltage_min_raw_v=7.1, uptime_s=12, balance_state=2)
    calm.update(values)
    return HealthRow(t_us=1_000_000, names=HEALTH_SIGNALS, values=tuple(calm[n] for n in HEALTH_SIGNALS))


def test_health_rows_arrive_with_every_listed_signal():
    fw = FakeFirmware(entries=with_health(speed_loop_table()))
    bridge = new_bridge(MemoryTransport(fw))
    bridge.controls.attach()
    assert bridge.controls.build_id() == "fake-build-1"
    assert bridge.controls.set_health(list(HEALTH_SIGNALS)) == ""
    (row,) = poll_until(bridge, 1)
    assert isinstance(row, HealthRow)
    assert row.names == HEALTH_SIGNALS
    assert dict(zip(row.names, row.values))["voltage_filtered_v"] == pytest.approx(7.4)
    bridge.link.close()


def test_an_unknown_health_name_is_refused_and_no_health_stream_starts():
    fw = FakeFirmware(entries=with_health(speed_loop_table()))
    bridge = new_bridge(MemoryTransport(fw))
    bridge.controls.attach()
    assert bridge.controls.set_health(["balance_state", "no_such_signal"]) == "unknown signal no_such_signal"
    assert bridge.controls.health is None
    assert fw.subscribed_slots() == []
    bridge.link.close()


def test_a_command_the_robot_refuses_comes_back_as_a_refusal_event():
    bridge = new_bridge(MemoryTransport(FakeFirmware()))  # CH5 not in the link position
    bridge.link.command(0.2, 0.0)
    (event,) = poll_until(bridge, 1)
    assert event == CommandRefused("CH5 not in link position (1)")
    bridge.link.close()


def test_an_accepted_command_reaches_the_firmware_with_its_values():
    fw = FakeFirmware(link_position=True)
    bridge = new_bridge(MemoryTransport(fw))
    bridge.link.command(0.2, -0.5)
    deadline = time.monotonic() + 2.0
    while not fw.commands and time.monotonic() < deadline:
        time.sleep(0.005)
    assert len(fw.commands) == 1
    assert fw.commands[0] == pytest.approx((0.2, -0.5))
    bridge.link.close()


def test_the_monitor_reports_ok_for_a_calm_robot():
    report = HealthMonitor().report(health_row())
    assert (report.level, report.message) == ("OK", "Run")
    assert report.voltage_v == pytest.approx(7.4)


def test_a_fault_is_an_error_named_by_its_reason():
    report = HealthMonitor().report(health_row(balance_state=4, balance_fault=7))
    assert (report.level, report.message) == ("ERROR", "Fault: Overspeed")


def test_a_tripped_overspeed_latch_is_an_error_even_when_balance_runs():
    report = HealthMonitor().report(health_row(overspeed_tripped=1))
    assert (report.level, report.message) == ("ERROR", "Run")


def test_a_rising_link_counter_is_a_warning_once():
    monitor = HealthMonitor()
    assert monitor.report(health_row(link_rx_bad_uart=3)).level == "OK"
    assert monitor.report(health_row(link_rx_bad_uart=4)).level == "WARN"
    assert monitor.report(health_row(link_rx_bad_uart=4)).level == "OK"


def test_the_values_show_the_state_and_fault_by_name():
    report = HealthMonitor().report(health_row(balance_state=4, balance_fault=6, rc_valid=1))
    shown = dict(report.values)
    assert shown["balance_state"] == "Fault"
    assert shown["balance_fault"] == "Fall"
    assert shown["rc_valid"] == "1"
    assert shown["voltage_filtered_v"] == "7.4"


def test_an_enum_value_outside_its_names_is_shown_as_unknown_not_raised():
    report = HealthMonitor().report(health_row(balance_state=9))
    assert report.message == "unknown (9)"
