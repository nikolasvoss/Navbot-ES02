import struct
import os
import pty
import sys
import time
from itertools import islice
from pathlib import Path
from threading import Event

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lds08rr_lidar.source import ByteActivity, SourceConfig, open_driver
from lds08rr_lidar.runtime import LidarRuntime
from lds08rr_lidar.web import make_lidar_app
from lds2d import Scan, ScanPoint
from lds2d.crc import camsense_checksum


class FixtureSerial:
    def __init__(self, data, max_chunk=7):
        self.data = data
        self.pos = 0
        self.max_chunk = max_chunk
        self.closed = False

    def read(self, size):
        count = min(size, self.max_chunk, len(self.data) - self.pos)
        result = self.data[self.pos:self.pos + count]
        self.pos += count
        return result

    def write(self, data):
        return len(data)

    def close(self):
        self.closed = True


def camsense_packet(start_deg, distances, corrupt=False):
    start = 0xA000 + round(start_deg * 64)
    end = 0xA000 + round((start_deg + 11) * 64)
    packet = bytearray(struct.pack("<BBBBHH", 0x55, 0xAA, 0x07, 0x0C, 19335, start))
    for distance in distances:
        packet.extend(struct.pack("<hBB", distance, 37, 0))
    packet.extend(struct.pack("<H", end))
    checksum = camsense_checksum(packet)
    packet.extend(struct.pack("<H", checksum ^ (1 if corrupt else 0)))
    return bytes(packet)


def test_upstream_camsense_decodes_fragmented_protocol_bytes(monkeypatch):
    data = b"noise" + camsense_packet(20, [700] * 12) + camsense_packet(32, [701] * 12)
    serial_port = FixtureSerial(data)
    monkeypatch.setattr("serial.Serial", lambda **kwargs: serial_port)
    config = SourceConfig(mode="serial", device=Path("fixture"))
    driver = open_driver(config, ByteActivity(), Event())
    try:
        points = list(islice(driver.points(), 24))
    finally:
        driver.close()
    assert len(points) == 24
    assert points[0].dist_mm == 700
    assert points[0].angle_deg == pytest.approx(20.0)
    assert serial_port.closed


def test_corrupt_camsense_packet_is_skipped_before_valid_packet(monkeypatch):
    serial_port = FixtureSerial(camsense_packet(4, [400] * 12, corrupt=True) + camsense_packet(20, [900] * 12))
    monkeypatch.setattr("serial.Serial", lambda **kwargs: serial_port)
    config = SourceConfig(mode="serial", device=Path("fixture"))
    driver = open_driver(config, ByteActivity(), Event())
    try:
        point = next(driver.points())
    finally:
        driver.close()
    assert point.angle_deg == pytest.approx(20)
    assert point.dist_mm == 900


class RuntimeDriver:
    def __init__(self, scans):
        self.values = scans
        self.closed = False

    def scans(self):
        yield from self.values
        while not self.closed:
            time.sleep(0.01)

    def close(self):
        self.closed = True


def wait_until(predicate, timeout=1.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.01)
    assert predicate(), "condition did not become true before timeout"


def test_runtime_discards_first_partial_rotation_and_hides_stale_plot():
    rotations = iter([
        Scan([ScanPoint(0, 111, 10)], 5.0),
        Scan([ScanPoint(0, 777, 80)], 5.0),
    ])
    driver = RuntimeDriver(rotations)
    config = SourceConfig(mode="serial", device=Path("fixture"))
    runtime = LidarRuntime(config, driver_factory=lambda *_: driver)
    runtime.start()
    try:
        wait_until(lambda: runtime.snapshot().scans_received == 1)
        assert runtime.snapshot().state == "live"
        client = make_lidar_app(runtime).test_client()
        fresh = client.get("/scan.json").get_json()
        assert fresh["n"] == 1
        assert fresh["points"][0]["dist"] == 777

        time.sleep(2.05)
        assert runtime.snapshot().state == "stale"
        stale = client.get("/scan.json").get_json()
        assert stale["n"] == 0
        assert client.get("/lidar").status_code == 200
        assert client.get("/status.json").get_json()["state"] == "stale"
    finally:
        stopped_at = time.monotonic()
        runtime.stop()
    assert time.monotonic() - stopped_at < 2.0


def test_runtime_reports_no_bytes_invalid_bytes_and_missing_uart():
    config = SourceConfig(mode="serial", device=Path("fixture"))
    driver = RuntimeDriver([])
    opened = Event()

    def open_empty(*_):
        opened.set()
        return driver

    runtime = LidarRuntime(config, driver_factory=open_empty)
    runtime.start()
    try:
        wait_until(opened.is_set)
        assert runtime.snapshot().state == "waiting"
        runtime.byte_activity.record(60)
        assert runtime.snapshot(now=time.monotonic() + 3).state == "invalid"
    finally:
        runtime.stop()

    unavailable = LidarRuntime(config, driver_factory=lambda *_: (_ for _ in ()).throw(FileNotFoundError("no UART")))
    unavailable.start()
    try:
        wait_until(lambda: unavailable.snapshot().state == "error")
        assert "no UART" in unavailable.snapshot().last_error
        assert make_lidar_app(unavailable).test_client().get("/lidar").status_code == 200
    finally:
        unavailable.stop()


def test_reopened_serial_session_waits_for_its_first_complete_scan():
    first = RuntimeDriver([
        Scan([ScanPoint(0, 400, 20)], 5.0),
        Scan([ScanPoint(0, 600, 30)], 5.0),
    ])
    second = RuntimeDriver([])
    opened = Event()
    calls = 0

    def reopen(*_):
        nonlocal calls
        calls += 1
        opened.set()
        return first if calls == 1 else second

    runtime = LidarRuntime(
        SourceConfig(mode="serial", device=Path("fixture")),
        driver_factory=reopen,
    )
    runtime.start()
    try:
        wait_until(lambda: runtime.snapshot().scans_received == 1)
        assert runtime.snapshot().state == "live"
        first.close()
        wait_until(lambda: calls >= 2 and runtime.snapshot().state == "waiting", timeout=3.0)
        assert runtime.snapshot().last_scan_age_sec is None
        assert runtime.snapshot().scans_received == 1
    finally:
        runtime.stop()


def test_ros_scan_conversion_uses_fixed_bins_metres_and_infinity():
    pytest.importorskip("rclpy")
    pytest.importorskip("diagnostic_msgs.msg")
    pytest.importorskip("sensor_msgs.msg")
    from builtin_interfaces.msg import Time
    from lds08rr_lidar.ros_node import to_laserscan

    scan = Scan([
        ScanPoint(0.1, 900, 30),
        ScanPoint(0.0, 800, 20),
        ScanPoint(1.0, 500, 70),
        ScanPoint(2.0, 0, 0),
    ], 5.0)
    message = to_laserscan(scan, Time(sec=12, nanosec=34))
    assert len(message.ranges) == 360
    assert message.angle_min == 0.0
    assert message.angle_increment == pytest.approx(2 * 3.141592653589793 / 360)
    assert message.header.stamp.sec == 12
    assert message.ranges[0] == pytest.approx(0.8)
    assert message.intensities[0] == 20.0
    assert message.ranges[1] == pytest.approx(0.5)
    assert message.ranges[2] == float("inf")


@pytest.mark.parametrize("native_angle, ros_bin", [
    (90.0, 270), (-90.0, 90), (361.0, 359), (-361.0, 1),
])
def test_ros_scan_converts_clockwise_native_bearings(native_angle, ros_bin):
    pytest.importorskip("rclpy")
    pytest.importorskip("diagnostic_msgs.msg")
    pytest.importorskip("sensor_msgs.msg")
    from builtin_interfaces.msg import Time
    from lds08rr_lidar.ros_node import to_laserscan

    message = to_laserscan(Scan([ScanPoint(native_angle, 1200, 37)], 5.0), Time())
    assert message.ranges[ros_bin] == pytest.approx(1.2)
    assert message.intensities[ros_bin] == 37.0
    assert message.ranges[360 - ros_bin] == float("inf")


def test_ros_scan_wraparound_keeps_nearest_return_and_its_intensity():
    pytest.importorskip("rclpy")
    pytest.importorskip("diagnostic_msgs.msg")
    pytest.importorskip("sensor_msgs.msg")
    from builtin_interfaces.msg import Time
    from lds08rr_lidar.ros_node import to_laserscan

    scan = Scan([
        ScanPoint(359.8, 900, 30),
        ScanPoint(-0.1, 800, 20),
        ScanPoint(360.2, 1000, 70),
    ], 5.0)
    message = to_laserscan(scan, Time())
    assert message.ranges[0] == pytest.approx(0.8)
    assert message.intensities[0] == 20.0
    assert sum(value != float("inf") for value in message.ranges) == 1


def test_module_tx_capture_decodes_literal_measurements(monkeypatch):
    captured = bytes.fromhex(
        "55AA070C344BD5CBC1001000CB000A00DA000900FA00030003012200"
        "FE003C0002013500090138000F013900140144001701350020010200"
        "90CEBF5F"
    )
    port = FixtureSerial(captured)
    monkeypatch.setattr("serial.Serial", lambda **kwargs: port)
    config = SourceConfig(mode="serial", device=Path("fixture"))
    driver = open_driver(config, ByteActivity(), Event())
    try:
        point = next(driver.points())
        assert point.dist_mm == 193
        assert point.quality == 16
        assert point.angle_deg == pytest.approx(175.328125)
    finally:
        driver.close()


def test_real_serial_transport_reconnects_and_stops_without_bytes(tmp_path):
    first_master, first_slave = pty.openpty()
    second_master, second_slave = pty.openpty()
    device = tmp_path / "lidar-uart"
    device.symlink_to(os.ttyname(first_slave))
    rotation = b"".join(camsense_packet(k * 12, [1000] * 12) for k in range(30))
    runtime = LidarRuntime(SourceConfig(mode="serial", device=device))
    runtime.start()
    try:
        wait_until(lambda: runtime._opened_at is not None)
        assert runtime.snapshot().state == "waiting"
        os.write(first_master, rotation * 3)
        wait_until(lambda: runtime.snapshot().scans_received >= 1, timeout=4)
        client = make_lidar_app(runtime).test_client()
        points = client.get("/scan.json").get_json()["points"]
        assert len(points) >= 350
        assert {point["dist"] for point in points} == {1000}
        count = runtime.snapshot().scans_received
        os.close(first_master)
        first_master = None
        wait_until(lambda: runtime.snapshot().state == "error", timeout=2)
        assert client.get("/scan.json").get_json()["points"] == []
        device.unlink()
        device.symlink_to(os.ttyname(second_slave))
        wait_until(lambda: runtime.snapshot().state == "waiting", timeout=3)
        os.write(second_master, rotation * 3)
        wait_until(lambda: runtime.snapshot().scans_received > count, timeout=4)
        assert runtime.snapshot().state == "live"
    finally:
        start = time.monotonic()
        runtime.stop()
        assert time.monotonic() - start < 2
        for descriptor in (first_master, first_slave, second_master, second_slave):
            if descriptor is not None:
                os.close(descriptor)


def test_hub_serves_tabs_slam_map_and_radar_without_hardware(tmp_path):
    import base64

    from lds08rr_lidar.slam import SlamState, grid_to_dict

    radar = tmp_path / "web"
    radar.mkdir()
    (radar / "index.html").write_text("<html><body><h1>HMMD radar data</h1></body></html>")
    (radar / "style.css").write_text("body{}")
    runtime = LidarRuntime(SourceConfig(mode="demo"), driver_factory=lambda *_: None)
    slam = SlamState()
    client = make_lidar_app(runtime, slam=slam, radar_web=radar).test_client()

    for path in ("/lidar", "/slam"):
        page = client.get(path).get_data(as_text=True)
        assert 'href="/slam"' in page and 'href="/radar/"' in page and 'href="/lidar"' in page

    radar_page = client.get("/radar/").get_data(as_text=True)
    assert "HMMD radar data" in radar_page and 'href="/slam"' in radar_page
    assert client.get("/radar/style.css").status_code == 200
    assert client.get("/radar/../index.html").status_code == 404

    empty = client.get("/map.json").get_json()
    assert empty["state"] == "waiting" and empty["map"] is None

    slam.set_grid(grid_to_dict(2, 1, 0.05, -1.0, -0.5, [-1, 100]))
    assert client.get("/map.json").get_json()["state"] == "no_pose"
    slam.set_pose(0.5, 0.25, 1.0)
    shown = client.get("/map.json").get_json()
    assert shown["state"] == "mapping"
    assert shown["pose"] == {"x": 0.5, "y": 0.25, "yaw": 1.0}
    assert list(base64.b64decode(shown["map"]["data"])) == [255, 100]
    assert shown["map"]["width"] == 2 and shown["map"]["origin_x"] == -1.0


def test_radar_tab_is_absent_not_broken_when_radar_files_missing(tmp_path):
    runtime = LidarRuntime(SourceConfig(mode="demo"), driver_factory=lambda *_: None)
    client = make_lidar_app(runtime, radar_web=tmp_path / "missing").test_client()
    assert client.get("/radar/").status_code == 404
    assert client.get("/lidar").status_code == 200
