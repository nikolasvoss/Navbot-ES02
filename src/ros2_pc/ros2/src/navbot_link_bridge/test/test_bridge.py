import time

import pytest

from fake_firmware import FakeFirmware, MemoryTransport
from navbot_link import navbot_link_pb2 as pb
from navbot_link.framing import encode_frame
from navbot_link.link import Link, Restarted
from navbot_link_bridge.bridge import ImuSample, LinkBridge, RobotLog
from navbot_link_bridge.odometry import DiffDriveOdometry, OdometryState
from helpers import WireTransport, poll_until

# Frames produced by the firmware encoder (tests/link/test_link_frame.cpp).
FIRMWARE_ODOMETRY_FRAME = bytes([
    0x0B, 0x08, 0x01, 0x10, 0x02, 0x72, 0x15, 0x08, 0xE8, 0x07, 0x11, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x04, 0xF8, 0x3F, 0x19, 0x01, 0x01, 0x01, 0x01, 0x01, 0x05, 0xD0, 0xBF, 0xC8, 0xD4, 0x00,
])
FIRMWARE_IMU_FRAME = bytes([
    0x0D, 0x08, 0x03, 0x10, 0x04, 0x52, 0x0E, 0x0D, 0x0A, 0xE8, 0x1C, 0x41, 0x35, 0x01, 0x01, 0x08,
    0x3F, 0x50, 0x87, 0xAD, 0x4B, 0xC7, 0x70, 0x00,
])


def new_bridge(transport, **link_args) -> LinkBridge:
    link = Link(transport, telemetry=True, clock=lambda: 0.0, **link_args)
    return LinkBridge(link, DiffDriveOdometry(0.05, 0.2))


def odom(seq: int, t_us: int, left: float, right: float) -> pb.Message:
    return pb.Message(seq=seq, odom=pb.WheelOdometry(t_us=t_us, left_angle_rad=left, right_angle_rad=right))


def test_firmware_frames_decode_byte_by_byte():
    bridge = new_bridge(WireTransport(FIRMWARE_ODOMETRY_FRAME + FIRMWARE_IMU_FRAME))
    out = poll_until(bridge, 1)
    assert len(out) == 1  # the first odometry sample only sets the baseline
    sample = out[0]
    assert isinstance(sample, ImuSample)
    assert sample.linear_acceleration == pytest.approx((9.80665, 0.0, 0.0))
    assert sample.angular_velocity == pytest.approx((0.0, 0.0, 0.5))
    assert (sample.roll_rad, sample.pitch_rad, sample.yaw_rad) == (0.0, 0.0, 0.0)
    assert sample.t_us == 1234567
    assert bridge.link.lost_frames == 1  # seq 1 then seq 3
    assert (bridge.link.bad_frames, bridge.link.restarts) == (0, 0)
    bridge.link.close()


def test_wheel_angles_become_pose_and_speed():
    fw = FakeFirmware()
    bridge = new_bridge(MemoryTransport(fw))
    fw.emit(odom(0, 0, 0.0, 0.0))
    fw.emit(odom(0, 1_000_000, 20.0, 20.0))
    out = poll_until(bridge, 1)
    assert len(out) == 1
    assert isinstance(out[0], OdometryState)
    assert out[0].x_m == pytest.approx(1.0)
    assert out[0].linear_mps == pytest.approx(1.0)
    bridge.link.close()


def test_robot_restart_rebases_without_pose_jump():
    fw = FakeFirmware()
    bridge = new_bridge(MemoryTransport(fw))
    fw.emit(odom(0, 0, 0.0, 0.0))
    fw.emit(odom(0, 1_000_000, 20.0, 20.0))
    assert poll_until(bridge, 1)[0].x_m == pytest.approx(1.0)

    fw.restart()  # seq counts from 1 again and the wheel angles jump
    fw.emit(odom(0, 2_000_000, 500.0, 500.0))
    restarted = poll_until(bridge, 1)
    assert restarted == [Restarted()]

    fw.emit(odom(0, 3_000_000, 501.0, 501.0))
    assert poll_until(bridge, 1)[0].x_m == pytest.approx(1.05)
    assert bridge.link.lost_frames == 0
    bridge.link.close()


def test_gyro_heading_uses_imu_gz_from_the_link():
    fw = FakeFirmware()
    link = Link(MemoryTransport(fw), telemetry=True, clock=lambda: 0.0)
    bridge = LinkBridge(link, DiffDriveOdometry(0.05, 0.2, gyro_yaw_sign=-1))
    for t_us, left, right in ((0, 0.0, 0.0), (20_000, 0.0, 0.0), (40_000, 1.0, -1.0)):
        fw.emit(odom(0, t_us, left, right))
        if t_us < 40_000:
            fw.emit(pb.Message(imu=pb.Imu(t_us=t_us, gz=0.5)))
    states = [x for x in poll_until(bridge, 4) if isinstance(x, OdometryState)]
    assert len(states) == 2
    assert states[-1].yaw_rad == pytest.approx(-0.01)  # wheels disagree; gyro wins
    assert states[-1].angular_rad_s == pytest.approx(-0.5)
    link.close()


def test_robot_log_lines_keep_severity_tag_and_code():
    fw = FakeFirmware()
    bridge = new_bridge(MemoryTransport(fw))
    for level, text, tag in (
        (pb.Log.DEBUG, "d", "imu"),
        (pb.Log.INFO, "i", "param"),
        (pb.Log.WARN, "w", "imu"),
        (pb.Log.ERROR, "e", "motor"),
        (pb.Log.FATAL, "f", "motor"),
        (pb.Log.INFO, "no tag", ""),
        (pb.Log.LEVEL_UNSPECIFIED, "unnamed level", "x"),
    ):
        fw.log(text, tag, level)
    assert poll_until(bridge, 7) == [
        RobotLog(10, "imu", "d"),
        RobotLog(20, "param", "i"),
        RobotLog(30, "imu", "w"),
        RobotLog(40, "motor", "e"),
        RobotLog(50, "motor", "f"),
        RobotLog(20, "robot", "no tag"),
        RobotLog(20, "x", "unnamed level"),
    ]
    bridge.link.close()


def test_log_code_is_appended_to_the_text():
    bridge = new_bridge(WireTransport(
        encode_frame(pb.Message(seq=1, log=pb.Log(level=pb.Log.ERROR, code=12, text="stall", tag="motor")
                                ).SerializeToString())))
    assert poll_until(bridge, 1) == [RobotLog(40, "motor", "stall (code 12)")]
    bridge.link.close()
