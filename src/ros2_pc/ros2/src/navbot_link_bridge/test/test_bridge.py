import pytest

from navbot_link_bridge import navbot_link_pb2
from navbot_link_bridge.bridge import ImuSample, LinkBridge
from navbot_link_bridge.framing import encode_frame
from navbot_link_bridge.odometry import DiffDriveOdometry, OdometryState

# Frames produced by the firmware encoder (tests/link/test_link_frame.cpp).
FIRMWARE_ODOMETRY_FRAME = bytes([
    0x0B, 0x08, 0x01, 0x10, 0x02, 0x72, 0x15, 0x08, 0xE8, 0x07, 0x11, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x04, 0xF8, 0x3F, 0x19, 0x01, 0x01, 0x01, 0x01, 0x01, 0x05, 0xD0, 0xBF, 0xC8, 0xD4, 0x00,
])
FIRMWARE_IMU_FRAME = bytes([
    0x0D, 0x08, 0x03, 0x10, 0x04, 0x52, 0x0E, 0x0D, 0x0A, 0xE8, 0x1C, 0x41, 0x35, 0x01, 0x01, 0x08,
    0x3F, 0x50, 0x87, 0xAD, 0x4B, 0xC7, 0x70, 0x00,
])


def new_bridge() -> LinkBridge:
    return LinkBridge(DiffDriveOdometry(0.05, 0.2))


def odom_frame(seq: int, t_us: int, left: float, right: float) -> bytes:
    msg = navbot_link_pb2.Message(
        seq=seq,
        t_ms=seq,
        odom=navbot_link_pb2.WheelOdometry(t_us=t_us, left_angle_rad=left, right_angle_rad=right),
    )
    return encode_frame(msg.SerializeToString())


def test_firmware_frames_decode_byte_by_byte():
    bridge = new_bridge()
    out = []
    for byte in FIRMWARE_ODOMETRY_FRAME:
        out += bridge.feed(bytes([byte]))
    assert out == []  # first odometry sample only sets the baseline

    out = []
    for byte in FIRMWARE_IMU_FRAME:
        out += bridge.feed(bytes([byte]))
    assert len(out) == 1
    sample = out[0]
    assert isinstance(sample, ImuSample)
    assert sample.linear_acceleration == pytest.approx((9.80665, 0.0, 0.0))
    assert sample.angular_velocity == pytest.approx((0.0, 0.0, 0.5))
    assert (sample.roll_rad, sample.pitch_rad, sample.yaw_rad) == (0.0, 0.0, 0.0)
    assert sample.t_us == 1234567
    assert bridge.lost_frames == 1  # seq 1 then seq 3
    assert (bridge.bad_frames, bridge.decode_errors, bridge.restarts) == (0, 0, 0)


def test_seq_gap_counts_lost_frames():
    bridge = new_bridge()
    assert bridge.feed(odom_frame(1, 0, 0.0, 0.0)) == []
    out = bridge.feed(odom_frame(4, 1_000_000, 20.0, 20.0))
    assert len(out) == 1
    assert isinstance(out[0], OdometryState)
    assert out[0].x_m == pytest.approx(1.0)
    assert out[0].linear_mps == pytest.approx(1.0)
    assert bridge.lost_frames == 2


def test_seq_reset_rebases_without_pose_jump():
    bridge = new_bridge()
    bridge.feed(odom_frame(1, 0, 0.0, 0.0))
    before = bridge.feed(odom_frame(2, 1_000_000, 20.0, 20.0))
    assert before[0].x_m == pytest.approx(1.0)

    # Robot restarted: seq counts from 1 again and the wheel angles jump.
    assert bridge.feed(odom_frame(1, 2_000_000, 500.0, 500.0)) == []
    assert bridge.restarts == 1

    after = bridge.feed(odom_frame(2, 3_000_000, 501.0, 501.0))
    assert after[0].x_m == pytest.approx(1.05)
    assert bridge.lost_frames == 0


def test_corrupted_frame_counts_bad_frame():
    bridge = new_bridge()
    bridge.feed(odom_frame(1, 0, 0.0, 0.0))
    corrupted = bytearray(odom_frame(2, 1_000_000, 20.0, 20.0))
    corrupted[3] ^= 0x01
    assert bridge.feed(bytes(corrupted)) == []
    assert bridge.bad_frames == 1


def test_non_protobuf_payload_counts_decode_error():
    bridge = new_bridge()
    assert bridge.feed(encode_frame(b"\xff\xff\xff")) == []
    assert bridge.decode_errors == 1
    assert bridge.bad_frames == 0
