import math

import pytest

from lds08rr_lidar.tilt_gate import TiltConfig, TiltGate, config_from_mount, roll_from_quaternion

INF = math.inf
STEP = math.tau / 360
# lds08rr bins: robot front is bin 120, rear bin 300, left bin 210, right bin 30 (mount yaw -120 deg).
FRONT, REAR, LEFT, RIGHT = 120, 300, 210, 30


def config(sign=1.0, level=0.0, yaw=-math.radians(120.0)):
    return TiltConfig(
        scan_height_m=0.20, roll_to_pitch_sign=sign, level_roll_rad=level,
        max_pitch_rad=0.06, floor_margin_m=0.03, mount_yaw_rad=yaw,
    )


def scan(**ranges_by_bin):
    ranges = [INF] * 360
    for key, distance in ranges_by_bin.items():
        ranges[int(key[1:])] = distance
    return ranges


def steady_imu(gate, roll, start=-0.1, end=0.3):
    t = start
    while t <= end + 1e-9:
        gate.add_imu(t, roll)
        t += 0.02


def test_level_scan_passes_unchanged():
    gate = TiltGate(config())
    steady_imu(gate, 0.0)
    ranges = scan(b120=5.0, b300=5.0, b210=5.0)
    assert gate.apply(ranges, 0.0, STEP, 0.0, 0.2) == ranges
    assert gate.snapshot() == {"state": "active", "scans_dropped": 0, "beams_dropped": 0}


def test_nose_down_drops_far_front_keeps_near_front_and_rear():
    gate = TiltGate(config())
    steady_imu(gate, 0.05)  # pitch 0.05 rad nose-down: front beams reach the floor beyond 3.4 m
    gated = gate.apply(scan(b120=4.0, b121=2.0, b300=4.0, b210=10.0, b30=10.0), 0.0, STEP, 0.0, 0.2)
    assert gated == scan(b121=2.0, b300=4.0, b210=10.0, b30=10.0)
    assert gate.snapshot() == {"state": "active", "scans_dropped": 0, "beams_dropped": 1}


def test_imu_roll_sign_and_level_offset_define_pitch():
    gate = TiltGate(config(sign=-1.0, level=0.10))
    steady_imu(gate, 0.05)  # pitch = -1 * (0.05 - 0.10) = +0.05 nose-down
    assert gate.apply(scan(b120=4.0, b300=4.0), 0.0, STEP, 0.0, 0.2) == scan(b300=4.0)


def test_pitch_swing_inside_window_drops_far_beams_both_ways():
    gate = TiltGate(config())
    gate.add_imu(-0.01, 0.05)  # sample just before the window counts
    for t in (0.03, 0.07, 0.11, 0.15, 0.19):
        gate.add_imu(t, 0.0)
    gate.add_imu(0.13, -0.05)  # out of order: ignored
    gate.add_imu(0.21, -0.05)  # after the window: ignored
    assert gate.apply(scan(b120=4.0, b300=4.0), 0.0, STEP, 0.0, 0.2) == scan(b300=4.0)


def test_over_limit_scan_is_dropped():
    gate = TiltGate(config())
    steady_imu(gate, 0.07)
    assert gate.apply(scan(b120=1.0), 0.0, STEP, 0.0, 0.2) is None
    assert gate.snapshot() == {"state": "active", "scans_dropped": 1, "beams_dropped": 0}


def test_mount_yaw_maps_bins_to_robot_bearing():
    rotated, unrotated = TiltGate(config()), TiltGate(config(yaw=0.0))
    steady_imu(rotated, 0.05)
    steady_imu(unrotated, 0.05)
    ranges = scan(b0=4.0, b120=4.0)
    assert rotated.apply(ranges, 0.0, STEP, 0.0, 0.2) == scan(b0=4.0)
    assert unrotated.apply(ranges, 0.0, STEP, 0.0, 0.2) == scan(b120=4.0)


def test_unconfigured_passes_through():
    gate = TiltGate(None)
    steady_imu(gate, 0.07)
    ranges = scan(b120=4.0)
    assert gate.apply(ranges, 0.0, STEP, 0.0, 0.2) == ranges
    assert gate.snapshot() == {"state": "unconfigured", "scans_dropped": 0, "beams_dropped": 0}


def test_measured_mount_constants_configure_the_gate():
    result = config_from_mount()
    assert (result.scan_height_m, result.roll_to_pitch_sign, result.level_roll_rad) == (0.122, -1, -0.0035)


def test_missing_mount_measurement_turns_the_gate_off(monkeypatch):
    monkeypatch.setattr("lds08rr_lidar.mount.IMU_ROLL_LEVEL_RAD", None)
    assert config_from_mount() is None


@pytest.mark.parametrize("name, value", [
    ("SCAN_PLANE_HEIGHT_M", 12.2),
    ("IMU_ROLL_TO_PITCH_SIGN", 0),
    ("IMU_ROLL_LEVEL_RAD", -4.0),
])
def test_mount_constants_in_wrong_units_are_refused(monkeypatch, name, value):
    monkeypatch.setattr(f"lds08rr_lidar.mount.{name}", value)
    with pytest.raises(ValueError):
        config_from_mount()


@pytest.mark.parametrize("start, end", [(-0.1, 0.05), (0.15, 0.3), (-2.0, -0.2)])
def test_stale_or_missing_imu_passes_through(start, end):
    gate = TiltGate(config())
    steady_imu(gate, 0.07, start=start, end=end)
    ranges = scan(b120=4.0)
    assert gate.apply(ranges, 0.0, STEP, 0.0, 0.2) == ranges
    assert gate.snapshot() == {"state": "no_imu", "scans_dropped": 0, "beams_dropped": 0}


def test_no_imu_at_all_passes_through():
    gate = TiltGate(config())
    assert gate.apply(scan(b120=4.0), 0.0, STEP, 0.0, 0.2) == scan(b120=4.0)
    assert gate.snapshot()["state"] == "no_imu"


def test_roll_from_quaternion():
    assert roll_from_quaternion(0.0, 0.0, 0.0, 1.0) == 0.0
    # roll 0.2 rad, yaw 1.0 rad, pitch 0
    assert roll_from_quaternion(0.08761206554319244, 0.047862689546603394, 0.477030407851843, 0.8731983044562818) == pytest.approx(0.2)
