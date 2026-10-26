import pytest

from navbot_link_bridge.odometry import DiffDriveOdometry, gyro_yaw_sign_for

RADIUS = 0.05
SEPARATION = 0.2


def test_first_update_only_sets_baseline():
    odom = DiffDriveOdometry(RADIUS, SEPARATION)
    assert odom.update(1000, 3.0, -2.0) is None


def test_straight_line_one_metre():
    odom = DiffDriveOdometry(RADIUS, SEPARATION)
    odom.update(0, 0.0, 0.0)
    state = odom.update(1_000_000, 20.0, 20.0)
    assert state.x_m == pytest.approx(1.0)
    assert state.y_m == pytest.approx(0.0, abs=1e-12)
    assert state.yaw_rad == pytest.approx(0.0, abs=1e-12)
    assert state.linear_mps == pytest.approx(1.0)
    assert state.angular_rad_s == pytest.approx(0.0, abs=1e-12)


def test_spin_in_place():
    odom = DiffDriveOdometry(RADIUS, SEPARATION)
    odom.update(0, 0.0, 0.0)
    state = odom.update(1_000_000, -1.0, 1.0)
    # yaw = 2 * r * a / sep = 2 * 0.05 * 1 / 0.2
    assert state.yaw_rad == pytest.approx(0.5)
    assert state.x_m == pytest.approx(0.0, abs=1e-12)
    assert state.y_m == pytest.approx(0.0, abs=1e-12)
    assert state.angular_rad_s == pytest.approx(0.5)


def test_micros_wrap_gives_small_dt():
    odom = DiffDriveOdometry(RADIUS, SEPARATION)
    odom.update(0xFFFFFF00, 0.0, 0.0)
    state = odom.update(0x00000100, 1.0, 1.0)  # dt = 512 us
    assert state.linear_mps == pytest.approx(0.05 / 512e-6)


def test_equal_timestamp_keeps_baseline():
    odom = DiffDriveOdometry(RADIUS, SEPARATION)
    odom.update(5000, 0.0, 0.0)
    assert odom.update(5000, 10.0, 10.0) is None
    state = odom.update(6000, 1.0, 1.0)
    assert state.x_m == pytest.approx(0.05)


def test_rebase_keeps_pose_and_ignores_jump():
    odom = DiffDriveOdometry(RADIUS, SEPARATION)
    odom.update(0, 0.0, 0.0)
    odom.update(1_000_000, 20.0, 20.0)
    odom.rebase()
    assert odom.update(2_000_000, 500.0, 500.0) is None
    state = odom.update(3_000_000, 501.0, 501.0)
    assert state.x_m == pytest.approx(1.05)


@pytest.mark.parametrize(
    "radius, separation",
    [(0.0, SEPARATION), (-0.05, SEPARATION), (RADIUS, 0.0), (RADIUS, float("nan")), (float("inf"), SEPARATION)],
)
def test_invalid_geometry_raises(radius, separation):
    with pytest.raises(ValueError):
        DiffDriveOdometry(radius, separation)


def gyro_odometry(sign: int = 1) -> DiffDriveOdometry:
    return DiffDriveOdometry(RADIUS, SEPARATION, gyro_yaw_sign=sign)


def run_ticks(odom, ticks, gz, left_per_tick, right_per_tick, start_us=0, period_us=20_000):
    """Mimics the firmware order per tick: wheel sample first, then the IMU sample of the same tick."""
    state = None
    for k in range(ticks):
        t_us = (start_us + k * period_us) & 0xFFFFFFFF
        state = odom.update(t_us, k * left_per_tick, k * right_per_tick) or state
        odom.add_gyro(t_us, gz)
    return state


def test_gyro_straight_line_zero_rate():
    odom = gyro_odometry()
    state = run_ticks(odom, 51, 0.0, 0.4, 0.4)  # 50 steps of 0.02 m
    assert state.x_m == pytest.approx(1.0)
    assert state.y_m == pytest.approx(0.0, abs=1e-12)
    assert state.yaw_rad == pytest.approx(0.0, abs=1e-12)
    assert state.angular_rad_s == 0.0
    assert odom.gyro_fallbacks == 1  # the first step has no IMU interval yet


def test_gyro_wins_over_disagreeing_wheels():
    odom = gyro_odometry()
    # Wheels claim a fast clockwise spin; the gyro says 0.5 rad/s counter-clockwise.
    state = run_ticks(odom, 52, 0.5, 1.0, -1.0)
    # 50 gyro-covered steps of 0.02 s at 0.5 rad/s; the first step used wheel yaw -0.5.
    assert state.yaw_rad == pytest.approx(50 * 0.01 - 0.5)
    assert state.angular_rad_s == pytest.approx(0.5)
    assert state.x_m == pytest.approx(0.0, abs=1e-12)
    assert odom.gyro_fallbacks == 1


def test_gyro_sign_minus_one_flips_yaw():
    odom = gyro_odometry(sign=-1)
    run_ticks(odom, 2, 0.0, 0.0, 0.0)  # baseline and the one fallback step
    state = run_ticks(odom, 11, 1.0, 0.0, 0.0, start_us=40_000)
    # Ramp interval 0 -> -1 rad/s gives -0.01, then 9 full intervals of -0.02.
    assert state.yaw_rad == pytest.approx(-0.19)
    assert state.angular_rad_s == pytest.approx(-1.0)
    assert odom.gyro_fallbacks == 1


def test_gyro_yaw_is_trapezoid_of_rates():
    odom = gyro_odometry()
    odom.update(0, 0.0, 0.0)
    odom.add_gyro(0, 0.0)
    odom.add_gyro(20_000, 1.0)
    state = odom.update(20_000, 0.0, 0.0)
    assert state.yaw_rad == pytest.approx(0.01)
    assert state.angular_rad_s == pytest.approx(1.0)


def test_gyro_micros_wrap():
    odom = gyro_odometry()
    odom.update(0xFFFFFF00, 0.0, 0.0)
    odom.add_gyro(0xFFFFFF00, 2.0)
    odom.add_gyro(0x00004D00, 2.0)  # dt = 0x4E00 us = 19968 us
    state = odom.update(0x00004D00, 0.0, 0.0)
    assert state.yaw_rad == pytest.approx(2.0 * 19968e-6)
    assert odom.gyro_fallbacks == 0


def test_gyro_missing_falls_back_to_wheel_yaw():
    odom = gyro_odometry()
    odom.update(0, 0.0, 0.0)
    state = odom.update(1_000_000, -1.0, 1.0)
    assert state.yaw_rad == pytest.approx(0.5)  # wheel yaw, as in test_spin_in_place
    assert state.angular_rad_s == pytest.approx(0.5)
    assert odom.gyro_fallbacks == 1


def test_gyro_gap_is_not_integrated_and_falls_back():
    odom = gyro_odometry()
    odom.update(0, 0.0, 0.0)
    odom.add_gyro(0, 3.0)
    odom.add_gyro(100_000, 3.0)  # 100 ms > 60 ms gap: yaw unknown
    state = odom.update(100_000, -0.1, 0.1)
    assert state.yaw_rad == pytest.approx(0.05)  # wheel yaw, not 3.0 * 0.1
    assert odom.gyro_fallbacks == 1
    odom.add_gyro(120_000, 3.0)
    state = odom.update(120_000, -0.1, 0.1)
    assert state.yaw_rad == pytest.approx(0.05 + 0.06)
    assert odom.gyro_fallbacks == 1


def test_gyro_rebase_keeps_pose_and_drops_gyro_baseline():
    odom = gyro_odometry()
    odom.update(0, 0.0, 0.0)
    odom.add_gyro(0, 1.0)
    odom.add_gyro(20_000, 1.0)
    assert odom.update(20_000, 0.0, 0.0).yaw_rad == pytest.approx(0.02)
    odom.add_gyro(25_000, 1.0)  # collected before the restart, must not leak into the next step
    odom.rebase()
    # After the restart the robot clock starts again near zero.
    assert odom.update(1_000, 500.0, 500.0) is None
    odom.add_gyro(1_000, 1.0)
    odom.add_gyro(21_000, 1.0)
    state = odom.update(21_000, 500.0, 500.0)
    assert state.yaw_rad == pytest.approx(0.04)
    assert state.x_m == pytest.approx(0.0, abs=1e-12)


def test_wheel_mode_ignores_gyro():
    odom = DiffDriveOdometry(RADIUS, SEPARATION)
    odom.update(0, 0.0, 0.0)
    odom.add_gyro(0, 5.0)
    odom.add_gyro(20_000, 5.0)
    state = odom.update(1_000_000, -1.0, 1.0)
    assert state.yaw_rad == pytest.approx(0.5)
    assert odom.gyro_fallbacks == 0


@pytest.mark.parametrize(
    "source, sign, expected",
    [("wheels", 0, None), ("wheels", 1, None), ("gyro", 1, 1), ("gyro", -1, -1)],
)
def test_heading_parameters(source, sign, expected):
    assert gyro_yaw_sign_for(source, sign) == expected


@pytest.mark.parametrize("source, sign", [("gyro", 0), ("gyro", 2), ("compass", 1), ("", 1)])
def test_invalid_heading_parameters_raise(source, sign):
    with pytest.raises(ValueError):
        gyro_yaw_sign_for(source, sign)
