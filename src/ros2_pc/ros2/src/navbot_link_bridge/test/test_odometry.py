import pytest

from navbot_link_bridge.odometry import DiffDriveOdometry

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
