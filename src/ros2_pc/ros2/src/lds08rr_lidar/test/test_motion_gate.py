from lds08rr_lidar.motion_gate import MAX_SPEED_M_S, MAX_YAW_RATE_RAD_S, MotionGate


def feed(gate, first, last, linear=0.0, angular=0.0, step=0.02):
    """Odometry every `step` seconds from index `first` to `last` (in steps of 1/50 s)."""
    for index in range(first, last + 1):
        gate.add_odom(index / 50, linear, angular)


def test_slow_motion_keeps_scan_and_reports_limits():
    gate = MotionGate()
    feed(gate, -5, 50, linear=0.1, angular=0.1)
    assert gate.apply(0.2, 0.2) is True
    assert gate.snapshot() == {
        "state": "ok", "scans_dropped": 0, "reason": None,
        "max_yaw_rate": MAX_YAW_RATE_RAD_S, "max_speed": MAX_SPEED_M_S,
    }


def test_turn_over_limit_drops_scan():
    gate = MotionGate()
    feed(gate, -5, 50, angular=1.0)
    assert gate.apply(0.2, 0.2) is False
    assert gate.snapshot()["state"] == "limiting"
    assert gate.snapshot()["reason"] == "turning too fast"
    assert gate.snapshot()["scans_dropped"] == 1


def test_speed_over_limit_drops_scan():
    gate = MotionGate()
    feed(gate, -5, 50, linear=0.5)
    assert gate.apply(0.2, 0.2) is False
    assert gate.snapshot()["reason"] == "driving too fast"


def test_sample_just_before_window_start_counts():
    gate = MotionGate()
    feed(gate, -5, 9)
    gate.add_odom(0.2, 0.0, 1.0)  # latest sample before the window start (0.205 s) violates
    feed(gate, 11, 50)
    assert gate.apply(0.205, 0.2) is False


def test_holdoff_keeps_dropping_then_recovers():
    gate = MotionGate()
    for index in range(-5, 51):
        stamp = index / 50
        angular = 1.0 if index == 15 else 0.0  # one violating sample at t = 0.30 s
        gate.add_odom(stamp, 0.0, angular)
    assert gate.apply(0.2, 0.2) is False  # window holds the violation
    assert gate.apply(0.4, 0.2) is False  # 0.4 < 0.30 + 0.4: hold-off
    assert gate.apply(0.6, 0.2) is False  # 0.6 < 0.70: still hold-off
    assert gate.apply(0.8, 0.2) is True  # 0.8 >= 0.70: recovered
    assert gate.snapshot()["state"] == "ok"
    assert gate.snapshot()["reason"] is None
    assert gate.snapshot()["scans_dropped"] == 3


def test_no_odom_keeps_scan_with_state_no_odom():
    gate = MotionGate()
    assert gate.apply(0.2, 0.2) is True
    assert gate.snapshot()["state"] == "no_odom"

    # Only a sample 0.5 s before the window: nothing within 0.1 s of its start.
    gate.add_odom(-0.3, 0.0, 1.0)
    assert gate.apply(0.2, 0.2) is True
    assert gate.snapshot() == {
        "state": "no_odom", "scans_dropped": 0, "reason": None,
        "max_yaw_rate": MAX_YAW_RATE_RAD_S, "max_speed": MAX_SPEED_M_S,
    }


def test_out_of_order_odom_is_ignored():
    gate = MotionGate()
    feed(gate, -5, 50)
    gate.add_odom(0.3, 0.0, 1.0)  # older than the newest sample (1.0 s): ignored
    assert gate.apply(0.2, 0.2) is True
    assert gate.snapshot()["state"] == "ok"
