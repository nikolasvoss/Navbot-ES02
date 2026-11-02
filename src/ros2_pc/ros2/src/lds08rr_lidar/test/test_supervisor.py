import pytest

from lds08rr_lidar.supervisor import CHECKS, NavSupervisor


class FakeClock:
    def __init__(self, now=0.0):
        self.now = now

    def __call__(self):
        return self.now


GOOD_HEALTH = {"balance_state": "Run", "ch5_position": "1", "rc_valid": "1"}


def make(now=0.0):
    clock = FakeClock(now)
    return NavSupervisor(clock=clock), clock


def healthy(sup, health=GOOD_HEALTH):
    sup.set_health(health)
    sup.set_voltage(7.62)
    sup.set_localization("localized")
    sup.scan_seen()
    sup.set_nav_ready(True)


def details(sup):
    return {c["name"]: c["detail"] for c in sup.snapshot()["checks"]}


def test_before_any_input_every_check_fails_with_its_first_detail():
    sup, _ = make()
    snap = sup.snapshot()
    assert snap["state"] == "not_ready"
    assert snap["reason"] == "no robot health yet"
    assert [c["name"] for c in snap["checks"]] == list(CHECKS)
    assert all(c["ok"] is False for c in snap["checks"])
    assert details(sup) == {
        "balance": "no robot health yet",
        "ch5": "no robot health yet",
        "remote": "no robot health yet",
        "health": "no robot health yet",
        "localization": "localization is unknown",
        "scan": "no LiDAR scan for 1 s",
        "battery": "no battery voltage yet",
        "nav2": "Nav2 is not ready",
    }


def test_all_checks_passing_gives_ready_with_shown_values():
    sup, _ = make()
    healthy(sup)
    snap = sup.snapshot()
    assert snap["state"] == "ready"
    assert snap["reason"] is None
    assert details(sup) == {
        "balance": "Run",
        "ch5": "1",
        "remote": "1",
        "health": "0.0 s",
        "localization": "localized",
        "scan": "0.0 s",
        "battery": "7.62 V",
        "nav2": "ready",
    }


@pytest.mark.parametrize(
    ("break_it", "reason"),
    [
        (lambda s, c: s.set_health({**GOOD_HEALTH, "balance_state": "Off"}), "balance is Off"),
        (lambda s, c: s.set_health({**GOOD_HEALTH, "ch5_position": "2"}), "CH5 is in position 2, not 1"),
        (lambda s, c: s.set_health({**GOOD_HEALTH, "rc_valid": "0"}), "no remote signal"),
        (lambda s, c: s.set_localization("searching"), "localization is searching"),
        (lambda s, c: s.set_voltage(6.5), "battery at 6.50 V, below 6.8 V"),
        (lambda s, c: s.set_nav_ready(False), "Nav2 is not ready"),
    ],
)
def test_one_failing_check_gives_its_detail_as_reason(break_it, reason):
    sup, clock = make()
    healthy(sup)
    break_it(sup, clock)
    assert sup.snapshot()["state"] == "not_ready"
    assert sup.snapshot()["reason"] == reason


def test_health_older_than_3_seconds_fails_with_its_age():
    sup, clock = make()
    healthy(sup)
    clock.now = 3.0
    sup.scan_seen()
    assert sup.snapshot()["state"] == "ready"
    assert details(sup)["health"] == "3.0 s"
    clock.now = 3.1
    sup.scan_seen()
    assert sup.snapshot()["reason"] == "no robot health for 3.1 s"


def test_scan_older_than_one_second_fails():
    sup, clock = make()
    healthy(sup)
    clock.now = 1.0
    sup.set_health(GOOD_HEALTH)
    assert sup.snapshot()["state"] == "ready"
    assert details(sup)["scan"] == "1.0 s"
    clock.now = 1.1
    sup.set_health(GOOD_HEALTH)
    assert sup.snapshot()["reason"] == "no LiDAR scan for 1 s"


def test_battery_at_exactly_6_8_volts_passes():
    sup, _ = make()
    healthy(sup)
    sup.set_voltage(6.8)
    assert details(sup)["battery"] == "6.80 V"
    assert sup.snapshot()["state"] == "ready"


@pytest.mark.parametrize("remote_value", ["1", "True", True])
def test_rc_valid_one_or_true_passes(remote_value):
    sup, _ = make()
    healthy(sup, {**GOOD_HEALTH, "rc_valid": remote_value})
    assert details(sup)["remote"] == str(remote_value)
    assert sup.snapshot()["state"] == "ready"


def test_first_failing_check_in_checks_order_is_the_reason():
    sup, _ = make()
    healthy(sup, {**GOOD_HEALTH, "balance_state": "Off"})
    sup.set_nav_ready(False)
    assert sup.snapshot()["reason"] == "balance is Off"


def test_request_goal_refused_with_reason_when_not_ready():
    sup, _ = make()
    healthy(sup)
    sup.set_nav_ready(False)
    assert sup.request_goal() == "Nav2 is not ready"
    assert sup.snapshot()["state"] == "not_ready"


def test_not_ready_becomes_ready_when_the_check_clears():
    sup, _ = make()
    healthy(sup)
    sup.set_nav_ready(False)
    sup.set_nav_ready(True)
    assert sup.snapshot()["state"] == "ready"


def test_request_goal_from_ready_drives_and_second_request_replaces_the_goal():
    sup, _ = make()
    healthy(sup)
    assert sup.request_goal() is None
    assert sup.snapshot()["state"] == "driving"
    assert sup.snapshot()["reason"] is None
    assert sup.request_goal() is None
    assert sup.snapshot()["state"] == "driving"


def test_goal_ended_while_driving_returns_to_ready():
    sup, _ = make()
    healthy(sup)
    sup.request_goal()
    sup.goal_ended()
    assert sup.snapshot()["state"] == "ready"


def test_goal_ended_with_a_failing_check_is_not_ready():
    sup, _ = make()
    healthy(sup)
    sup.request_goal()
    sup.set_voltage(6.0)
    sup.goal_ended()
    assert sup.snapshot()["state"] == "not_ready"
    assert sup.snapshot()["reason"] == "battery at 6.00 V, below 6.8 V"


def test_check_is_quiet_while_all_checks_pass():
    sup, _ = make()
    healthy(sup)
    sup.request_goal()
    assert sup.check() is False
    assert sup.snapshot()["state"] == "driving"


def test_failing_check_while_driving_stops_the_goal_once():
    sup, _ = make()
    healthy(sup)
    sup.request_goal()
    sup.set_localization("lost")
    assert sup.check() is True
    assert sup.check() is False
    snap = sup.snapshot()
    assert snap["state"] == "stopped"
    assert snap["reason"] == "localization is lost"


def test_request_goal_refused_while_stopped_until_resume():
    sup, _ = make()
    healthy(sup)
    sup.request_goal()
    sup.set_localization("lost")
    sup.check()
    assert sup.request_goal() == "stopped: localization is lost; press Resume"


def test_stop_while_driving_cancels_the_goal_once():
    sup, _ = make()
    healthy(sup)
    sup.request_goal()
    sup.stop()
    snap = sup.snapshot()
    assert snap["state"] == "stopped"
    assert snap["reason"] == "stopped by the user"
    assert sup.check() is True
    assert sup.check() is False


def test_stop_from_ready_does_not_cancel_anything():
    sup, _ = make()
    healthy(sup)
    sup.stop()
    assert sup.snapshot()["state"] == "stopped"
    assert sup.check() is False


def test_stop_from_not_ready_is_stopped():
    sup, _ = make()
    sup.stop()
    assert sup.snapshot()["state"] == "stopped"
    assert sup.snapshot()["reason"] == "stopped by the user"


def test_stop_while_already_stopped_keeps_the_first_reason():
    sup, _ = make()
    healthy(sup)
    sup.request_goal()
    sup.set_localization("lost")
    sup.check()
    sup.stop()
    assert sup.snapshot()["reason"] == "localization is lost"


def test_goal_ended_while_stopped_changes_nothing():
    sup, _ = make()
    healthy(sup)
    sup.stop()
    sup.goal_ended()
    assert sup.snapshot()["state"] == "stopped"


def test_resume_refused_while_a_check_fails():
    sup, _ = make()
    healthy(sup)
    sup.stop()
    sup.set_nav_ready(False)
    assert sup.resume() == "cannot resume: Nav2 is not ready"
    assert sup.snapshot()["state"] == "stopped"


def test_resume_returns_to_ready_when_all_checks_pass():
    sup, _ = make()
    healthy(sup)
    sup.stop()
    sup.set_nav_ready(False)
    sup.resume()
    sup.set_nav_ready(True)
    assert sup.resume() is None
    snap = sup.snapshot()
    assert snap["state"] == "ready"
    assert snap["reason"] is None


def test_resume_outside_stopped_changes_nothing():
    sup, _ = make()
    healthy(sup)
    assert sup.resume() is None
    assert sup.snapshot()["state"] == "ready"


def test_resume_after_stop_while_driving_still_cancels_the_goal_once():
    sup, _ = make()
    healthy(sup)
    sup.request_goal()
    sup.stop()
    assert sup.resume() is None
    assert sup.snapshot()["state"] == "ready"
    assert sup.check() is True
    assert sup.check() is False


def test_health_stale_after_driving_stops_goal_on_check():
    sup, clock = make()
    healthy(sup)
    sup.request_goal()
    clock.now = 3.5
    assert sup.check() is True
    assert sup.snapshot()["reason"] == "no robot health for 3.5 s"


def test_snapshot_lists_checks_in_checks_order():
    sup, _ = make()
    healthy(sup)
    assert tuple(c["name"] for c in sup.snapshot()["checks"]) == CHECKS
