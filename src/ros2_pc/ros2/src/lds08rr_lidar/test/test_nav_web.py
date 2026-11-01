import math

import pytest

pytest.importorskip("flask")

from lds08rr_lidar.runtime import LidarRuntime
from lds08rr_lidar.slam import SlamState
from lds08rr_lidar.source import SourceConfig
from lds08rr_lidar.web import make_lidar_app


class FakeNavControl:
    def __init__(self):
        self.goal_error = None
        self.cancel_error = None
        self.resume_error = None
        self.goals = []
        self.cancels = 0
        self.stops = 0
        self.resumes = 0

    def nav_goal(self, x, y, yaw):
        self.goals.append((x, y, yaw))
        return self.goal_error

    def nav_cancel(self):
        self.cancels += 1
        return self.cancel_error

    def nav_stop(self):
        self.stops += 1

    def nav_resume(self):
        self.resumes += 1
        return self.resume_error


def make_client(tmp_path, control):
    runtime = LidarRuntime(SourceConfig(mode="demo"), driver_factory=lambda *_: None)
    return make_lidar_app(runtime, slam=SlamState(), control=control, radar_web=tmp_path / "missing").test_client()


def test_nav_routes_answer_with_literal_json(tmp_path):
    control = FakeNavControl()
    client = make_client(tmp_path, control)

    assert client.post("/nav/goal", json={"x": 1.2, "y": -0.4, "yaw": 1.571}).get_json() == {"ok": True}
    assert control.goals == [(1.2, -0.4, 1.571)]

    control.goal_error = "Goals need navigation mode"
    response = client.post("/nav/goal", json={"x": 1, "y": 2, "yaw": 0})
    assert response.status_code == 409
    assert response.get_json() == {"ok": False, "error": "Goals need navigation mode"}

    assert client.post("/nav/cancel").get_json() == {"ok": True}
    control.cancel_error = "No goal is running"
    response = client.post("/nav/cancel")
    assert response.status_code == 409
    assert response.get_json() == {"ok": False, "error": "No goal is running"}

    assert client.post("/nav/stop").get_json() == {"ok": True}
    assert control.stops == 1

    control.resume_error = "cannot resume: CH5 is in position 2, not 1"
    response = client.post("/nav/resume")
    assert response.status_code == 409
    assert response.get_json() == {"ok": False, "error": "cannot resume: CH5 is in position 2, not 1"}
    control.resume_error = None
    assert client.post("/nav/resume").get_json() == {"ok": True}
    assert control.resumes == 2


def test_a_goal_body_without_numbers_reaches_nav_goal_as_nan(tmp_path):
    control = FakeNavControl()
    client = make_client(tmp_path, control)

    client.post("/nav/goal", json={"x": 1.5, "y": "2", "yaw": True})
    client.post("/nav/goal", json=[1, 2, 3])
    client.post("/nav/goal", data="not json")

    x, y, yaw = control.goals[0]
    assert x == 1.5 and math.isnan(y) and math.isnan(yaw)
    assert all(math.isnan(value) for value in control.goals[1])
    assert all(math.isnan(value) for value in control.goals[2])


def test_every_nav_route_is_a_409_without_a_running_control(tmp_path):
    client = make_client(tmp_path, None)
    refused = {"ok": False, "error": "SLAM is not running on this service"}
    for path in ("/nav/goal", "/nav/cancel", "/nav/stop", "/nav/resume"):
        response = client.post(path)
        assert response.status_code == 409
        assert response.get_json() == refused


def test_nav_routes_refuse_a_cross_site_post(tmp_path):
    control = FakeNavControl()
    client = make_client(tmp_path, control)
    for path in ("/nav/goal", "/nav/cancel", "/nav/stop", "/nav/resume"):
        response = client.post(path, headers={"Origin": "http://evil.example"})
        assert response.status_code == 403
    assert control.goals == []
    assert (control.cancels, control.stops, control.resumes) == (0, 0, 0)


def test_hub_page_has_the_mode_buttons_the_navigation_panel_and_the_map_actions(tmp_path):
    html = make_client(tmp_path, FakeNavControl()).get("/slam").get_data(as_text=True)
    for element_id in (
        "modeButtons", "modeMapping", "modeLocalization", "modeNavigation", "locate",
        "navPanel", "navState", "navChecks", "navDistance", "navOutcome",
        "setGoal", "cancelGoal", "stopNav", "resumeNav",
        "saveMap", "resetMap", "deleteMap", "undoSelect", "undoMap",
    ):
        assert f'id="{element_id}"' in html
    assert 'id="switchMode"' not in html


