import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lds2d.viz import ScanBuffer

from lds08rr_lidar.health import ServiceMonitor
from lds08rr_lidar.web import make_lidar_app


class FakeRuntime:
    def __init__(self, state="live", last_error=None):
        self.state, self.last_error = state, last_error
        self.scan_buffer = ScanBuffer()

    def snapshot(self):
        return SimpleNamespace(state=self.state, last_error=self.last_error)

    def status_dict(self):
        return {"state": self.state, "last_error": self.last_error}


class FakeControl:
    def __init__(self, state="running", exit_code=None, error=None, mode="mapping", localization=None):
        self.process = {"state": state, "exit_code": exit_code, "error": error}
        self.mode = mode
        self.localization = localization

    def status(self):
        return {"process": self.process, "mode": self.mode, "localization": self.localization}


def localization(state, error=None, hint=None):
    return {"state": state, "pose": None, "std": None, "search_age_sec": None, "error": error, "hint": hint}


class FakeRos:
    def __init__(self, ages=None, publishers=None, nodes=()):
        self.ages = {"/odom": 0.1, "/imu/data": 0.1} if ages is None else ages
        self.publishers = {"/odom": 1, "/imu/data": 1} if publishers is None else publishers
        self.nodes = nodes

    def topic_age(self, topic):
        return self.ages.get(topic)

    def publisher_count(self, topic):
        return self.publishers.get(topic, 0)

    def node_exists(self, name):
        return name in self.nodes


class FakeSlam:
    def __init__(self, state="mapping"):
        self.state = state

    def snapshot(self):
        return {"state": self.state}


def rows(runtime=None, control=None, ros=None, slam=None, port_open=lambda port: False):
    monitor = ServiceMonitor(
        runtime or FakeRuntime(), control or FakeControl(), ros or FakeRos(), slam=slam, port_open=port_open,
    )
    return {row["key"]: row for row in monitor.snapshot()}


def test_everything_healthy():
    result = rows(ros=FakeRos(nodes=("hmmd_sensor",)), port_open=lambda port: True)
    assert {key: row["level"] for key, row in result.items()} == {
        "lidar": "ok", "robot_link": "ok", "slam": "ok", "radar": "ok",
    }


def test_lidar_states():
    for state, short in (("waiting", "No data"), ("invalid", "Invalid data"), ("stale", "Stale")):
        lidar = rows(runtime=FakeRuntime(state))["lidar"]
        assert (lidar["level"], lidar["short"]) == ("error", short)
    demo = rows(runtime=FakeRuntime("demo"))["lidar"]
    assert (demo["level"], demo["short"]) == ("warn", "Demo")
    error = rows(runtime=FakeRuntime("error", "SerialException: gone"))["lidar"]
    assert error["level"] == "error" and "SerialException: gone" in error["message"]


def test_robot_link_bridge_not_running():
    ros = FakeRos(ages={}, publishers={})
    link = rows(ros=ros)["robot_link"]
    assert link["level"] == "error" and "not running" in link["message"]


def test_robot_link_bridge_running_without_robot_data():
    ros = FakeRos(ages={"/odom": 30.0, "/imu/data": None})
    link = rows(ros=ros)["robot_link"]
    assert link["level"] == "error"
    assert "/odom or /imu/data" in link["message"] and "UART" in link["message"]


def test_slam_down_and_missing():
    down = rows(control=FakeControl("restarting", exit_code=1))["slam"]
    assert down["level"] == "warn" and down["short"] == "Restarting"
    assert "restarting" in down["message"] and "exit code 1" in down["message"]
    stopped = rows(control=FakeControl("stopped"))["slam"]
    assert (stopped["level"], stopped["short"]) == ("error", "Not running")
    stale = rows(slam=FakeSlam("stale"))["slam"]
    assert (stale["level"], stale["short"]) == ("error", "Stale")
    assert rows(slam=FakeSlam("waiting"))["slam"]["level"] == "ok"
    missing = ServiceMonitor(FakeRuntime(), control=None, ros=None).snapshot()
    assert [row["key"] for row in missing] == ["lidar", "slam"]
    assert missing[1]["level"] == "warn"


def test_radar_reports_what_is_missing_as_a_warning():
    radar = rows()["radar"]
    assert radar["level"] == "warn" and radar["short"] == "Not running"
    assert "hmmd_sensor ROS node" in radar["message"] and "rosbridge" in radar["message"]
    assert "rosbridge" not in rows(ros=FakeRos(nodes=("hmmd_sensor",)), port_open=lambda port: True)["radar"]["message"]


def test_services_endpoint_and_nav_chips():
    app = make_lidar_app(FakeRuntime("waiting"), control=FakeControl(), ros=FakeRos(), radar_web=None)
    client = app.test_client()
    payload = client.get("/services.json").get_json()
    assert payload[0]["key"] == "lidar" and payload[0]["level"] == "error"
    status = client.get("/status.json").get_json()
    assert (status["state"], status["level"], status["label"]) == ("waiting", "error", "No data")
    for page in ("/lidar", "/slam"):
        assert 'data-svc="robot_link"' in client.get(page).get_data(as_text=True)


def test_short_state_words_for_the_status_chips():
    result = rows(runtime=FakeRuntime("waiting"), ros=FakeRos(ages={}, publishers={}), control=FakeControl("restarting"))
    assert result["lidar"]["short"] == "No data"
    assert result["robot_link"]["short"] == "Not running"
    assert result["slam"]["short"] == "Restarting"
    assert result["radar"]["short"] == "Not running"
    assert rows(ros=FakeRos(ages={"/odom": 30.0}))["robot_link"]["short"] == "No data"


def test_nav_is_one_layout_item_with_a_chip_per_service():
    from html.parser import HTMLParser

    from jinja2 import Environment, FileSystemLoader

    templates = Path(__file__).resolve().parents[1] / "lds08rr_lidar" / "templates"
    html = Environment(loader=FileSystemLoader(templates)).get_template("_nav.html").render(active="lidar")

    class TopLevel(HTMLParser):
        void = {"meta", "br", "img", "input", "link"}

        def __init__(self):
            super().__init__()
            self.depth, self.top, self.services = 0, [], []

        def handle_starttag(self, tag, attrs):
            if self.depth == 0:
                self.top.append(tag)
            if "data-svc" in dict(attrs):
                self.services.append(dict(attrs)["data-svc"])
            if tag not in self.void:
                self.depth += 1

        def handle_endtag(self, tag):
            self.depth -= 1

    parser = TopLevel()
    parser.feed(html)
    # The pages are CSS grids with fixed row templates: a second visible top-level item shifts every row.
    assert [tag for tag in parser.top if tag not in ("script", "style")] == ["nav"]
    assert sorted(parser.services) == ["lidar", "radar", "robot_link", "slam"]


def test_localization_row_follows_the_tracker_state():
    def slam_row(state, error=None, hint=None):
        control = FakeControl(mode="localization", localization=localization(state, error, hint))
        return rows(control=control, slam=FakeSlam("localization"))["slam"]

    localized = slam_row("localized")
    assert (localized["level"], localized["short"]) == ("ok", "Localized")
    searching = slam_row("searching")
    assert (searching["level"], searching["short"]) == ("warn", "Searching")
    lost = slam_row("lost", hint="Drive or turn the robot a little")
    assert (lost["level"], lost["short"]) == ("warn", "Lost")
    assert "Drive or turn" in lost["message"]
    failed = slam_row("lost", error="AMCL services did not appear")
    assert (failed["level"], failed["short"]) == ("error", "Lost")
    assert "AMCL services did not appear" in failed["message"]
    assert slam_row("idle")["short"] == "Starting"


def test_localization_row_ignores_map_age():
    result = rows(
        control=FakeControl(mode="localization", localization=localization("localized")), slam=FakeSlam("stale"),
    )["slam"]
    assert (result["level"], result["short"]) == ("ok", "Localized")


def test_localization_process_down_uses_the_process_rules():
    restarting = rows(control=FakeControl("restarting", exit_code=1, mode="localization", localization=localization("idle")))
    assert restarting["slam"]["level"] == "warn" and restarting["slam"]["short"] == "Restarting"
    assert "AMCL localization is restarting" in restarting["slam"]["message"]
    stopped = rows(control=FakeControl("stopped", mode="localization", localization=localization("lost")))
    assert (stopped["slam"]["level"], stopped["slam"]["short"]) == ("error", "Not running")


def test_slam_row_is_named_for_slam_and_localization():
    assert rows()["slam"]["name"] == "SLAM / localization"
