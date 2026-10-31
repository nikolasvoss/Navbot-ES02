from threading import Lock
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("std_srvs.srv")
pytest.importorskip("tf2_ros")
pytest.importorskip("nav2_msgs.action")

from lds08rr_lidar.ros_node import LidarNode, firmware_values, nav_outcome, reduce_plan


class FakeFuture:
    def __init__(self):
        self.callbacks = []
        self.done = False

    def add_done_callback(self, callback):
        self.callbacks.append(callback)
        if self.done:  # rclpy runs a callback at once when the future is already complete
            callback(self)

    def finish(self):
        self.done = True
        for callback in self.callbacks:
            callback(self)

    def exception(self):
        return None


class FakeClient:
    def __init__(self, future, answers):
        self.future = future
        self.answers = answers  # True: the call completes; False: it stays pending
        self.removed = []

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        if self.answers:
            self.future.finish()
        return self.future

    def remove_pending_request(self, future):
        self.removed.append(future)
        return True


class FakeNode:
    def __init__(self, client):
        self._service_lock = Lock()
        self._service_clients = {}
        self._client = client

    def create_client(self, srv_type, name):
        return self._client


def test_call_service_returns_none_when_the_call_completes():
    client = FakeClient(FakeFuture(), answers=True)
    assert LidarNode.call_service(FakeNode(client), "/request_nomotion_update", 0.5) is None
    assert client.removed == []


def test_call_service_timeout_removes_the_pending_request():
    future = FakeFuture()
    client = FakeClient(future, answers=False)
    assert LidarNode.call_service(FakeNode(client), "/request_nomotion_update", 0.01) == "call timed out"
    assert client.removed == [future]


def test_firmware_values_takes_only_the_navbot_firmware_status():
    statuses = [
        SimpleNamespace(name="lds08rr", values=[SimpleNamespace(key="mode", value="live")]),
        SimpleNamespace(name="navbot_firmware", values=[
            SimpleNamespace(key="balance_state", value="Run"),
            SimpleNamespace(key="ch5_position", value="1"),
        ]),
    ]
    assert firmware_values(statuses) == {"balance_state": "Run", "ch5_position": "1"}


def test_firmware_values_is_none_without_the_navbot_firmware_status():
    assert firmware_values([SimpleNamespace(name="lds08rr", values=[])]) is None


def test_reduce_plan_keeps_short_plans_whole():
    assert reduce_plan([[0.0, 0.0], [0.4, -0.1]]) == [[0.0, 0.0], [0.4, -0.1]]


def test_reduce_plan_keeps_at_most_200_points_and_the_last_pose():
    points = [[float(i), 0.0] for i in range(250)]
    reduced = reduce_plan(points)
    assert len(reduced) == 126
    assert reduced[0] == [0.0, 0.0]
    assert reduced[-1] == [249.0, 0.0]


def test_reduce_plan_of_1000_points_has_168_points_ending_on_the_last_pose():
    points = [[float(i), 0.0] for i in range(1000)]
    reduced = reduce_plan(points)
    assert len(reduced) == 168
    assert reduced[1] == [6.0, 0.0]
    assert reduced[-1] == [999.0, 0.0]


def test_nav_outcome_maps_each_result_status():
    assert nav_outcome(4, "") == ("reached", "")
    assert nav_outcome(5, "") == ("canceled", "")
    assert nav_outcome(6, "Failed to find a valid path") == ("failed", "Failed to find a valid path")


class FakeNavClient:
    def __init__(self, ready):
        self.ready = ready

    def server_is_ready(self):
        return self.ready


def test_send_nav_goal_is_refused_while_nav2_is_not_ready():
    node = FakeNode(None)
    node._nav_client = FakeNavClient(ready=False)
    assert LidarNode.send_nav_goal(node, 1.2, -0.4, 1.571, None, None) == "Nav2 is not ready"


class FakeSupervisor:
    def __init__(self):
        self.localization = None
        self.nav_ready = None
        self.checks = 0

    def set_localization(self, state):
        self.localization = state

    def set_nav_ready(self, ready):
        self.nav_ready = ready

    def check(self):
        self.checks += 1
        return self.checks == 1  # True exactly once, at the stop


class FakeLocalization:
    def snapshot(self):
        return {"state": "localized"}


class FakeTickNode:
    def __init__(self):
        self.supervisor = FakeSupervisor()
        self.localization = FakeLocalization()
        self.cancels = 0

    def nav_server_ready(self):
        return True

    def cancel_nav_goal(self):
        self.cancels += 1


def test_supervisor_tick_cancels_the_goal_once_when_check_says_so():
    node = FakeTickNode()
    LidarNode.on_supervisor_tick(node)
    LidarNode.on_supervisor_tick(node)
    assert node.cancels == 1
    assert node.supervisor.localization == "localized"
    assert node.supervisor.nav_ready is True
