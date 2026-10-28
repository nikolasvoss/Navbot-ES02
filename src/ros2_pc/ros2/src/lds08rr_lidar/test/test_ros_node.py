from threading import Lock

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("std_srvs.srv")
pytest.importorskip("tf2_ros")

from lds08rr_lidar.ros_node import LidarNode


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
