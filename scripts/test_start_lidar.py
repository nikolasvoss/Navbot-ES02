from __future__ import annotations

import importlib.util
import json
import signal
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/start_lidar.py"
SPEC = importlib.util.spec_from_file_location("start_lidar", SCRIPT)
START_LIDAR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(START_LIDAR)


def test_help_is_available_without_ros_or_sensor():
    result = subprocess.run([sys.executable, str(SCRIPT), "--help"], text=True, capture_output=True, check=False)
    assert result.returncode == 0
    assert "--demo" in result.stdout
    assert "--protocol" in result.stdout
    assert "--ssh HOST" in result.stdout
    assert "--remote-path" in result.stdout


def test_ssh_command_quotes_remote_path_and_forwards_port():
    command, url = START_LIDAR.ssh_invocation(
        "niko@cm5", "/home/niko/Navbot with spaces", ["--demo", "--port", "8181"]
    )
    assert command[:5] == ["ssh", "-o", "ExitOnForwardFailure=yes", "-L", "8181:127.0.0.1:8181"]
    assert command[-2] == "niko@cm5"
    assert "'/home/niko/Navbot with spaces'" in command[-1]
    assert '"$HOME/.venvs/navbot-lidar/bin/python"' in command[-1]
    assert "source /opt/ros/jazzy/setup.bash" in command[-1]
    assert "--host 127.0.0.1" in command[-1]
    assert url == "http://127.0.0.1:8181/lidar"


def test_ssh_does_not_accept_lan_bind():
    try:
        START_LIDAR.ssh_invocation("cm5", "/home/niko/Navbot", ["--demo", "--host", "0.0.0.0"])
    except ValueError as exc:
        assert "loopback" in str(exc)
    else:
        raise AssertionError("SSH forwarding must keep the remote HTTP server on loopback")


@pytest.mark.parametrize("target_args", [
    ["--ssh=cm5", "--remote-path=/tmp/Navbot"],
    ["--ssh", "cm5", "--remote-path=/tmp/Navbot"],
])
def test_ssh_equals_arguments_dispatch(monkeypatch, target_args):
    commands = []

    def launch(command, **kwargs):
        commands.append(command)
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(START_LIDAR.subprocess, "run", launch)
    assert START_LIDAR.main([*target_args, "--demo"]) == 0
    assert commands[0][-2] == "cm5"
    assert '.venvs/navbot-lidar/bin/python' in commands[0][-1]
    assert 'scripts/start_lidar.py --demo' in commands[0][-1]


def free_port():
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        return listener.getsockname()[1]


def wait_for_status(url, timeout=8.0):
    end = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < end:
        try:
            with urllib.request.urlopen(url, timeout=1.0) as response:
                return json.load(response)
        except Exception as exc:
            last_error = exc
            time.sleep(0.1)
    raise AssertionError(f"LiDAR service did not become ready: {last_error}")


def wait_for_scan(url, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        status = wait_for_status(url, timeout=max(0.1, end - time.monotonic()))
        if status["scans_received"] > 0:
            return status
        time.sleep(0.1)
    raise AssertionError("demo service did not publish a complete scan")


def wait_for_state(url, expected, timeout=5.0):
    end = time.monotonic() + timeout
    status = None
    while time.monotonic() < end:
        status = wait_for_status(url, timeout=max(0.1, end - time.monotonic()))
        if status["state"] == expected:
            return status
        time.sleep(0.1)
    raise AssertionError(f"expected LiDAR state {expected}, got {status}")


def test_demo_browser_hides_stale_plot_and_reports_server_loss():
    playwright = pytest.importorskip("playwright.sync_api")
    port = free_port()
    base = f"http://127.0.0.1:{port}"
    process = subprocess.Popen(
        [sys.executable, str(SCRIPT), "--demo", "--ros", "off", "--port", str(port)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        status = wait_for_scan(f"{base}/status.json")
        assert status["mode"] == "demo"
        assert urllib.request.urlopen(f"{base}/").status == 200
        assert urllib.request.urlopen(f"{base}/lidar").status == 200
        assert urllib.request.urlopen(f"{base}/scan.json").status == 200

        with playwright.sync_playwright() as browser_driver:
            browser = browser_driver.chromium.launch(headless=True)
            page = browser.new_page()
            page.goto(f"{base}/lidar")
            page.get_by_text("DEMO DATA · synthetic scan").wait_for(timeout=5000)
            page.locator("#plot").wait_for(state="visible", timeout=5000)
            page.set_viewport_size({"width": 390, "height": 844})
            page.locator("summary").click()
            assert page.evaluate("document.documentElement.scrollWidth <= innerWidth")
            page.locator("summary").click()
            page.route("**/status.json", lambda route: route.fulfill(
                status=200,
                content_type="application/json",
                body=json.dumps({
                    "state": "stale", "mode": "serial", "device": "/dev/ttyAMA2",
                    "protocol": "camsense", "bytes_received": 60, "scans_received": 1,
                    "last_scan_age_sec": 3.0, "scan_rate_hz": 5.0,
                    "valid_points": 0, "last_error": None,
                }),
            ))
            page.locator("#plotState").wait_for(state="visible", timeout=3000)
            page.locator("#plot").wait_for(state="hidden", timeout=3000)
            page.route("**/status.json", lambda route: route.fulfill(
                status=200,
                content_type="application/json",
                body=json.dumps({
                    "state": "stale", "mode": "demo", "device": None,
                    "protocol": None, "bytes_received": 0, "scans_received": 1,
                    "last_scan_age_sec": 3.0, "scan_rate_hz": 5.0,
                    "valid_points": 0, "last_error": None,
                }),
            ))
            page.locator("#summary").get_by_text("synthetic scan is not currently fresh").wait_for(timeout=3000)
            page.locator("#plot").wait_for(state="hidden", timeout=1000)
            page.unroute("**/status.json")
            process.terminate()
            process.wait(timeout=3)
            page.locator("#modeBadge").get_by_text("SERVER UNAVAILABLE", exact=True).wait_for(timeout=7000)
            page.locator("#plot").wait_for(state="hidden", timeout=1000)
            browser.close()
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3)


def test_missing_serial_device_keeps_status_page_available():
    port = free_port()
    base = f"http://127.0.0.1:{port}"
    process = subprocess.Popen(
        [sys.executable, str(SCRIPT), "--device", "/dev/navbot-lidar-not-present",
         "--protocol", "camsense", "--ros", "off", "--port", str(port)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        status = wait_for_state(f"{base}/status.json", "error")
        assert "navbot-lidar-not-present" in status["last_error"]
        assert urllib.request.urlopen(f"{base}/lidar").status == 200
    finally:
        process.terminate()
        process.wait(timeout=3)


def test_second_service_fails_before_starting_a_duplicate_reader():
    port = free_port()
    command = [sys.executable, str(SCRIPT), "--demo", "--ros", "off", "--port", str(port)]
    first = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    try:
        wait_for_status(f"http://127.0.0.1:{port}/status.json")
        second = subprocess.run(command, text=True, capture_output=True, timeout=5, check=False)
        assert second.returncode != 0
        assert "port" in second.stderr.lower() and "in use" in second.stderr.lower()
    finally:
        first.terminate()
        first.wait(timeout=3)


def test_ros_subscriber_receives_demo_and_ctrl_c_exits_cleanly():
    rclpy = pytest.importorskip("rclpy")
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    context = rclpy.context.Context()
    rclpy.init(context=context)
    observer = rclpy.create_node("lidar_acceptance_observer", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(observer)
    scans, statuses = [], []
    observer.create_subscription(LaserScan, "/scan", scans.append, qos_profile_sensor_data)
    observer.create_subscription(DiagnosticArray, "/lidar/status", statuses.append, 10)
    port = free_port()
    process = subprocess.Popen(
        [sys.executable, str(SCRIPT), "--demo", "--ros", "on", "--port", str(port)],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    try:
        wait_for_state(f"http://127.0.0.1:{port}/status.json", "demo")
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and not (scans and statuses):
            executor.spin_once(timeout_sec=0.1)
        assert scans and statuses
        assert scans[-1].header.frame_id == "lds08rr"
        assert len(scans[-1].ranges) == 360
        assert any(0.1 < distance < 6.0 for distance in scans[-1].ranges)
        assert scans[-1].scan_time == pytest.approx(0.2)
        fields = {item.key: item.value for item in statuses[-1].status[0].values}
        assert fields["mode"] == "demo"
        process.send_signal(signal.SIGINT)
        stdout, stderr = process.communicate(timeout=5)
        assert process.returncode == 0, stderr
        assert "Traceback" not in stderr
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3)
        executor.shutdown()
        observer.destroy_node()
        context.try_shutdown()
