"""The live mode of navbot_sim: the NavBot link over TCP, as the PC library uses it on the robot's port."""

import importlib.util
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from navbot_link.link import Link  # noqa: E402

SIGNALS_DEF = ROOT / "src" / "ES-02" / "OllieFOCdrive" / "signals.def.h"
_SPEC = importlib.util.spec_from_file_location("signals_catalog", ROOT / "scripts" / "signals_catalog.py")
signals_catalog = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(signals_catalog)


def free_port():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def wait_for_port(process, port):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        assert process.poll() is None, "navbot_sim exited before it listened"
        try:
            socket.create_connection(("127.0.0.1", port), timeout=0.2).close()
            return
        except OSError:
            time.sleep(0.05)
    raise AssertionError("navbot_sim did not listen")


def wait_until(condition, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.05)
    return False


@pytest.fixture
def live_port(sim_binary, tmp_path):
    port = free_port()
    scenario = tmp_path / "scenario.txt"
    scenario.write_text("0 rc 5 1000\n3600 end\n")  # CH5 at position 1, the link
    with (tmp_path / "stderr.txt").open("w") as stderr:
        process = subprocess.Popen(
            [str(sim_binary), "--live", "--listen", f"127.0.0.1:{port}", "--scenario", str(scenario)],
            stdout=subprocess.DEVNULL,
            stderr=stderr,
        )
        try:
            wait_for_port(process, port)
            yield port
        finally:
            process.terminate()
            process.wait(timeout=5)


def test_catalog_is_the_entries_of_signals_def(live_port):
    with Link(f"tcp:127.0.0.1:{live_port}", 0) as link:
        names = [entry.name for entry in link.catalog().entries]
    expected = [entry.name for entry in signals_catalog.parse(SIGNALS_DEF.read_text(encoding="utf-8"))]
    assert names == expected


def test_balance_call_starts_the_controller(live_port):
    with Link(f"tcp:127.0.0.1:{live_port}", 0) as link:
        assert link.get("balance_state") == 0
        link.call("balance", on=1)
        assert wait_until(lambda: link.get("balance_state") == 2, 5.0)


def test_command_reaches_the_controller(live_port):
    with Link(f"tcp:127.0.0.1:{live_port}", 0) as link:
        link.call("balance", on=1)
        assert wait_until(lambda: link.get("balance_state") == 2, 5.0)
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            link.command(0.1, 0.0)
            time.sleep(0.1)
            assert link.get("cmd_speed_mps") == pytest.approx(0.1)
        assert link.get("ch3_target") == pytest.approx(0.1)
