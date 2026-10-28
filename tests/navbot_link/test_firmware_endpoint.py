"""The PC library against the firmware's own link code (src/ES-02/OllieFOCdrive/link_endpoint.cpp and friends), built
for the host and run as a subprocess. The fake firmware tests show what the library expects; this shows that the
firmware code gives it."""

import os
import queue
import select
import shutil
import subprocess
import time
from pathlib import Path

import pytest

from navbot_link.link import Link, Row
from navbot_link.protocol import Kind, LinkError, Type

ROOT = Path(__file__).resolve().parents[2]
SKETCH = ROOT / "src" / "ES-02" / "OllieFOCdrive"
NANOPB = SKETCH / "src" / "navbot_link"

pytestmark = pytest.mark.skipif(shutil.which("cc") is None or shutil.which("c++") is None,
                                reason="no host C and C++ compiler")


@pytest.fixture(scope="module")
def server(tmp_path_factory):
    out = tmp_path_factory.mktemp("endpoint-server")
    objects = []
    for source in ("pb_common", "pb_encode", "pb_decode", "navbot_link.pb"):
        obj = out / f"{source}.o"
        subprocess.run(["cc", "-std=c99", "-c", str(NANOPB / f"{source}.c"), "-o", str(obj)], check=True)
        objects.append(str(obj))
    exe = out / "endpoint_server"
    subprocess.run(
        ["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", f"-I{SKETCH}",
         str(ROOT / "tests" / "navbot_link" / "endpoint_server.cpp"),
         *(str(SKETCH / f) for f in ("link_frame.cpp", "link_tx.cpp", "link_subscriptions.cpp", "link_endpoint.cpp")),
         *objects, "-o", str(exe)],
        check=True)
    return exe


class ProcessTransport:
    def __init__(self, exe):
        self.proc = subprocess.Popen([str(exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE)

    def read(self) -> bytes:
        ready, _, _ = select.select([self.proc.stdout], [], [], 0.01)
        return os.read(self.proc.stdout.fileno(), 4096) if ready else b""

    def write(self, data: bytes) -> None:
        self.proc.stdin.write(data)
        self.proc.stdin.flush()

    def close(self) -> None:
        self.proc.stdin.close()
        self.proc.wait(timeout=2)


@pytest.fixture
def link(server):
    with Link(ProcessTransport(server), timeout_s=1.0) as link:
        yield link


def rows(link, count, timeout_s=5.0):
    found, deadline = [], time.monotonic() + timeout_s
    while len(found) < count:
        try:
            event = link.events.get(timeout=max(0.0, deadline - time.monotonic()))
        except queue.Empty:
            pytest.fail(f"only {len(found)} of {count} rows arrived")
        if isinstance(event, Row):
            found.append(event)
    return found


def test_catalog_lists_every_entry_with_kind_type_unit_and_build(link):
    catalog = link.catalog()
    assert catalog.build == "server-1"
    assert [(e.name, e.unit, e.kind, e.type) for e in catalog.entries] == [
        ("tick", "", Kind.SIGNAL, Type.U32),
        ("ramp", "m", Kind.SIGNAL, Type.F32),
        ("negative", "", Kind.SIGNAL, Type.I32),
        ("odd", "", Kind.SIGNAL, Type.BOOL),
        ("falling_min", "V", Kind.SIGNAL_MIN, Type.F32),
        ("spike_max", "", Kind.SIGNAL_MAX, Type.U32),
        ("gain", "A/rad", Kind.PARAM, Type.F32),
        ("go", "", Kind.ACTION, None),
    ]
    go = catalog.action("go")
    assert [(a.name, a.min, a.max) for a in go.args] == [("left_a", -2.0, 2.0), ("lease_ms", 1.0, 1000.0)]


def test_get_returns_the_typed_value(link):
    assert link.get("gain") == 0.25


def test_rows_carry_typed_values_and_the_running_maximum(link):
    names = ["tick", "ramp", "negative", "odd", "falling_min", "spike_max"]
    subscription = link.subscribe(names, rate_hz=100)
    assert subscription.slot == 1
    got = rows(link, 8)
    previous = None
    for row in got:
        tick, ramp, negative, odd, falling_min, spike_max = row.values
        assert ramp == 0.5 * tick and negative == -tick and odd == bool(tick % 2)
        # falling_min counts down by one per tick, so the lowest value of a window is the current one
        assert falling_min == 100 - tick
        if previous is not None:
            assert 5000 <= (row.t_us - previous.t_us) % 2**32 <= 100000
            # The spike is on one tick in seven. Any window of 7 ticks holds one, which the running maximum reports.
            if tick - previous.values[0] >= 7:
                assert spike_max == 1000
        previous = row
    link.unsubscribe(subscription.slot)


def test_set_applies_a_value_and_refuses_one_out_of_range(link):
    link.catalog()
    assert link.set("gain", 0.5) == 0.5  # the robot's applied value
    assert link.get("gain") == 0.5
    with pytest.raises(LinkError, match=r"gain: must be within \[0, 1\]"):
        link.set("gain", 2.0)
    assert link.get("gain") == 0.5  # the running value stayed


def test_call_starts_an_action_and_refuses_a_bound_or_a_count(link):
    link.catalog()
    link.call("go", left_a=0.5, lease_ms=100)  # accepted: no value, no error
    with pytest.raises(LinkError, match=r"left_a=3 is outside \[-2, 2\]"):
        link.call("go", left_a=3.0, lease_ms=100)


def test_over_budget_subscription_is_refused_with_the_numbers(link):
    link.catalog()
    with pytest.raises(LinkError, match=r"over budget: \d+\.\d kB/s of 135\.0 kB/s allowed"):
        link.subscribe(["tick"] * 48, rate_hz=0)
