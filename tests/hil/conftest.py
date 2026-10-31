"""Hardware-in-the-loop tests: one link per session, to the fake firmware or to the robot with --robot PORT.

Without --robot the tests run against the fake firmware. With --robot and without --guided, the gate signals are
read before the first test; a robot that is not safe for unattended tests ends the session with exit code 4.
"""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
for path in (ROOT / "src", ROOT / "tests" / "navbot_link", ROOT / "scripts"):
    sys.path.insert(0, str(path))

import devtools  # noqa: E402
from fake_firmware import FakeFirmware, MemoryTransport, device_table  # noqa: E402
from navbot_link.link import Link  # noqa: E402
from navbot_link.protocol import UnknownName  # noqa: E402

USB_BAUD = 576_000
UART_BAUD = 2_000_000
GATE_EXIT_CODE = 4
GATE_SIGNALS = ("balance_state", "motor_test_active", "rc_valid", "ch5_position")


@pytest.fixture(scope="session")
def fake(request):
    """The fake robot for this session, or None when the tests run on a real robot."""
    if request.config.getoption("--robot") is not None:
        return None
    return FakeFirmware(entries=device_table())


@pytest.fixture(scope="session")
def link(request, fake):
    """The one link of the session. A USB or ACM port runs at 576000 baud, anything else at 2000000."""
    port = request.config.getoption("--robot")
    if port is None:
        link = Link(MemoryTransport(fake))
    else:
        link = Link(port, USB_BAUD if "USB" in port or "ACM" in port else UART_BAUD)
    yield link
    link.close()


def gate_needed(items, guided: bool) -> bool:
    """Whether the gate runs. With --guided it is skipped only when every collected test is guided, since the user reads the gate then."""
    if not guided:
        return True
    return not all("guided" in item.keywords for item in items)


@pytest.fixture(scope="session", autouse=True)
def robot_gate(request, link):
    """Before the first test of a robot session: refuse to run unless the robot is safe, as deploy.py does."""
    if request.config.getoption("--robot") is None:
        return
    if not gate_needed(request.session.items, request.config.getoption("--guided")):
        return
    values = {}
    for name in GATE_SIGNALS:
        try:
            values[name] = link.get(name)
        except UnknownName:
            values[name] = None
    reason = devtools.gate_ok(values)
    if reason is not None:
        pytest.exit(f"robot not safe for unattended tests: {reason}", returncode=GATE_EXIT_CODE)
