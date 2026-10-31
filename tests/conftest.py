"""Shared pytest setup for every area under tests/.

Options and markers live here because pytest reads options only from the first conftest on the path.
Every tests/<area>/run_host_test.sh is collected as one item, so the host C++ checks run in the same pass.
"""

import subprocess
from pathlib import Path

import pytest

HOST_TEST_SCRIPT = "run_host_test.sh"
OUTPUT_TAIL_LINES = 40


def pytest_addoption(parser: pytest.Parser) -> None:
    """Register the robot port and the guided switch."""
    parser.addoption("--robot", metavar="PORT", default=None, help="serial port or tcp:host:port of the robot; default: fake firmware")
    parser.addoption("--guided", action="store_true", default=False, help="run tests that need the user at the robot")


def pytest_configure(config: pytest.Config) -> None:
    """Declare the markers used by the hardware tests."""
    config.addinivalue_line("markers", "guided: needs the user at the robot; runs only with --guided")
    config.addinivalue_line("markers", "robot: needs a robot; runs only with --robot")


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    """Skip guided tests without --guided and robot tests without --robot."""
    skip_guided = pytest.mark.skip(reason="guided test: run with --guided")
    skip_robot = pytest.mark.skip(reason="needs a robot: run with --robot PORT")
    for item in items:
        if "guided" in item.keywords and not config.getoption("--guided"):
            item.add_marker(skip_guided)
        if "robot" in item.keywords and config.getoption("--robot") is None:
            item.add_marker(skip_robot)


def pytest_collect_file(file_path: Path, parent: pytest.Collector) -> pytest.Collector | None:
    """Collect tests/<area>/run_host_test.sh as a host test file."""
    if file_path.name == HOST_TEST_SCRIPT:
        return HostTestFile.from_parent(parent, path=file_path)
    return None


class HostTestFile(pytest.File):
    """A run_host_test.sh script; yields one item that runs it."""

    def collect(self):
        yield HostTestItem.from_parent(self, name=f"{self.path.parent.name}/{HOST_TEST_SCRIPT}")


class HostTestItem(pytest.Item):
    """Runs a host test script with bash and fails with the tail of its output."""

    def runtest(self) -> None:
        script = Path(self.path)
        result = subprocess.run(
            ["bash", str(script)],
            cwd=script.parent,
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            output = (result.stdout + result.stderr).splitlines()
            tail = "\n".join(output[-OUTPUT_TAIL_LINES:])
            pytest.fail(f"{script} exited with {result.returncode}\n{tail}", pytrace=False)

    def reportinfo(self):
        return self.path, None, self.name
