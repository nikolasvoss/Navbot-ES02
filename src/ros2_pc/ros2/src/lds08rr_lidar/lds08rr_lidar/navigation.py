"""Paths and command line of the Nav2 navigation launch. ROS-free."""

from __future__ import annotations

from pathlib import Path

NAVIGATION_LAUNCH = Path(__file__).with_name("navigation_launch.py")
NAVIGATION_PARAMS = Path(__file__).with_name("navigation.yaml")


def navigation_command(keepout_yaml: Path | None) -> list[str]:
    """The ros2 launch command of Nav2; without a keepout mask, no mask servers start."""
    command = ["ros2", "launch", str(NAVIGATION_LAUNCH), f"params_file:={NAVIGATION_PARAMS}"]
    # ros2 launch rejects an empty "name:=" value, so no mask means leaving the argument at its empty default.
    if keepout_yaml is not None:
        command.append(f"keepout_mask:={keepout_yaml}")
    return command
