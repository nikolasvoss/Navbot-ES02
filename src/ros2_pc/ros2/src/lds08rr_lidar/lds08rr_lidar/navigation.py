"""Paths and command lines of the Nav2 navigation launch and the bench launch. ROS-free."""

from __future__ import annotations

from pathlib import Path

from .localization import LOCALIZATION_PARAMS

NAVIGATION_LAUNCH = Path(__file__).with_name("navigation_launch.py")
NAVIGATION_PARAMS = Path(__file__).with_name("navigation.yaml")
BENCH_LAUNCH = Path(__file__).with_name("bench_launch.py")


def navigation_command(keepout_yaml: Path | None) -> list[str]:
    """The ros2 launch command of Nav2; an empty keepout_mask starts no mask servers."""
    raise NotImplementedError


def bench_command(map_yaml: Path) -> list[str]:
    """The ros2 launch command of the bench: localization, loopback simulator and fake robot."""
    raise NotImplementedError
