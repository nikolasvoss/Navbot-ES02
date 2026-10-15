#!/usr/bin/env python3
from __future__ import annotations

import os
import shlex
import subprocess
import sys
import tempfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = REPO_ROOT / "config/systemd/navbot-slam.service.in"
UNIT_PATH = "/etc/systemd/system/navbot-slam.service"
ROS_SETUP = Path("/opt/ros/jazzy/setup.bash")
WORKSPACE_SETUP = REPO_ROOT / "src/ros2_pc/ros2/install/setup.bash"
PACKAGE_SHARE = REPO_ROOT / "src/ros2_pc/ros2/install/lds08rr_lidar/share/lds08rr_lidar"
SLAM_TOOLBOX_LAUNCH = Path("/opt/ros/jazzy/share/slam_toolbox/launch/online_async_launch.py")


def systemd_quote(value: str) -> str:
    if "\n" in value or "\r" in value:
        raise ValueError("Paths containing newlines cannot be used in a systemd unit")
    escaped = value.replace("%", "%%").replace("$", "$$").replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'


def render_unit() -> str:
    required = (
        TEMPLATE,
        ROS_SETUP,
        WORKSPACE_SETUP,
        PACKAGE_SHARE / "launch/slam.launch.py",
        PACKAGE_SHARE / "config/slam_toolbox.yaml",
        SLAM_TOOLBOX_LAUNCH,
    )
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError("Required SLAM startup files are missing: " + ", ".join(missing))

    result = subprocess.run(
        ["/bin/bash", "-c", f"source {shlex.quote(str(ROS_SETUP))} && command -v ros2"],
        check=True,
        capture_output=True,
        text=True,
    )
    ros2 = result.stdout.strip()
    if not ros2 or not Path(ros2).is_file():
        raise FileNotFoundError("ROS 2 Jazzy setup did not provide a ros2 executable")

    shell_command = (
        f"source {shlex.quote(str(ROS_SETUP))} && "
        f"source {shlex.quote(str(WORKSPACE_SETUP))} && exec "
        + shlex.join([ros2, "launch", "lds08rr_lidar", "slam.launch.py"])
    )
    values = {
        "@UID@": str(os.getuid()),
        "@GID@": str(os.getgid()),
        "@EXEC_START@": "/bin/bash -c " + systemd_quote(shell_command),
    }
    rendered = TEMPLATE.read_text()
    for placeholder, value in values.items():
        rendered = rendered.replace(placeholder, value)
    return rendered


def main() -> int:
    if len(sys.argv) != 1:
        print("Usage: python3 scripts/install_slam_service.py", file=sys.stderr)
        return 2
    if os.getuid() == 0:
        print("Run this installer as the SLAM service user, without sudo.", file=sys.stderr)
        return 2

    try:
        unit = render_unit()
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8") as temporary:
            temporary.write(unit)
            temporary.flush()
            subprocess.run(["sudo", "install", "-m", "644", temporary.name, UNIT_PATH], check=True)
        subprocess.run(["sudo", "systemctl", "daemon-reload"], check=True)
        subprocess.run(["sudo", "systemctl", "enable", "navbot-slam.service"], check=True)
        subprocess.run(["sudo", "systemctl", "restart", "navbot-slam.service"], check=True)
    except (FileNotFoundError, ValueError, subprocess.CalledProcessError) as error:
        print(f"SLAM service installation failed: {error}", file=sys.stderr)
        return 1

    print("SLAM service enabled and restarted.")
    print("Check with: sudo systemctl status navbot-slam.service")
    print("Page: http://<ros2_pc-host>:8081/slam")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
