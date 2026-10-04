#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ros_distro="${ROS_DISTRO:-jazzy}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"
workspace_setup="${workspace_root}/src/cm5/ros2/install/setup.bash"

if [[ ! -r "$ros_setup" ]]; then
    echo "ROS setup not found: $ros_setup" >&2
    exit 1
fi
if [[ ! -r "$workspace_setup" ]]; then
    echo "HMMD workspace is not built: $workspace_setup" >&2
    echo "Build it with the steps in docs/cm5/software/how-to/hmmd-ros2.md" >&2
    exit 1
fi

source "$ros_setup"
source "$workspace_setup"

hmmd_port="${HMMD_PORT:-${1:-}}"
hmmd_baud_rate="${HMMD_BAUD_RATE:-115200}"

if [[ -z "$hmmd_port" ]]; then
    echo "Set HMMD_PORT to the verified HMMD serial device path." >&2
    echo "Example: HMMD_PORT=/path/to/verified/device $0" >&2
    exit 2
fi
if [[ ! -c "$hmmd_port" ]]; then
    echo "HMMD port is not a character device: $hmmd_port" >&2
    exit 2
fi
if [[ ! -r "$hmmd_port" || ! -w "$hmmd_port" ]]; then
    echo "No read/write access to $hmmd_port; check serial-device permissions (for example, dialout group membership)." >&2
    exit 2
fi
if [[ ! "$hmmd_baud_rate" =~ ^[1-9][0-9]*$ ]]; then
    echo "HMMD_BAUD_RATE must be a positive integer: $hmmd_baud_rate" >&2
    exit 2
fi

exec ros2 run hmmd_radar hmmd_sensor --ros-args \
    -p "port:=${hmmd_port}" \
    -p "baud_rate:=${hmmd_baud_rate}"
