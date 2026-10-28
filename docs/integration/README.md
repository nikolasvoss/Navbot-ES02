# Integration documentation

The integration system is the connection between the robot (ESP32-S3) and the ros2_pc: the UART link, its protocol, the ROS 2 bridge, and the cabling between the two. Placement rules for new pages are in [AGENTS.md](../../AGENTS.md#where-files-go).

## Software

- [NavBot link protocol](software/ros2-uart-protocol.md): framing, messages, odometry convention, and the firmware sender.
- [ros2_pc setup](software/ros2-pc-setup.md): everything to configure on the ros2_pc outside the checkout: packages, boot overlay, permissions, checks.
- [NavBot link bridge](software/navbot-link-bridge.md): the ROS 2 node that publishes `/odom`, the `odom` to `base_link` TF, and `/imu/data`; build, run, and open checks.
- [navlink](software/navlink.md): the `navbot_link` library and the `navlink` shell and CSV capture for signals, parameters and actions, and the presets that reproduce the `K55` to `K58` profiles; the firmware serves the signals, not yet parameters or actions.

## Plans

- [Roadmap](plans/roadmap.md)
- [Logging, telemetry and control over the NavBot link](plans/logging-and-control-link.md): proposal to replace the `K` selectors, log profiles and letter commands with one table of signals, parameters and actions on both the ros2_pc UART and the USB port.
