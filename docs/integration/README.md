# Integration documentation

The integration system is the connection between the robot (ESP32-S3) and the ros2_pc: the UART link, its protocol, the ROS 2 bridge, and the cabling between the two. Placement rules for new pages are in [AGENTS.md](../../AGENTS.md#where-files-go).

## Software

- [NavBot link protocol](software/ros2-uart-protocol.md): framing, messages, odometry convention, the firmware sender, the drive command with the CH5 positions, and the link multiplexer.
- [ros2_pc setup](software/ros2-pc-setup.md): everything to configure on the ros2_pc outside the checkout: packages, boot overlay, permissions, checks.
- [NavBot link bridge](software/navbot-link-bridge.md): the ROS 2 node that publishes `/odom`, the `odom` to `base_link` TF, `/imu/data`, `/diagnostics` and `/battery_state`, and takes `/cmd_vel`; build, run, and open checks.
- [Development hosts](software/dev-hosts.md): the desktop PC and the ros2_pc, which one has the toolchain and the robot's USB cable, and how `deploy.py` picks them.
- [Agent workflow](software/agent-workflow.md): task worktrees and folders, the sweep, `deploy.py`, the device test tiers, `verify.py`, skips, hooks and permissions.
- [navlink quick start](software/navlink-quick-start.md): one page on starting the shell, reading, streaming, setting and recording.
- [navlink](software/navlink.md): the `navbot_link` library, the `navlink` shell, the one-shot commands with JSON and exit codes, and CSV capture for signals, parameters and actions, with the presets for the balance, drive and health traces.
- [Signal table, parameters, actions and link logs](software/signal-table.md): the entries the firmware serves, the settings groups, the action safety rules, the subscription budget and the USB backup port.

## Plans

- [Roadmap](plans/roadmap.md): the robot checks for the link, its logs and signals, the link autonomy checks, and the open dashboard decision.
- [Fully autonomous driving](plans/autonomous-driving.md): the required and useful features for Nav2 driving on the saved map, the decisions for the user, and the phases from the first link drive to the first autonomous course.
