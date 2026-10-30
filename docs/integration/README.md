# Integration documentation

The integration system is the connection between the robot (ESP32-S3) and the ros2_pc: the UART link, its protocol, the ROS 2 bridge, and the cabling between the two. Placement rules for new pages are in [AGENTS.md](../../AGENTS.md#where-files-go).

## Software

- [NavBot link protocol](software/ros2-uart-protocol.md): framing, messages, odometry convention, and the firmware sender.
- [ros2_pc setup](software/ros2-pc-setup.md): everything to configure on the ros2_pc outside the checkout: packages, boot overlay, permissions, checks.
- [NavBot link bridge](software/navbot-link-bridge.md): the ROS 2 node that publishes `/odom`, the `odom` to `base_link` TF, and `/imu/data`; build, run, and open checks.
- [navlink quick start](software/navlink-quick-start.md): one page on starting the shell, reading, streaming, setting and recording.
- [navlink](software/navlink.md): the `navbot_link` library and the `navlink` shell and CSV capture for signals, parameters and actions, with the presets for the balance and drive traces.
- [Signal table, parameters, actions and link logs](software/signal-table.md): the entries the firmware serves, the settings groups, the action safety rules, the subscription budget and the USB backup port.

## Plans

- [Roadmap](plans/roadmap.md): the robot checks for the link, its logs and signals, and the open dashboard decision.
- [Link features for autonomous development](plans/link-autonomy.md): build ID, health signals, reboot, live parameters, CH5 link mode and ROS `Command`, link multiplexer, one-shot navlink; the task plan for the implementing agents.
