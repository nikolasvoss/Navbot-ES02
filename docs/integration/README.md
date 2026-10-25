# Integration documentation

The integration system is the connection between the robot (ESP32-S3) and the ros2_pc: the UART link, its protocol, the ROS 2 bridge, and the cabling between the two. Placement rules for new pages are in [AGENTS.md](../../AGENTS.md#where-files-go).

## Software

- [NavBot link protocol](software/ros2-uart-protocol.md): framing, messages, odometry convention, and the firmware sender.
- [NavBot link bridge](software/navbot-link-bridge.md): the ROS 2 node that publishes `/odom`, the `odom` to `base_link` TF, and `/imu/data`; build, run, and open checks.

## Plans

- [Roadmap](plans/roadmap.md)
