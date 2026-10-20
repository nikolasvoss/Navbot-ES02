# ros2_pc documentation

The ros2_pc is the onboard computer that runs ROS 2, with its carrier board and the sensors attached to it. Each peripheral stays pending until its page marks it verified. Placement rules for new pages are in [AGENTS.md](../../AGENTS.md#where-files-go).

## Hardware

- [System overview](hardware/system-overview.md): start here for any ros2_pc hardware question.
- [Carrier board pinout](hardware/ros2-pc-carrier-board.md): connectors, alternate functions, and electrical limits.
- [LDS08RR LiDAR](hardware/lds08rr-lidar.md): wiring, electrical interface, data format, and mechanics.
- [HMMD radar](hardware/hmmd-radar.md): protocol, wiring limits, and vendor documents.

## Software

- [LDS08RR LiDAR software](software/lds08rr-lidar.md): setup, demo, ROS 2 topics, and boot service.
- [HMMD ROS 2 developer guide](software/hmmd-ros2.md): build, startup, browser, topics, and recording.

## Plans

- [Roadmap](plans/roadmap.md)
