# Touchscreen stand and robot mounting holes

[`touchscreenstand.stl`](../../../../hardware/3d%20print/touchscreenstand.stl) is the 3D-printed stand that held the original touchscreen on top of the robot. The touchscreen is removed. Its mounting-hole pattern is still the robot-side interface: the [LiDAR/ros2_pc stand](../../../ros2_pc/hardware/README.md#mechanics) (`hardware/3d print/lidar_ros2_pc_stand/`) uses the same pattern. This page describes only the mounting holes.

## Mounting holes

| Feature | Value |
| --- | --- |
| Pattern | Rectangle, 96.4 mm × 72.9 mm centre-to-centre |
| Hole diameter | 2.2 mm |
| Countersink | Present; dimensions not given |

## Not specified

- Countersink diameter, depth, angle and side.
- Tolerances of the hole dimensions.
- Hole position and axis direction in the STL coordinate system.
- Screw size.

## Source

User statement of 2026-10-13. The rectangle dimensions are read as distances between hole centres. Nobody has measured the STL file for these values. Evidence: [finding](../../../../agent_notes/robot/hardware-findings/touchscreenstand-mounting-holes.md).
