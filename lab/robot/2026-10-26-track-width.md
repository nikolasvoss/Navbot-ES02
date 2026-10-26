# Track width of the robot, 2026-10-26

Status: concluded

## Observation

The user measured the track width, the distance between the wheel centres, as 113 mm.

## Evidence

User report in the chat on 2026-10-26. The measuring method and its uncertainty were not recorded. The wheel diameter of 38.3 mm was measured on 2026-10-24 (see the comment at `wheelRadiusM` in `OllieFOCdrive.ino`).

## Practical consequence

The ROS 2 bridge (`wheel_separation_m`) needs 0.113. An error in the track width scales the yaw rate computed from the wheel difference by the same ratio, so a turn test on the floor (rotate the robot by a known angle and compare the integrated yaw) would show whether 113 mm is accurate enough. The bridge doc and the roadmap point to this note's result in the [robot hardware overview](../../docs/robot/hardware/system-overview.md).
