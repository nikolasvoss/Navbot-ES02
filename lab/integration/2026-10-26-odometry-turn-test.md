# Odometry turn test: wheel yaw, gyro yaw and left/right mapping, 2026-10-26

Status: concluded

## Observation

The robot was turned on the spot exactly 3 turns counter-clockwise (seen from above) against a floor mark. Then, after a pause on the mark, it was turned about a quarter turn further before recording stopped. `navbot_link_bridge` ran as the boot service in wheel heading mode (`wheel_radius_m` 0.01915, `wheel_separation_m` 0.113).

| Point | Wheel yaw (`/odom`) | Gyro yaw (+gz integrated) |
| --- | ---: | ---: |
| On the mark after 3 turns (true 1080°) | 1226.2° | 1080.4° |
| End of recording (about 1170° true) | 1324.9° | 1168.7° |

- Wheel yaw is positive for a counter-clockwise turn, so the left/right wheel mapping is correct.
- Integrated +gz is positive for a counter-clockwise turn, so `gyro_yaw_sign` is +1.
- The gyro matches the true turn within 0.04 % at the mark (1080.4° for 1080°).
- Wheel yaw over-reads by a constant factor: 1226.2 / 1080 = 1.135 at the mark, 1324.9 / 1168.7 = 1.134 at the end.

## Evidence

Recorded with `python3 scripts/odometry_turn_test.py --turns 3` on the ros2_pc: 1873 `/odom` and 1873 `/imu/data` messages. The script's final JSON assumed 3 turns at the end of the recording, which was wrong because of the extra quarter turn; the numbers above use the plateau where the robot stood on the mark (gyro 1080.4°, wheel 1226.2°).

## Practical consequence

- Gyro heading is reliable and its sign is known: run the bridge with `heading_source:=gyro` and `gyro_yaw_sign:=1`.
- In wheel heading mode, yaw scales with `wheel_radius_m / wheel_separation_m`. The measured ratio is 0.01915 / 0.113 / 1.135 = 0.149 instead of 0.169. If the radius is right, the effective track width is 0.113 × 1.135 = 0.128 m. A spin cannot separate the two; a straight push along a tape measure gives the radius on its own. The cause of the 13.5 % (wheel scrub during the spin, contact points outside the wheel centres, or a radius error) is not identified.
