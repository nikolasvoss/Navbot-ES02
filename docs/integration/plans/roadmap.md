# Integration roadmap

## ESP32 to ros2_pc link: remaining work

Status: the protocol (COBS, CRC-16, Protobuf), the firmware sender for `WheelOdometry` and `Imu` at 50 Hz, and the ROS 2 bridge `navbot_link_bridge` are implemented. The bridge ran on the ros2_pc only with simulated frames through a pseudo-terminal. Details: [protocol](../software/ros2-uart-protocol.md), [bridge](../software/navbot-link-bridge.md).

- [x] Merge the odometry bridge branch into `feature/esp32-ros2-uart-interface`.
- [x] Merge `main` (with the new repository layout) into `feature/esp32-ros2-uart-interface` and resolve the conflicts there, not on `main`.
- [x] Merge the robust control branch (`docs/robust-control-plan`) and attach the odometry call to its control tick. The sender now publishes forward-positive wheel angles: `motor.shaftAngle()` times `WheelBoundary::kLeftMotorSign` and `kRightMotorSign`, stamped with the wheel read time. The forward sign was checked on the robot on 2026-10-25.
- [x] Choose the ros2_pc UART: GPIO8/GPIO9 (J8 pins 24 and 21), recorded in the [J8 allocation](../../ros2_pc/hardware/system-overview.md#j8-pin-allocation) on 2026-10-25. The user called it UART4. The carrier datasheet lists GPIO8/9 as UART3 and UART4 as GPIO12/13, so the pins decide.
- [x] Enable the UART on the ros2_pc (boot overlay and reboot), confirm the `/dev/ttyAMA*` path, verify pins and 3.3 V levels on the board, wire it, and replace the `port` placeholder in the [bridge page](../software/navbot-link-bridge.md).
- [x] Flash the firmware and check the 500 kBd link on the real robot (2026-10-26): `WheelOdometry` and `Imu` at 50.0 Hz each on a 20 ms schedule, 12002 CRC-valid frames in 2 minutes with the robot balancing, no lost frames. Measured with a script, not with the bridge.
- [ ] Run the bridge on the ros2_pc against the robot: `/odom` and `/imu/data` at about 50 Hz, no `bad_frames` or `lost_frames` in the bridge log.
- [x] Track width 113 mm (2026-10-26) and wheel diameter 38.3 mm (2026-10-24) are documented in the [robot hardware overview](../../robot/hardware/system-overview.md#key-findings-and-consequences). The control firmware uses a radius of 0.019 m (exact: 0.01915 m). The bridge has no default for either parameter; pass `wheel_radius_m` and `wheel_separation_m`.
- [x] Left/right mapping checked with a 3-turn spin on 2026-10-26: correct ([lab note](../../../lab/integration/2026-10-26-odometry-turn-test.md)). The same test showed that the wheel yaw over-reads by 1.135 (effective separation 0.128 m with the measured radius).
- [ ] Check the wheel radius with a straight push along a tape measure, and the `/odom` twist signs on the robot.
- [x] Gyro heading in the bridge: `heading_source:=gyro` takes the `/odom` yaw from the IMU gyro z rate and the distance from the wheels, with wheel-yaw fallback when IMU samples are missing ([bridge page](../software/navbot-link-bridge.md#heading-source)). Tested with simulated samples only.
- [x] `gyro_yaw_sign` is +1 (turn test 2026-10-26: 1080.4° for 3 counter-clockwise turns, [lab note](../../../lab/integration/2026-10-26-odometry-turn-test.md)).
- [ ] Run gyro heading on the robot: standstill drift over a minute, and a SLAM map that is no longer rotated.
- [ ] Define the static TF `base_link` to `imu_link` from the IMU mounting. `/imu/data` is in the sensor frame.
- [x] Align the SLAM branch (`feat/slam`): the static `odom` to `lds08rr` TF is gone, the LiDAR service publishes a static TF `base_link` to `lds08rr`, and `slam_toolbox` runs with `base_frame: base_link`. Not yet run on the robot.
- [ ] Check the LiDAR yaw offset on the robot: `SENSOR_MOUNT_YAW_RAD` is −120° (ahead reads 240° clockwise on the LiDAR page). Push the robot straight forward and confirm the SLAM pose arrow points along the direction of travel. `SENSOR_MOUNT_X_M` and `SENSOR_MOUNT_Y_M` stay zero because the LiDAR sits at the robot's rotation centre.
- [x] Start `navbot_link_bridge` at boot with `scripts/install_link_bridge_service.py` (unit and installer written, test passes). Not yet installed or run on the robot.

## Decide the dashboard for logs and debug signals

Context: the [logging and control proposal](logging-and-control-link.md) sends text logs and selectable signals over the NavBot link, both on UART1 to the ros2_pc and on the USB port as a backup. The ROS 2 bridge publishes them as topics. The protocol does not depend on this decision.

Decision: which tool shows the live debug signals (plots for control tuning), the log messages, and the parameter get/set. Choose one:

| Option | What it is | Runs on | Effort here |
| --- | --- | --- | --- |
| PlotJuggler | Desktop app for live time-series plots of ROS 2 topics; also opens CSV files. No logs or parameter panel. | Laptop with ROS 2, or the ros2_pc with a screen | Install only |
| Foxglove | Web or desktop app; plots, log list, 3D view of the map and scan, parameter panel. Connects to `foxglove_bridge`, a ROS 2 node on the ros2_pc. | Any browser; bridge on the ros2_pc | Install `foxglove_bridge`, save a layout |
| Own web page | Extend the LiDAR web service (`lds08rr_lidar/web.py`) with a signal plot, a log list and parameter fields. | Any browser | Write and maintain the page |

Criteria: plot rate needed for control tuning (up to the control tick, about 700 Hz, decimated), whether the tool works without a ROS 2 install on the laptop, and maintenance effort.

- [ ] Try PlotJuggler and Foxglove with the bridge's signal topic once it exists, then record the choice here.
