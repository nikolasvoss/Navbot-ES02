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
- [ ] Check the left/right mapping and the `/odom` twist signs on the robot. The forward sign is done (2026-10-25: both wheel angles grow when rolled forward); roll one wheel at a time to check left and right. The wheel boundary signs themselves were measured on 2026-10-22.
- [x] Gyro heading in the bridge: `heading_source:=gyro` takes the `/odom` yaw from the IMU gyro z rate and the distance from the wheels, with wheel-yaw fallback when IMU samples are missing ([bridge page](../software/navbot-link-bridge.md#heading-source)). Tested with simulated samples only.
- [ ] Verify `gyro_yaw_sign` with the 90° counter-clockwise hand test on the [bridge page](../software/navbot-link-bridge.md#heading-source), then run gyro heading on the robot (standstill drift, SLAM map no longer rotated) and switch the boot service to it.
- [ ] Define the static TF `base_link` to `imu_link` from the IMU mounting. `/imu/data` is in the sensor frame.
- [x] Align the SLAM branch (`feat/slam`): the static `odom` to `lds08rr` TF is gone, the LiDAR service publishes a static TF `base_link` to `lds08rr`, and `slam_toolbox` runs with `base_frame: base_link`. Not yet run on the robot.
- [ ] Check the LiDAR yaw offset on the robot: `SENSOR_MOUNT_YAW_RAD` is −120° (ahead reads 240° clockwise on the LiDAR page). Push the robot straight forward and confirm the SLAM pose arrow points along the direction of travel. `SENSOR_MOUNT_X_M` and `SENSOR_MOUNT_Y_M` stay zero because the LiDAR sits at the robot's rotation centre.
- [x] Start `navbot_link_bridge` at boot with `scripts/install_link_bridge_service.py` (unit and installer written, test passes). Not yet installed or run on the robot.
