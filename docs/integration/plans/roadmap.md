# Integration roadmap

## ESP32 to ros2_pc link: remaining work

Status: the protocol (COBS, CRC-16, Protobuf), the firmware sender for `WheelOdometry` and `Imu` at 50 Hz, and the ROS 2 bridge `navbot_link_bridge` are implemented. The bridge ran on the ros2_pc only with simulated frames through a pseudo-terminal. Details: [protocol](../software/ros2-uart-protocol.md), [bridge](../software/navbot-link-bridge.md).

- [x] Merge the odometry bridge branch into `feature/esp32-ros2-uart-interface`.
- [x] Merge `main` (with the new repository layout) into `feature/esp32-ros2-uart-interface` and resolve the conflicts there, not on `main`.
- [x] Merge the robust control branch (`docs/robust-control-plan`) and attach the odometry call to its control tick. The sender now publishes forward-positive wheel angles: `motor.shaftAngle()` times `WheelBoundary::kLeftMotorSign` and `kRightMotorSign`, stamped with the wheel read time. Not yet checked on the robot.
- [x] Choose the ros2_pc UART: GPIO8/GPIO9 (J8 pins 24 and 21), recorded in the [J8 allocation](../../ros2_pc/hardware/system-overview.md#j8-pin-allocation) on 2026-10-25. The user called it UART4. The carrier datasheet lists GPIO8/9 as UART3 and UART4 as GPIO12/13, so the pins decide.
- [ ] Enable the UART on the ros2_pc (boot overlay and reboot), confirm the `/dev/ttyAMA*` path, verify pins and 3.3 V levels on the board, wire it, and replace the `port` placeholder in the [bridge page](../software/navbot-link-bridge.md).
- [ ] Flash the firmware and check the 500 kBd link on the real robot: `/odom` and `/imu/data` at about 35 Hz (the control tick runs at 710 Hz), no `bad_frames` or `lost_frames` in the bridge log.
- [ ] Measure the track width (distance between the wheel centres) and document it. The wheel radius is measured: 38.3 mm diameter on 2026-10-24, and the control firmware uses 0.019 m. The bridge has no default for either parameter.
- [ ] Check the odometry signs on the robot: roll it forward by hand. Positive wheel angles must mean forward motion, and left and right must match. The wheel boundary signs themselves were measured on 2026-10-22.
- [ ] Define the static TF `base_link` to `imu_link` from the IMU mounting. `/imu/data` is in the sensor frame.
- [ ] Align the SLAM branch (`feat/slam`): it binds `odom` statically to `lds08rr`. With the bridge, that becomes a static TF `base_link` to `lds08rr`, and the static `odom` to `lds08rr` TF goes away.
