# Integration roadmap

## ESP32 to ros2_pc link: remaining work

Status: the protocol (COBS, CRC-16, Protobuf), the firmware sender for `WheelOdometry` and `Imu` at 50 Hz, and the ROS 2 bridge `navbot_link_bridge` are implemented. The bridge ran on the ros2_pc only with simulated frames through a pseudo-terminal. Details: [protocol](../software/ros2-uart-protocol.md), [bridge](../software/navbot-link-bridge.md).

- [x] Merge the odometry bridge branch into `feature/esp32-ros2-uart-interface`.
- [x] Merge `main` (with the new repository layout) into `feature/esp32-ros2-uart-interface` and resolve the conflicts there, not on `main`.
- [ ] Merge the robust control branch (`docs/robust-control-plan`) and attach the odometry call to its control tick. That branch replaces the wheel-velocity code the call sits next to. It also fixes the wheel sign convention: forward is a negative motor-local target on both wheels, and wheel speeds are computed from `motor.shaftAngle()`. Publish the forward-positive wheel angles from that boundary, not the raw sensor angles the sender uses today (`+sensor 1`, `-sensor 2`).
- [x] Choose the ros2_pc UART: GPIO8/GPIO9 (J8 pins 24 and 21), recorded in the [J8 allocation](../../ros2_pc/hardware/system-overview.md#j8-pin-allocation) on 2026-10-25. The user called it UART4. The carrier datasheet lists GPIO8/9 as UART3 and UART4 as GPIO12/13, so the pins decide.
- [ ] Enable the UART on the ros2_pc (boot overlay and reboot), confirm the `/dev/ttyAMA*` path, verify pins and 3.3 V levels on the board, wire it, and replace the `port` placeholder in the [bridge page](../software/navbot-link-bridge.md).
- [ ] Flash the firmware and check the 500 kBd link on the real robot: `/odom` and `/imu/data` at 50 Hz, no `bad_frames` or `lost_frames` in the bridge log.
- [ ] Measure the track width and document it with `wheel_radius_m`. The control branch records a measured wheel diameter of 38.3 mm (2026-10-24) and uses a 0.019 m radius. The bridge has no default for either parameter.
- [ ] Check the signs: roll the robot forward by hand. Positive wheel angles must mean forward motion, and left and right must match.
- [ ] Define the static TF `base_link` to `imu_link` from the IMU mounting. `/imu/data` is in the sensor frame.
- [ ] Align the SLAM branch (`feat/slam`): it binds `odom` statically to `lds08rr`. With the bridge, that becomes a static TF `base_link` to `lds08rr`, and the static `odom` to `lds08rr` TF goes away.
