# ESP32 ↔ ros2_pc UART protocol (NavBot link v1)

Status: **specification, firmware sender for `WheelOdometry` and `Imu` (50 Hz each), and ROS 2 bridge `navbot_link_bridge` for both.** The firmware sender and the bridge are untested on hardware.

## Design

Three standard parts, nothing custom:

| Layer | Choice | Why |
| --- | --- | --- |
| Messages | [Protocol Buffers](https://protobuf.dev), schema in [navbot_link.proto](../../../src/navbot_link/navbot_link.proto) | Compact binary, the `.proto` is the documentation, forward-compatible (add fields, never reuse numbers). nanopb runs on the ESP32 without heap. |
| Framing | [COBS](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing) with `0x00` delimiter | Resynchronises after any corruption or mid-stream start by waiting for the next `0x00`. Overhead ≤ 1 byte per 254. |
| Integrity | CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`) | Detects line noise; bad frames are dropped and counted, never decoded. |

micro-ROS was rejected: ESP32-S3 support in `micro_ros_arduino` is unconfirmed and its UART transport is not pluggable.

## Wire format

```text
frame = COBS( payload || crc16_le(payload) ) || 0x00
payload = serialized navbot.link.v1.Message   (<= 252 bytes)
```

Both directions use the same format. Transport: ESP32 UART1 (the former touchscreen port, now removed), **500000 baud, 8N1**, RX = GPIO20, TX = GPIO19 in the firmware (`link_uart.h`). The firmware opens the port at startup and sends `WheelOdometry` and `Imu` in normal mode. One `Message` carries a `seq` (gap = lost frame), `t_ms` (sender `millis()`) and exactly one `body`: wheel odometry (`WheelOdometry`), sensor data (`Imu`, `Battery`), `Status`, `Log` (levels DEBUG–FATAL with a stable numeric `code` and short text for errors), and PC→robot `Command` with `Ack`.

## Rules

- Receivers ignore unknown fields and unknown `body` types.
- The sender never blocks on a full UART; it drops the frame and counts it. The firmware count is not sent yet, so `Status.dropped_frames` stays unused until `Status` is sent.
- A receiver treats a frame with a bad CRC as lost. It does not try to repair it.
- Safety does not depend on this link. The robot keeps balance and fall handling locally and must stop or hold safely when `Command` frames stop.

## Odometry

Normal mode sends one `WheelOdometry` and one `Imu` every 20 ms (50 Hz each, from the 1 kHz control loop). Sensor diagnostic mode sends neither.

- `t_us` is `micros()` at the wheel sample. Use it for deltas, not `t_ms`.
- `left_angle_rad` and `right_angle_rad` are cumulative wheel angles in radians, not wrapped. Positive angle means forward rolling, the same sign as the controller's wheel speeds. The firmware takes them from `motor.shaftAngle()` times `WheelBoundary::kLeftMotorSign` and `kRightMotorSign`, the only place motor signs appear.
- The sign convention was measured on the robot for the wheel boundary (2026-10-22: positive motor-local q rolls both wheels backward). The odometry stream itself has not been checked on hardware. Roll the robot forward and check the signs before relying on them.
- The PC owns wheel radius and track width. It computes wheel travel from the angle deltas, so a dropped frame loses one sample and not the total.
- `Imu` values use SI units. Axes are the IMU sensor frame, not a robot body frame. Acceleration and angular rate are the filtered values.

## Code

- [generate_link_proto.sh](../../../scripts/generate_link_proto.sh): regenerates the nanopb C code in `src/ES-02/OllieFOCdrive/src/navbot_link/` and the Python protobuf module in `src/ros2_pc/ros2/src/navbot_link_bridge/navbot_link_bridge/`. It needs `nanopb_generator` 0.4.9.1 and `protoc` 3.21.x on `PATH`; the script refuses other versions. Run it after every schema change.
- The nanopb 0.4.9.1 encode runtime is vendored in the same folder. Do not edit the generated `navbot_link.pb.*` files.
- Firmware: `link_frame` (pure C++, frames one `Message`) and `link_uart` (UART1 init, sequence and time stamps, non-blocking send, odometry and IMU publish).
- [framing.py](../../../src/ros2_pc/ros2/src/navbot_link_bridge/navbot_link_bridge/framing.py): `encode_frame`, `FrameDecoder` (stdlib only).
- ROS 2 bridge: [navbot_link_bridge](../../../src/ros2_pc/ros2/src/navbot_link_bridge/) (`bridge.py` decodes frames, `odometry.py` integrates wheel angles, `node.py` is the rclpy node). Build and run steps: [ros2_pc software](navbot-link-bridge.md).
- Tests: `PYTHONPATH=src/ros2_pc/ros2/src/navbot_link_bridge python3 -m pytest -q src/ros2_pc/ros2/src/navbot_link_bridge/test` (framing: CRC check value `0x29B1`, COBS vectors, resync after garbage; bridge: the firmware literal frames, seq gaps and restarts; odometry: straight line, spin, `micros()` wrap).
- Host test: `bash tests/link/run_host_test.sh` encodes an odometry and an IMU `Message` with the firmware encoder and compares the frame bytes to literal vectors.

## Next steps

1. Verify pins, 3.3 V levels and the 500 kBd link on the physical board before wiring the ros2_pc.
2. Measure the track width and check the odometry signs by rolling the robot forward. The bridge takes the wheel radius and the track width as required parameters.
3. Map `Battery`, `Status` and `Log` in the ROS 2 bridge to `sensor_msgs/BatteryState`, `diagnostic_msgs` and `/rosout`.
