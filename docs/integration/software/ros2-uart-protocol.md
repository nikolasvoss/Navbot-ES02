# ESP32 ↔ ros2_pc UART protocol (NavBot link v1)

Status: **specification, firmware sender for `WheelOdometry` and `Imu` (50 Hz each), and ROS 2 bridge `navbot_link_bridge` for both.** The firmware sender was checked on the robot on 2026-10-26 (50 Hz, no lost frames, also while the robot balanced); the bridge has not run against the robot.

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

Both directions use the same format. Transport: ESP32 UART1 (the former touchscreen port, now removed), **2000000 baud, 8N1**, RX = GPIO20, TX = GPIO19 in the firmware (`link_uart.h`). The firmware opens the port at startup and sends `WheelOdometry` and `Imu` in normal mode. One `Message` carries a `seq` (gap = lost frame), `t_ms` (sender `millis()`) and exactly one `body`: wheel odometry (`WheelOdometry`), sensor data (`Imu`, `Battery`), `Status`, `Log` (levels DEBUG–FATAL with a stable numeric `code` and short text for errors), PC→robot `Command` with `Ack`, and the catalog and subscription messages below.

## Catalog, subscriptions, parameters and actions

The schema also defines the messages for the entry table of the [logging and control plan](../plans/logging-and-control-link.md). **The firmware does not send or answer them yet**; only the PC library, [navlink](navlink.md) and its test firmware use them.

| PC → robot | Reply | Meaning |
| --- | --- | --- |
| `GetCatalog{index}` | `CatalogEntry` | One table entry: name, unit, `Kind` (`SIGNAL`, `SIGNAL_MIN`, `SIGNAL_MAX`, `PARAM`, `ACTION`), `Type` (`F32`, `I32`, `U32`, `BOOL`), and for an action its `Arg` names and bounds. Every entry carries `count`; the entry at index 0 also carries the firmware `build`. |
| `Subscribe{slot, ids, period_us}` | `Ack` | Stream the entries `ids` (table indices) every `period_us` (0 = every control tick) as `SignalBatch` frames. Sending the same slot again renews the 5 s lease. |
| `Unsubscribe{slot}` | `Ack` | Stop the slot. |
| `GetParam{id}`, `SetParam{id, value}` | `Ack` with `value` | Read or write one entry. `value` is 4 bytes little-endian, typed by the catalog. |
| `Call{id, args}` | `Ack` | Start an action; `args` are floats in catalog order. |

- `Ack.command_id` echoes the request's `Message.seq` (for `Command`, its `id`). A refused request has `accepted = false` and the reason in `error`.
- `SignalBatch.rows` holds one or more rows of `[t_us u32][4-byte value per id]`, little-endian.
- `Log.tag` names the source of a log line.
- nanopb bounds: 48 ids per subscription (196-byte row), 4 action arguments, 24-character names. The largest `Message` encodes to 229 bytes, below the 252-byte payload limit.

## Rules

- Receivers ignore unknown fields and unknown `body` types.
- The sender never blocks on a full UART; it drops the frame and counts it. The firmware count is not sent yet, so `Status.dropped_frames` stays unused until `Status` is sent.
- A receiver treats a frame with a bad CRC as lost. It does not try to repair it.
- Safety does not depend on this link. The robot keeps balance and fall handling locally and must stop or hold safely when `Command` frames stop.

## Odometry

Normal mode sends one `WheelOdometry` and one `Imu` on a 20 ms schedule (50 Hz each on average, from the control loop; see the rules below). Sensor diagnostic mode sends neither.

- `t_us` is `micros()` at the wheel read. Use it for deltas, not `t_ms`.
- `left_angle_rad` and `right_angle_rad` are cumulative wheel angles in radians, not wrapped. Positive angle means forward rolling, the same sign as the controller's wheel speeds. The firmware takes them from `motor.shaftAngle()` times `WheelBoundary::kLeftMotorSign` and `kRightMotorSign`, the only place motor signs appear.
- The sign convention was measured on the robot for the wheel boundary (2026-10-22: positive motor-local q rolls both wheels backward). The odometry stream was checked on hardware on 2026-10-25: rolling both wheels forward by hand gave positive changes in `left_angle_rad` and `right_angle_rad` (+11.04 and +9.96 rad). Rolling backward was not tested ([lab note](../../../lab/integration/2026-10-25-odometry-signs-and-rate.md)).
- The PC owns wheel radius and track width. It computes wheel travel from the angle deltas, so a dropped frame loses one sample and not the total.
- `Imu` values use SI units. Axes are the IMU sensor frame, not a robot body frame. Acceleration and angular rate are the filtered values.
- `Imu.t_us` is `micros()` at the IMU read (`imu::Snapshot::readTimeUs`), a host timestamp, not the sensor's own. Use it for IMU deltas, not `t_ms`. Firmware older than this field leaves it unset, which decodes as 0.

The first hardware test on 2026-10-25 ran at 115200 baud; the link runs at 2000000 baud (raised from 500000 on 2026-10-27; 921600, 2000000 and 3000000 each ran with 0 bad and 0 lost frames, 2000000 for 120 s, so 2000000 was kept with margin). Measured on the robot on 2026-10-26: 4.96 kB/s for `WheelOdometry` and `Imu` at 50 Hz each, about 2.5 % of the 200 kB/s the line carries at 2000000 baud ([lab note](../../../lab/integration/2026-10-25-odometry-signs-and-rate.md)). `LINK_UART_BAUD` in `link_uart.h` and the bridge's `baud` parameter must match.

## Time stamps

The robot and the ros2_pc run on unsynchronized clocks, and nothing converts one into the other.

| Stamp | Source | Meaning |
| --- | --- | --- |
| `Message.seq` | robot counter | Gap = lost frame; a non-increasing value = robot restart. |
| `Message.t_ms` | robot `millis()` when the frame is encoded | Monotonic, 1 ms resolution. The bridge does not use it. |
| `WheelOdometry.t_us` | robot `micros()` at the wheel read | Wraps at 2^32 (about 71 min). The bridge uses only differences, which are wrap-safe. |
| `Imu.t_us` | robot `micros()` at the IMU read | Same clock and wrap as `WheelOdometry.t_us`. The IMU read and the wheel read happen at different moments, so the two values differ. The bridge decodes it into `ImuSample.t_us` but computes no IMU deltas yet, and `/imu/data` does not carry it. |
| ROS `header.stamp` | ros2_pc `get_clock().now()` | Taken once per serial poll (200 Hz timer) and shared by every frame in that read. It is the arrival time, not the sample time. |

Consequences:

- Velocities and yaw rate come from robot-clock differences (`t_us`), so they are not affected by link jitter. A consumer that needs IMU sample spacing must take it from `Imu.t_us` differences, not from ROS stamps.
- Absolute stamps on `/odom`, the `odom` to `base_link` TF and `/imu/data` are arrival times. They lag the true sample by the transmission time (20 µs per byte at 500 kBd: about 0.8 ms for an odometry frame of about 40 bytes and up to 1.4 ms for an `Imu` frame of up to 70 bytes), the robot's TX queue and the polling delay (up to 5 ms). Odometry and IMU frames from the same tick often carry the same stamp.
- Robot data and other ROS data (the LiDAR scan, for example) are matched only through these ROS stamps. A robot-to-ROS clock offset is not estimated. Better matching would need an offset estimate from `t_us` against the arrival time, or an explicit time-sync message. Neither exists.
- The send is checked once per control tick, right after the wheel read, so the data is at most one tick old. Each send moves the 20 ms deadline forward by exactly 20 ms (`LinkScheduleDue()` in `link_schedule.h`, period `LINK_ODOMETRY_PERIOD_US`), so the average rate is 50 Hz. A frame goes out on the first tick at or after its deadline, so single intervals jitter by up to one control tick (measured period about 1.41 ms, [lab note](../../../lab/integration/2026-10-25-odometry-signs-and-rate.md)): between about 18.6 and 21.4 ms. After a stall of a full period or more (a blocking calibration step, for example), one frame is sent and the schedule restarts 20 ms later; missed frames are not caught up. Use `t_us` for deltas, not the nominal rate.

## Code

- [generate_link_proto.sh](../../../scripts/generate_link_proto.sh): regenerates the nanopb C code in `src/ES-02/OllieFOCdrive/src/navbot_link/` and the Python protobuf module `navbot_link_pb2.py` in `src/navbot_link/`. It needs `nanopb_generator` 0.4.9.1 and `protoc` 3.21.x on `PATH`; the script refuses other versions. Run it after every schema change.
- The nanopb 0.4.9.1 encode runtime is vendored in the same folder. Do not edit the generated `navbot_link.pb.*` files.
- Firmware: `link_frame` (pure C++, frames one `Message`) and `link_uart` (UART1 init, sequence and time stamps, non-blocking send, odometry and IMU publish).
- PC: the Python package [navbot_link](../../../src/navbot_link/) has no ROS dependency and is shared by the ROS 2 bridge and the host tools. [framing.py](../../../src/navbot_link/framing.py) holds `encode_frame` and `FrameDecoder`, which returns payloads and, for a printable chunk that is not a valid frame, `Text`. `protocol.py` holds the catalog types, the 4-byte value codec and the per-port `seq` tracker; `link.py` holds the `Link` client and `capture.py` the CSV writer and presets used by [navlink](navlink.md).
- ROS 2 bridge: [navbot_link_bridge](../../../src/ros2_pc/ros2/src/navbot_link_bridge/) (`bridge.py` decodes frames, `odometry.py` integrates wheel angles, `node.py` is the rclpy node). Its `setup.py` installs `navbot_link` next to the bridge. Build and run steps: [ros2_pc software](navbot-link-bridge.md).
- Tests: `PYTHONPATH=src:src/ros2_pc/ros2/src/navbot_link_bridge python3 -m pytest -q src/ros2_pc/ros2/src/navbot_link_bridge/test` (bridge: the firmware literal frames including the IMU `t_us`, seq gaps and restarts; odometry: straight line, spin, `micros()` wrap) and `python3 -m pytest -q tests/navbot_link` (framing: CRC check value `0x29B1`, COBS vectors, resync after garbage, raw text between USB frames).
- Host test: `bash tests/link/run_host_test.sh` encodes an odometry and an IMU `Message` (with `t_us`) with the firmware encoder and compares the frame bytes to literal vectors.

## Next steps

1. Done on 2026-10-26: pins, levels and the 500 kBd link work on the robot. The cause of the earlier failure was an unpowered STC8H1K08 on the line ([lab note](../../../lab/robot/2026-10-25-stc8h1k08-backpowers-uart1.md)). GPIO19/20 are the USB Serial/JTAG pads and power up in USB mode, so `LinkUartInit()` releases them (`USB_SERIAL_JTAG.conf0.usb_pad_enable = 0`) before opening UART1 and logs `usbPad=` at boot. A build without this release also carried clean frames, so the release is a safeguard, not the fix.
2. Done: the track width is 113 mm and the forward sign was checked (both wheel angles grow when rolled forward). The left/right mapping was checked with a turn test on 2026-10-26 ([lab note](../../../lab/integration/2026-10-26-odometry-turn-test.md)); the `/odom` twist signs are not yet checked. The bridge takes the wheel radius and the track width as required parameters.
3. Map `Battery`, `Status` and `Log` in the ROS 2 bridge to `sensor_msgs/BatteryState`, `diagnostic_msgs` and `/rosout`.
