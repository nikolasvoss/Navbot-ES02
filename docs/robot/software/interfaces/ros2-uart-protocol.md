# ESP32 ↔ ros2_pc UART protocol (NavBot link v1)

Status: **specification and PC-side framing library only.** Firmware integration (nanopb), and the ROS 2 bridge node are not done; the UART settings are fixed below but no hardware test has been run.

## Design

Three standard parts, nothing custom:

| Layer | Choice | Why |
| --- | --- | --- |
| Messages | [Protocol Buffers](https://protobuf.dev), schema in [navbot_link.proto](../../../../src/navbot_link/navbot_link.proto) | Compact binary, the `.proto` is the documentation, forward-compatible (add fields, never reuse numbers). nanopb runs on the ESP32 without heap. |
| Framing | [COBS](https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing) with `0x00` delimiter | Resynchronises after any corruption or mid-stream start by waiting for the next `0x00`. Overhead ≤ 1 byte per 254. |
| Integrity | CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`) | Detects line noise; bad frames are dropped and counted, never decoded. |

micro-ROS was rejected: ESP32-S3 support in `micro_ros_arduino` is unconfirmed and its UART transport is not pluggable.

## Wire format

```text
frame = COBS( payload || crc16_le(payload) ) || 0x00
payload = serialized navbot.link.v1.Message   (<= 252 bytes)
```

Both directions use the same format. Transport: ESP32 UART1 (the former touchscreen port, now removed), **500000 baud, 8N1**, RX = GPIO20, TX = GPIO19 in the firmware (`link_uart.h`). The firmware opens the port at startup; no frames are sent yet. One `Message` carries a `seq` (gap = lost frame), `t_ms` (sender `millis()`) and exactly one `body`: sensor data (`Imu`, `Battery`), `Status`, `Log` (levels DEBUG–FATAL with a stable numeric `code` and short text for errors), and PC→robot `Command` with `Ack`.

## Rules

- Receivers ignore unknown fields and unknown `body` types.
- The sender never blocks on a full UART; it drops the frame and increments `Status.dropped_frames`.
- A receiver treats a frame with a bad CRC as lost. It does not try to repair it.
- Safety does not depend on this link. The robot keeps balance and fall handling locally and must stop or hold safely when `Command` frames stop.

## Code

- [framing.py](../../../../src/navbot_link/framing.py): `encode_frame`, `FrameDecoder` (stdlib only).
- Tests: `python3 src/navbot_link/test_framing.py` (CRC check value `0x29B1`, COBS vectors, resync after garbage and a corrupted frame).

## Next steps

1. Verify pins, 3.3 V levels and the 500 kBd link on the physical board before wiring the ros2_pc.
2. Generate the C code with nanopb and add the encoder plus a non-blocking TX queue to the firmware.
3. Add the ROS 2 bridge node mapping `Imu`, `Battery`, `Status` and `Log` to `sensor_msgs/Imu`, `BatteryState`, `diagnostic_msgs` and `/rosout`.
