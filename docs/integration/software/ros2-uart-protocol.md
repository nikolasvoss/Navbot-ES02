# ESP32 ↔ ros2_pc UART protocol (NavBot link v1)

Status: **specification, firmware sender for `WheelOdometry` and `Imu` (50 Hz each), ROS 2 bridge `navbot_link_bridge` for both, the firmware side of the catalog, subscription, parameter and action messages, the drive `Command`, and the PC-side link multiplexer.** The firmware sender was checked on the robot on 2026-10-26 (50 Hz, no lost frames, also while the robot balanced); the bridge has not run against the robot. The queue-and-writer-task transmit path (see [Transmit path](#transmit-path)) was checked on the robot with 50 Hz odometry and no dropped frames. The receive path, the catalog, the subscriptions, the `Log` frames on both ports, `Command`, the live parameters and the multiplexer are host-tested and have not run on the robot; the open robot checks are in the [roadmap](../plans/roadmap.md#link-autonomy-on-the-robot).

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

Both directions use the same format. The PC writes a `0x00` before each frame it sends, so bytes the robot still holds from an earlier session cannot join that frame; the robot ignores the empty chunk. Transport: ESP32 UART1 (the former touchscreen port, now removed), **2000000 baud, 8N1**, RX = GPIO20, TX = GPIO19 in the firmware (`link_uart.h`). The firmware opens the port at startup and sends `WheelOdometry` and `Imu` in normal mode. One `Message` carries a `seq` (gap = lost frame), `t_ms` (sender `millis()`) and exactly one `body`: wheel odometry (`WheelOdometry`), sensor data (`Imu`), `Log` (levels DEBUG–FATAL with a stable numeric `code` and short text for errors), PC→robot `Command` with `Ack`, and the catalog, subscription, parameter and action messages below. The messages `Battery` and `Status` were removed; their field numbers 11 and 12 are reserved.

## Catalog, subscriptions, parameters and actions

The messages for the entry table in the [signal table](signal-table.md). The firmware answers `GetCatalog`, `Subscribe`, `Unsubscribe`, `GetParam`, `SetParam` and `Call` on both ports and streams `SignalBatch` rows; `Command` is answered with an `Ack` too. Its table (`signals.def.h`) holds 55 signals, 18 parameters (balance, IMU, servo-trim and log settings) and 7 actions (motor tests, motor stop and reset, calibration, `balance` and `reboot`). A refused request has its reason in `Ack.error`. [navlink](navlink.md) is the host side.

| PC → robot | Reply | Meaning |
| --- | --- | --- |
| `GetCatalog{index}` | `CatalogEntry` | One table entry: name, unit, `Kind` (`SIGNAL`, `SIGNAL_MIN`, `SIGNAL_MAX`, `PARAM`, `ACTION`), `Type` (`F32`, `I32`, `U32`, `BOOL`), for a parameter its `live_step` (0 = refused while balancing, `inf` = any change), and for an action its `Arg` names and bounds. Every entry carries `count`; the entry at index 0 also carries the firmware `build`. |
| `Command{speed_mps, turn_rad_s}` | `Ack` | Drive request. Accepted only in CH5 position 1; see [Command](#command). |
| `Subscribe{slot, ids, period_us}` | `Ack` with `value` = slot | Stream the entries `ids` (table indices) every `period_us` (0 = every control tick) as `SignalBatch` frames. `slot = 0` asks the robot for a free slot; the `Ack` returns it in `value` as a u32. A nonzero slot renews the 5 s lease or replaces the entry list. |
| `Unsubscribe{slot}` | `Ack` | Stop the slot. A free slot is accepted. |
| `GetParam{id}`, `SetParam{id, value}` | `Ack` with `value` | Read or write one entry. `value` is 4 bytes little-endian, typed by the catalog. An applied `SetParam` also sends a `Log` (tag `param`, text `name: old -> new`) on each port, whichever port asked. |
| `Call{id, args}` | `Ack` | Start an action; `args` are floats in catalog order, checked against their bounds before the handler runs. A started action also sends a `Log` (tag `action`) on each port. |

- `Ack.command_id` echoes the request's `Message.seq`, for every request including `Command`. A refused request has `accepted = false` and the reason in `error`.
- While balance is requested (CH5 not at 0, or the balance controller running) a `SetParam` of a parameter with `live_step = 0` is refused with `<name>: refused while balancing`. A parameter with a live step may change by at most that step: a larger change is refused with `<name>: step <d> over <step> while balancing` (`%g` for both numbers). The comparison allows a relative slack of 1e-6 so that a change of exactly one step is accepted despite float rounding. Every other `SetParam` check (type, size, cross-field rules) still applies. `GetParam`, `Subscribe` and the `SignalBatch` rows are not affected.
- The robot owns the slot table: 8 slots shared by all ports. A slot belongs to the port that subscribed it. `Subscribe` or `Unsubscribe` naming another port's slot is refused with `slot in use`, so two clients on two ports never collide.
- A subscription ends 5 s after the last `Subscribe` for its slot. The robot then sends a `Log` (`WARN`, tag `link`, `slot <n> lease expired`) on each port. The check runs at the end of a control tick, after the requests of the next tick start have been applied, so a renewal that waited out a long stall still counts.
- A new `Subscribe` is refused when the port's streaming load would pass 70 % of its line rate: the sum over the port's subscriptions of `(4 + 4 × entries + 26) × rate`, where 26 bytes is the frame cost of each row. UART1 allows 140 kB/s at 2000000 baud minus 5 kB/s kept for odometry and `Imu`; USB allows 40.3 kB/s at 576000 baud. A period of 0 counts as 710 rows per second. The error names both numbers: `over budget: 78.1 kB/s of 40.3 kB/s allowed`. Other refusals: `all 8 slots are in use`, `no subscription in slot <n>`, `not a signal: <id>` (past the table, or an action) and `need 1 to 48 entries, got <n>` and `period_us must be at most 60000000`.
- `SIGNAL_MIN` and `SIGNAL_MAX` entries report the lowest or highest value of every control tick since the previous row of the same subscription. Both restart after each row.
- A request with more than 48 ids does not decode (nanopb rejects it) and gets no reply. The library never builds one.
- `SignalBatch.rows` holds one row of `[t_us u32][4-byte value per id]`, little-endian, per frame. `t_us` is `micros()` at the start of the control tick.
- `Log.tag` names the source of a log line. Every `Logging::message` of the firmware goes out as a `Log` frame on each port, UART1 and USB, and so does every applied `SetParam` and started action.
- nanopb bounds: 48 ids per subscription (196-byte row), 4 action arguments, 24-character names. The largest `Message` encodes to 229 bytes, below the 252-byte payload limit.

## Rules

- Receivers ignore unknown fields and unknown `body` types.
- The sender never blocks and never writes a UART from the control loop. It encodes the frame and pushes it into the port's TX queue; a full queue drops the frame and counts it. The counts are signals of the [signal table](signal-table.md#health-balance-state-and-link-signals) (`link_tx_drop_uart`, `link_tx_drop_usb`, `link_rx_bad_uart`, `link_rx_bad_usb`, `link_req_dropped`, `link_undecodable`), not a periodic log line.
- `seq` is a counter per port (UART1 and the USB port each start at 1 after a reset). A dropped frame still uses its number, so a gap on a port means a frame was lost on that port.
- A receiver treats a frame with a bad CRC as lost. It does not try to repair it.
- The robot's writer task hands each frame to the port whole, with no pause inside it: it writes a frame only when `availableForWrite()` covers all of it, otherwise it waits one tick. The PC library treats bytes still waiting for a delimiter after 250 ms of silence as text (a boot message or crash dump without a trailing `0x00`), so a gap inside one frame must stay well below 250 ms.
- Safety does not depend on this link. The robot keeps balance and fall handling locally and must stop or hold safely when `Command` frames stop.

## Command

`Command` is the drive request from the ros2_pc: `speed_mps` (m/s, forward positive) and `turn_rad_s` (rad/s, positive to the left, counter-clockwise, REP 103). The robot takes it only with CH5 in the link position. The CH5 switch has three positions (`ch5_position`, the raw S.BUS value of CH5):

| Position | Raw value | Balance enable | Motion demand | Link may |
| --- | --- | --- | --- | --- |
| 0 (end) | below 592 | off | none | `reboot` and every `set` |
| 1 (middle) | 592 to 1401 | the link flag `link_balance` (signal) | the stored `Command`, zero when it is older than 250 ms | `balance`, `Command`, live `set` |
| 2 (other end) | 1402 and above | on | the RC sticks (CH3, CH4) | live `set` only |

- A `Command` outside position 1 is refused with `CH5 not in link position (1)`. The refusals in order are `sensor diagnostic build has no motor control`, `CH5 not in link position (1)`, `CH5 position change pending` (the switch moved on the last RC frame, so the position was not yet in effect), and `speed and turn must be finite numbers`. An accepted `Command` replaces the stored one.
- A `Command` older than 250 ms counts as lost: the demand goes to zero and the controller keeps balancing in place. The stored `Command` is then cleared and stays expired until a new one arrives, so a `micros()` wrap cannot revive it. The demand is capped and ramped by the controller (`speed_cap`, `yaw_rate_cap`, see the [signal table](signal-table.md#parameters-settings-groups)).
- The RC stays the authority. Position 0 always switches balance off. An RC loss counts as a lost request, as before. CH6, CH7 and CH9 keep their RC functions in every position.
- Moves between positions:
  - 2 → 1: a balancing robot is handed to the link and keeps balancing, with zero demand until a `Command` arrives.
  - 2 → 0 (through 1): balance stays on for the few milliseconds of the transit and ends at 0.
  - 0 → 2 (through 1): nothing starts in position 1, because the controller is off when 1 is entered; position 2 then starts balance as the RC start does.
  - 0 → 1: nothing starts until the link sends `balance on=1`.
  - Leaving position 1 clears `link_balance` and the stored `Command`.
- The thresholds 592 and 1402 are not yet checked on the robot ([roadmap](../plans/roadmap.md#link-autonomy-on-the-robot)).

## Link multiplexer on the ros2_pc

The serial port can be opened by one program only, so `navbot_link.mux` owns `/dev/ttyAMA3` (2000000 baud) and shares it with any number of local clients over TCP at `127.0.0.1:7650`. It runs as `navbot-link-mux.service`, which `scripts/install_link_bridge_service.py` writes and starts before the bridge. Clients connect with `tcp:127.0.0.1:7650`; they speak the unchanged frame format.

- Each client has its own `seq`. A frame the robot sends to another client does not look like a loss to this one. A real gap on the robot is a gap for every client; a robot restart (a non-increasing `seq`) clears the slot table and restarts every client's counter, so each client sees the `Restarted` event.
- A request is renumbered with the multiplexer's counter and its reply goes back to the asking client only. A request with no reply within 5 s is dropped.
- A `Subscribe` or `Unsubscribe` naming a slot that another client owns is refused by the multiplexer with `slot in use`, without reaching the robot. An accepted subscribe assigns the slot to its client.
- `CatalogEntry` replies go to the clients waiting for that index; `SignalBatch` rows go to the owner of the slot. `Log`, odometry, IMU and other frames go to every client, as do text chunks (boot output).
- A client that disconnects has its slots unsubscribed on the robot. A client whose unsent output exceeds 256 kB is disconnected.
- A serial error ends the multiplexer with status 1; systemd restarts it.

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
- Firmware, pure C++ with host tests: `link_frame` (frames one `Message`; `LinkRxDecoder` splits a received stream into frames), `link_tx` (`LinkItemQueue`, `LinkWriter`, `LinkPort` with the per-port `seq`), `signal_table.h` and `signal_table_expand.h` (the entry table, built from `signals.def.h`, with typed 4-byte values and build-time name checks), `link_subscriptions` (slots, ownership, leases, budget, running minimum and maximum, row layout) and `link_endpoint` (decodes a request and replies on its port; applies `SetParam` and `Call` through the table), `link_log` (`LinkSendLog`: one Log frame on each port), `settings_groups.h` (the rules a settings group applies to a candidate) and `rc_mode.h` (the CH5 positions and the `Command` age, without Arduino). Glue for the board: `link_uart` (UART1 init, one lock, queue and writer task per port, one receive task per port, the request queue, odometry and IMU publish, and `LinkPublishLog`, which `Logging::message` calls).
- PC: the Python package [navbot_link](../../../src/navbot_link/) has no ROS dependency and is shared by the ROS 2 bridge and the host tools. [framing.py](../../../src/navbot_link/framing.py) holds `encode_frame` and `FrameDecoder`, which returns payloads and, for a printable chunk that is not a valid frame, `Text`. `protocol.py` holds the catalog types, the 4-byte value codec and the per-port `seq` tracker; `link.py` holds the `Link` client and `capture.py` the CSV writer and presets used by [navlink](navlink.md). `transport.py` opens the serial port or the multiplexer's TCP address, `mux.py` is the [multiplexer](#link-multiplexer-on-the-ros2_pc), and `health.py` names the health signals and the balance enum values.
- ROS 2 bridge: [navbot_link_bridge](../../../src/ros2_pc/ros2/src/navbot_link_bridge/) (`bridge.py` decodes frames, `odometry.py` integrates wheel angles, `node.py` is the rclpy node). Its `setup.py` installs `navbot_link` next to the bridge. Build and run steps: [ros2_pc software](navbot-link-bridge.md).
- Tests: `PYTHONPATH=src:src/ros2_pc/ros2/src/navbot_link_bridge python3 -m pytest -q src/ros2_pc/ros2/src/navbot_link_bridge/test` (bridge: the firmware literal frames including the IMU `t_us`, seq gaps and restarts; odometry: straight line, spin, `micros()` wrap) and `python3 -m pytest -q tests/navbot_link` (framing: CRC check value `0x29B1`, COBS vectors, resync after garbage, raw text between USB frames; the `Link` client against a fake firmware and, in `test_firmware_endpoint.py`, against the firmware's own endpoint code built for the host) and `python3 -m pytest -q tests/scripts` (`navlink`, its presets, `signals_catalog.py`).
- Host test: `bash tests/link/run_host_test.sh` encodes an odometry and an IMU `Message` (with `t_us`) with the firmware encoder and compares the frame bytes to literal vectors. It also checks the TX path with literal bytes: a frame is written whole or not at all under partial `availableForWrite()`, a full queue drops and counts (and the next frame shows the seq gap), each port counts `seq` on its own, and the USB writer puts `0x00` before each frame and holds a frame until the sink has room for all of it. `test_link_log` sends `Logging::message` lines through the same queues and reads both ports back: both get the same line, USB has the `0x00`, text is cut at 96 characters and tags at 16. The receive and endpoint tests feed frames built by the PC library and compare the reply bytes: catalog entries, `Subscribe` accepted and refused with the budget numbers, lease expiry, slot ownership, running minimum and maximum, the row layout. A build of the real `signals.def.h` against stand-in state is compared with `scripts/signals_catalog.py`, and three bad entry files (a duplicate name, a name over 24 characters, a 64-bit value) must stop the build.

## Transmit path

```text
control loop ── Log, replies, rows ──► TX queue UART1 ──► writer task ──► Serial1 (2000000 baud)
control loop ── Log, replies, rows ──► TX queue USB   ──► writer task ──► Serial  (576000 baud, CH340)
control loop ── odometry, Imu     ──► TX queue UART1
```

- Each port has a byte-ring queue of whole frames (1024 bytes on UART1, 2560 on USB), a spinlock around it and one writer task (core 0, priority 2). `LinkWriter::Pump` is the only code that writes to a UART. The control loop only pushes.
- Each port has its own `seq`, so a gap in a port's `seq` is a lost frame on that port.
- A frame is written whole or not at all. On USB a `0x00` goes in front of each frame.
- Log messages go to both ports. Replies and rows go to the port they concern. None of them waits: a full queue drops and counts. Odometry and `Imu` frames are sent on UART1 and not on USB.
- The firmware writes no text of its own. Setup messages, IMU timing, SimpleFOC debug output and the ICM42688 device check all go through `Logging::message`, so they reach the ros2_pc as `Log` frames.

## Receive path

```text
Serial1 (UART1) -> receive task: COBS, CRC -+
                                            +-> request queue -> control loop, tick start: LinkEndpoint -> reply on the same port
Serial  (USB)   -> receive task: COBS, CRC -+                     control loop, tick end:   rows to the owner of each due slot
```

- One receive task per port (core 0, priority 2) polls its UART every tick, feeds `LinkRxDecoder` and pushes each valid payload into the request queue (1536 bytes, each item tagged with the port). It touches no robot state. A chunk with a bad COBS structure or CRC is dropped and counted (`link_rx_bad_uart` and `link_rx_bad_usb` signals). A full request queue drops the request and counts it (`reqDropped`); the PC retries.
- The control loop takes up to 2 requests at the start of each tick (`LinkServiceRequests`, `LINK_REQUESTS_PER_TICK`), so a tick never sees a half-applied change, and replies on the port that asked. Rows are sampled at the end of the tick (`LinkSampleSignals`), so `control_execution_us` is the value of the same tick.
- UART1 receives into a 1024-byte driver buffer, about 5 ms of line.
- The USB port carries the same frames. The PC writes a `0x00` before each frame, so a frame never follows other bytes in one chunk. A chunk that fails COBS or CRC is dropped and counted; the PC library retries.
- A catalog read costs one control tick per entry: the PC asks for one entry and waits for its reply, and requests are applied at the start of a tick. The cap of 2 requests per tick does not slow it.

## Next steps

1. Done on 2026-10-26: pins, levels and the 500 kBd link work on the robot. The cause of the earlier failure was an unpowered STC8H1K08 on the line ([lab note](../../../lab/robot/2026-10-25-stc8h1k08-backpowers-uart1.md)). GPIO19/20 are the USB Serial/JTAG pads and power up in USB mode, so `LinkUartInit()` releases them (`USB_SERIAL_JTAG.conf0.usb_pad_enable = 0`) before opening UART1 and logs `usbPad=` at boot. A build without this release also carried clean frames, so the release is a safeguard, not the fix.
2. Done: the track width is 113 mm and the forward sign was checked (both wheel angles grow when rolled forward). The left/right mapping was checked with a turn test on 2026-10-26 ([lab note](../../../lab/integration/2026-10-26-odometry-turn-test.md)); the `/odom` twist signs are not yet checked. The bridge takes the wheel radius and the track width as required parameters.
3. Done in the code, not on the robot: the ROS 2 bridge maps `Log` to `/rosout`, the health signals to `/diagnostics` and `/battery_state`, and `/cmd_vel` to `Command`. `Battery` and `Status` are removed from the protocol.
