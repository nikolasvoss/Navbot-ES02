# WLAN logging test plan

## Scope

Automated checks cover the portable 128-byte record codec, 32-byte frame codec, CRC-32/ISO-HDLC, recorder state transitions, bounded SPSC ring, Python receiver/decoder, existing Wi-Fi tuning behavior, and firmware compile configurations. They do not establish real-time performance on the robot.

## Required checks

1. Verify golden little-endian bytes, float32/NaN encoding, CRC vector `123456789`, exact frame and record lengths, and metadata/data/end order.
2. Exercise record ring wrap, full/empty behavior, SPSC publication, nonblocking producer behavior, and no consumer-visible incomplete records.
3. Exercise prepare/start/stop/drain/ack/release, duplicate IDs, boot/session mismatch, ticket reuse, lease expiry, configuration changes, disconnect, and resource reuse.
4. Feed the Python receiver one-byte reads, fragmented headers/payloads, coalesced frames, sequence gaps, bad CRC, invalid order/type/version/length, and truncation at each boundary.
5. Verify `.partial` retention, successful fsync/rename before ACK, USER_STOP classification, disk errors, CTRL-C, no blind START replay, and explicit partial recovery exit status.
6. Run existing tuning parameter C++ tests and Python tests unchanged.
7. Build WLAN recording disabled and enabled with dummy local credentials, plus the current baseline, using the installed core/libraries and separate output directories.
8. Run the new scripts' help, usage, status mock, fake fragmented stream and decoder paths. Hardware timing, movement, throughput and battery acceptance remain a separate, unexecuted gate.

## Verification run

Executed on the current `wifi-parameter-terminal` checkout:

- The C++ golden record/frame, CRC vector, SPSC ring capacity/full/empty, and session prepare/ready/start/duration/stop/drain/ack paths passed `tests/telemetry_wire_test.cpp` with `-Wall -Wextra -Werror -pedantic`.
- Existing tuning parameter C++ tests passed.
- The Python WLAN decoder, fragmented receive, corruption/truncation, `.partial` byte retention, no-overwrite publication, config permissions, stream handshake exclusion, status mock, and existing tuning CLI tests passed (21 tests total).
- Existing drive analyzer Python tests passed (3 tests).
- Recorder and decoder `--help` commands passed. Baseline, recording-disabled and recording-enabled firmware builds passed with Arduino CLI 1.5.1, ESP32 core 3.3.11, SimpleFOC 2.3.4 and ArduinoJson 7.4.3.

Not covered by host tests: live firmware session lifecycle and leases, stream-ticket rejection on-device, wrong IDs against HTTP handlers, config drift in firmware, sender disconnect and socket timeout behavior, ring reuse across repeated live sessions, disk-full injection, CTRL-C against a live robot, and stalled-sender control-loop timing. Do not treat the current tests as hardware acceptance.
