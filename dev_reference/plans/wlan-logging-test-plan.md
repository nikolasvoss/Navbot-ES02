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
8. Run the new scripts' help, usage, status mock, fake fragmented stream and decoder paths. Assess hardware timing, movement, throughput, and battery behavior separately from host tests.

## Verification run

Executed on the current `wifi-parameter-terminal` checkout:

- The C++ golden record/frame, CRC vector, SPSC ring capacity/full/empty, and session prepare/ready/start/duration/stop/drain/ack paths passed `tests/telemetry_wire_test.cpp` with `-Wall -Wextra -Werror -pedantic`.
- Existing tuning parameter C++ tests passed.
- The Python WLAN decoder, fragmented receive, corruption/truncation, `.partial` byte retention, no-overwrite publication, config permissions, stream handshake exclusion, status mock, and existing tuning CLI tests passed (21 tests total).
- Existing drive analyzer Python tests passed (3 tests).
- Recorder and decoder `--help` commands passed. Baseline, recording-disabled and recording-enabled firmware builds passed with Arduino CLI 1.5.1, ESP32 core 3.3.11, SimpleFOC 2.3.4 and ArduinoJson 7.4.3.

Not covered by host tests: live firmware session lifecycle and leases, stream-ticket rejection on-device, wrong IDs against HTTP handlers, config drift in firmware, sender disconnect and socket timeout behavior, ring reuse across repeated live sessions, disk-full injection, CTRL-C against a live robot, and stalled-sender control-loop timing. Do not treat the current tests as hardware acceptance.

## Live follow-up on 30.09.2026

Two consecutive 20-second recordings and a third run stopped by `SIGINT` passed on a secured robot. The client saved and ACKed all three files. The decoder found zero CRC, sequence, and numeric faults in each file. The repeated run exercised ring and session reuse; the interrupted run produced a valid `USER_STOP` footer. The remaining live cases from the list above are still open except for reuse and CTRL-C. The [hardware observations](../../agent_notes/wireless-logger-hardware.md#wlan-aufnahmen-am-aufgebauten-roboter-am-30092026) record measurements and limits.

### Control timing follow-up on 30.09.2026

With CH5 off, the 100 Hz capture path measured median `control_dt` 2.354 ms. Batching producer notifications alone did not form larger batches because the sender woke every 20 ms; median timing was 2.397 ms and ring high-water was 2. The follow-up build waits up to 60 ms and notifies every five records. The resulting 20-second capture had 1,846 records, ring high-water 5, median `control_dt` 2.032 ms, p95 2.813 ms, and zero CRC, sequence, or numeric errors. This is close to the prior no-recording CH5-off K58 median of 2.104 ms.

A separate 20-second capture with CH5=1 on the pre-batching 100 Hz build had 1,778 active records, median `control_dt` 2.481 ms, and p95 3.221 ms, with no tumble or data faults. The user reported that measurement still affected behavior. A post-batching 20-second CH5=1 capture then completed with 1,768 active records, ring high-water 5, median `control_dt` 2.128 ms, p95 2.948 ms, and zero CRC, sequence, numeric, or overflow faults. The same-build CH5-off run measured median `control_dt` 2.032 ms and p95 2.813 ms. This comparison also changes regulator state, so it cannot isolate logging overhead.

A same-boot CH5=1 comparison then captured a 20-second K58 baseline with Wi-Fi recording idle and a 20-second Wi-Fi capture. The baseline had 393 samples at 20 Hz, median/p95 `control_dt` 2.122/2.762 ms. The Wi-Fi capture had 1,777 samples at about 89 Hz, median/p95 2.117/2.892 ms, ring high-water 5, and no transport or numeric faults. The median stayed nearly the same, while p95 rose by 130 µs. The run windows were 4.3 minutes apart, and the sampling rates differ. Treat this as a timing estimate. It does not settle the user's report of changed balance behavior. Movement remains untested.
