# Implementation brief: buffered Wi-Fi recording

Prepared 2026-09-28 for **GPT-6 Luna, reasoning high**. This is a self-contained implementation assignment: no conversation history or previous plan is needed. Planning only; no implementation or worktree has been started by writing this document.

Engineering planning status: **clean for the scope below**. Hardware performance remains an acceptance test, not an assumed result. Risk tier: **high**, because adding runtime load can affect a balancing robot. Use `plan-eng-review` before implementation, then `security-review`, `cli-qa`, `review`, and `document-release` as applicable. Read their actual instructions when executing them.

## 1. User requirements and non-goals

The user tunes a Navbot-ES02 from a Linux Mint computer on the home Wi-Fi network. They want regulator/sensor data to diagnose pitch/tilt vibration and driving faults. Implement computer-triggered recordings of **20–40 seconds, default 30 seconds**, capturing every actual balance-control tick. Streaming is explicitly allowed. Data may be lost on robot reboot; measurements need not survive power failure. No USB cable should be required during recording.

Use **binary TCP streaming, a bounded internal-RAM buffer on the robot, and continuous file writing on the computer**. The buffer absorbs short network stalls, not the whole recording. Keep existing RC driving independent of the PC. Logging failures stop logging, never block the control loop or change motor commands.

Hardware confirmed by user: **ESP32-S3-WROOM-1, no external RAM, existing pin assignment correct**. Do not enable PSRAM or change wiring. A contrary N8R8 designation in the design export is not evidence of usable PSRAM.

Out of scope: regulator redesign, changing gains/defaults, motor control from the PC, autonomous driving trials, SD/flash logging, OTA, cloud, AP mode, browser UI, live plotting, compression, IMU FIFO/interrupt redesign, guaranteed fixed-rate 1-kHz sampling, recovery of the same recording after TCP reconnect. Preserve parameter tuning if present; do not implement a second tuning product if absent. Do not flash hardware without a separate instruction.

## 2. Worktree and dependency handling

Source workspace: `/home/niko/Dokumente/Bastelei/roboter/Navbot-ES02`. Read its `AGENTS.md` before any project inspection, then `agent_notes/hardware-overview.md` and `agent_notes/usb-serial.md`. Inspect the relevant current code rather than relying on stale line numbers.

The original checkout has important **uncommitted and untracked changes**. A worktree from HEAD alone loses the current robot behavior. The executor must:

1. Record HEAD, branch and `git status --short`; create a temporary snapshot and hash manifest of the current sources. Confirm originals have not changed while copying. If they have, recapture a coherent snapshot.
2. Create an isolated worktree, e.g. sibling `Navbot-ES02-wifi-logging`, on an unused branch such as `wifi-buffered-logging`. Never overwrite an existing worktree/branch. A managed Codex worktree is also suitable; use its returned path explicitly.
3. Apply tracked working-copy changes, including deletions, using a binary diff against captured HEAD. Copy needed untracked code/docs/tests from the sketch, `scripts/`, `agent_notes/`, `dev_reference/`, and `AGENTS.md`. Exclude builds, logs, caches, images and secret configuration. Preserve existing ignore rules; do not bulk-add everything.
4. Verify hashes, inspect for secrets, and create a **local baseline commit in the worktree**. Do not commit, stash, reset or edit the original checkout. Compare the eventual feature diff against this baseline, not old HEAD.
5. Compile the baseline first and record Arduino core/library versions. If parameter-tuning work has since landed in the selected checkout, include it in the baseline. Do not silently cherry-pick an arbitrary branch or assume another worktree is authoritative.

At plan time this checkout contains no implemented Wi-Fi parameter service. Two supported starting conditions:

- **Service exists at execution time:** reuse its station connection, credentials, authentication, boot/build identity, HTTP server, bounded control queue and parameter lock. Extend those components; do not start competing servers/tasks or rename its public APIs unnecessarily.
- **Service absent:** add only the minimal shared WLAN station, authenticated HTTP recording controls and data socket described here. Keep Serial for changing parameters before recording. Do not implement wireless parameter editing/profile management as a dependency. Isolate the small transport utility so a future parameter service can reuse it.

Explain the chosen path in the implementation report. Missing Wi-Fi code is not a reason to stop the whole assignment.

## 3. Existing code to inspect and preserve

Sketch: `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`; also inspect `filter.*`, `ICM42688.*`, `FUTABA_SBUS.*`, `ble.*`, `robot.*`, and existing Python capture/analysis scripts.

- `loop()` executes motor FOC and sensor work, then enters the balance-control section when `time_dt >= 0.001`. This is **not a fixed 1-kHz scheduler**. Prior observations around 1.664 ms are historical, not a promise of 600 Hz under new network load.
- `print_data()` K58 currently emits a serial DRIVE row approximately every 50 ms. Its position precedes later controller calculations; do not reuse it as the high-rate snapshot point.
- At the end of the timed control section, after regulator/tumble/servo handling and before `now_us1 = now_us`, take one coherent snapshot. Read existing values only; no extra SPI, I²C, ADC or servo operations for logging.
- `ImuUpdate()` polls an IMU configured for 1-kHz ODR, with software filters and attitude estimation. Timestamp **MCU read completion** for age information without changing filters, reads or ODR. Do not label this as sensor conversion time or proof of fresh samples.
- `AnglePid`/`SpeedPid` etc. are configuration objects; `Angle_Pid`/`Speed_Pid` etc. are runtime states with potentially scaled/effective gains. Record that distinction. `PidParameterTuning==0` can replace configured gains automatically.
- Preserve current `V`/`wheelSpeedFeedbackGain`, `wheelSpeedFeedbackOutput`, `balancePidNeedsPriming`, actual-dt wheel estimation, CH3 scaling, gains and all motor/fall logic. Re-read names if later code moved them.
- A BLE connection currently affects the driving input path. WLAN recording must not set `rp.ble_connected` or impersonate BLE driving. Reuse the tuning build's BLE policy if present; otherwise skip BLE init/task only in the explicitly enabled WLAN build. Default legacy build remains unchanged.

Recording supports the existing **two-wheel master balance path** only. Report unsupported diagnostic-only, slave or four-wheel builds clearly. It must record while CH5 is OFF and while ON; changing CH5 is part of a useful trial. Do not apply parameter-write safety gates to read-only acquisition. Never automatically enable CH5.

## 4. Firmware components and real-time contract

Suggested modules: `TelemetryRecorder.{h,cpp}`, `TelemetryWire.{h,cpp}`, `TelemetryTransport.{h,cpp}` plus minimal main-sketch hooks. Keep pure state/format logic host-testable. Reuse existing naming conventions when sensible.

**Producer:** the control loop owns start/stop transitions, metadata snapshot and sample generation. Each control tick performs at most one fixed-size record copy into an already allocated single-producer/single-consumer buffer. No sockets, JSON, allocation/free, filesystem, CRC scan, waiting on a network lock, or unbounded loops in the producer.

**Consumer:** a separate lower-priority sender task batches records and owns the data socket. Block/yield while idle, never busy-spin. Use correct cross-core synchronization; `volatile` alone is insufficient. Prefer bounded SPSC ownership with acquire/release publication and a tested implementation, or very short metadata-only critical sections. Do not hold a critical section around socket calls or bulk serialization. Preserve existing control-task placement and verify priorities; another core alone does not guarantee isolation from Wi-Fi interrupts or shared memory load.

Target a **64-KiB internal record ring**, 512 records × 128 bytes, allocated once before acquisition, plus a bounded 4-KiB sender staging buffer. Include TCP buffers, task stacks, metadata and control service in the memory budget. If 64 KiB cannot be reserved with operating headroom, report the actual supported capacity; optionally use an explicitly advertised 32-KiB configuration. Do not silently exhaust the heap or allocate repeatedly during a run. Reuse buffers across sessions and free only when both owners have stopped.

At 600 Hz, 128-byte records generate 76.8 kB/s; at 1 kHz, 128 kB/s. Forty seconds produces 3.072 MB or 5.12 MB on the PC. An empty 64-KiB ring covers at most 0.85 s or 0.51 s of no progress. Actual reserve is lower when occupied. Expose capacity and high-water mark. Sender must have spare throughput to drain a backlog.

Batch up to **32 records (4096 bytes)**, or flush a nonempty batch after 20 ms. Nonblocking BSD socket calls with `select`/bounded waits must handle partial `send`, `EAGAIN`/`EWOULDBLOCK`, `EINTR`, close and errors. Retain staging data until fully accepted by the socket; never release and reuse bytes still referenced by a pending send. `send` success means accepted by TCP, not saved by the PC. Check APIs against the installed Arduino core/ESP-IDF, not current online examples alone.

Temporarily suppress existing periodic Serial traces during capture to avoid perturbing timing; restore the previous trace selection afterward. Preserve UART parsing and RC safety. Rate-limit/defer error prints reached in the timed section to counters while recording. Do not add synchronous high-rate debug output. Outside recording, preserve legacy telemetry behavior.

## 5. Fixed record schema v1

One profile initially: `balance_v1`, **128 bytes**. Encode explicitly little-endian; no raw ABI-dependent struct dump. Float format IEEE-754 binary32. C++ size assertions and an independent Python golden fixture must agree.

Header (24 bytes):

| Offset | Type | Meaning |
|---|---|---|
| 0 | u64 | MCU monotonic snapshot time in microseconds |
| 8 | u32 | sample sequence, starts at zero for this recording |
| 12 | u32 | flags defined below |
| 16 | u32 | age in µs since last completed MCU IMU read; UINT32_MAX if unavailable |
| 20 | u32 | actual control start-to-start dt in µs |

Then 26 float32 values, in this exact order (104 bytes):

1. `roll_ok`
2. `BodyPitching_f` (stored variable, not silently sign-inverted into the angle setpoint)
3–5. `attitude.gyrof.x`, `.y`, `.z`
6–7. `MovementSpeed`, `BodyTurn`
8–9. `Motor1_Velocity_f`, `Motor2_Velocity_f`
10–14. `Speed_Pid.error`, `.outP`, `.outI`, `.outD`, `.output`
15–19. `Angle_Pid.error`, `.outP`, `.outI`, `.outD`, `.output`
20. `Yaw_Pid.output`
21. `BodyX`
22–23. `motor1.target`, `motor2.target`
24. `wheelSpeedFeedbackOutput`
25. `Voltage`
26. `wheelSpeedFeedbackGain`

Flags: bits 0–1 = current `pid_gains_mode` (0/1/2); bit 2 = tumble; bit 3 = regulator calculated this tick; bit 4 = RC valid/fresh if reliably available; bit 5 = IMU read timestamp available; bit 6 = numeric fault in expected-valid data; bits 7–8 = `roll_mode`, bits 9–10 = `attitude_mode`, bits 11–12 = `posture_or_mark_mode`; remaining bits zero. Validate current mode enum values fit; if not, change the schema version before implementation rather than truncating values. Track validity with actual branches, not just a guess based on CH5.

When no regulator calculation ran, encode runtime PID fields 10–20 as canonical quiet NaN and clear bit 3, instead of presenting stale values as current. Other fields reflect the actual snapshot. Nonfinite expected-valid values set bit 6, remain visible, and make the quality report flag sensor/controller invalidity; do not silently drop those records or confuse them with transport failure.

Record units and sign conventions in metadata, traced to current calculations. Gyro is filtered rad/s, voltage is the existing battery estimate, motor targets are commands rather than measured currents. Do not assume wheel units or the sign of `BodyPitching_f`; document actual code semantics. No extra unmeasured “voltage minimum” field invented to satisfy old analysis code.

Use `esp_timer_get_time()` or an equivalent monotonic 64-bit device clock for timestamps; avoid rollover-prone 32-bit absolute `micros()`. Arrival time at the PC is not measurement time. Records may have irregular spacing; report actual rate/jitter. Neither µs timestamps nor 1-kHz IMU ODR imply µs accuracy or independent 1-kHz samples.

## 6. Wire framing and saved file

Default data socket TCP port **8766**, one authenticated client. The control API advertises the port. No UDP, compression or custom retransmission protocol. TCP is a byte stream: reads may split or combine headers/payloads arbitrarily.

After authentication (next section), server sends frames with a **32-byte little-endian header**:

| Offset | Field |
|---|---|
| 0 | 4 bytes ASCII `NBL1` |
| 4 | u16 protocol version = 1 |
| 6 | u16 type: 1 META, 2 DATA, 3 END |
| 8 | u64 recording_id |
| 16 | u32 frame_sequence, consecutive starting at 0 |
| 20 | u32 payload_length |
| 24 | u32 record_count (DATA only, otherwise 0) |
| 28 | u32 CRC32 of header bytes 0…27 concatenated with payload |

Use standard CRC-32/ISO-HDLC compatible with Python `zlib.crc32`, check vector `123456789` → `0xcbf43926`. CRC is corruption detection, not authentication. Sender computes it off the control loop. Payload limits: META ≤8192 bytes, DATA 1…32 records with exact `count*128` length, END ≤4096 bytes. Reject all other lengths/types/version/order without unbounded allocation. Receiver must never scan corrupt data for a guessed new frame boundary and call it complete.

META is UTF-8 JSON, one frame after actual start: schema, boot_id, recording_id, t_start_us, requested duration, build identity/source hash, actual relevant build flags, configured gains, effective scaling rules, U/V, filter/ODR/calibration configuration, units/field order, buffer capacity, mode interpretation. Snapshot config in loop once; serialize outside it. Do not include credentials or token. If metadata cannot fit, fail prepare explicitly rather than truncate.

DATA contains the records above. END JSON contains reason (`DURATION`, `USER_STOP`, or failure code), generated/queued/sent record counts, first/last timestamps and sequence, stop timestamp, requested/actual elapsed duration, ring high-water mark, overflow counters, interval statistics and `records_crc32`. This CRC covers all DATA payload bytes in stream order, excluding frame headers/META/END, accumulated by sender. State stats must use clear counter definitions; on success all record counts match.

PC stores the exact binary frames beginning with META in `OUTPUT.nblog.partial`; no network authentication bytes. Only after complete validation and successful file flush/fsync rename to `OUTPUT.nblog`. Explicit user stop may create a transport-complete but shorter file with reason USER_STOP; report it as stopped, not a full-duration run. Failures keep `.partial` and an error summary; truncated headers/payloads are reported with last valid offset and records retained.

## 7. Session/control API and lifetime

Use existing HTTP authentication if present; otherwise implement station-only HTTP on port 80 with `Authorization: Bearer <device token>`, small bounded requests and a control queue. Reuse one random boot ID per reboot, represented as hex in JSON, and unique u64 recording IDs represented as hex strings to avoid numeric-precision ambiguity. Maximum control JSON body 1024 bytes. No secrets in URLs.

Endpoints below are new extensions to `/api/v1/` (do not replace existing tuning endpoints):

- `GET /recording/status`: capability/schema, boot ID, state, recording ID, duration, counts, buffer use, failure reason, stream readiness. Stable terminal status retained until next prepare/release.
- `POST /recording/prepare`: `{request_id, expected_boot_id, duration_s:30, profile:"balance_v1"}`. Validate 20…40 integer seconds, supported mode, memory, manual tuning U=1, calibration inactive, no existing session. Snapshot configuration/lock it and reserve session. Return recording ID, one-use random stream ticket (≥128 bits), port and 15-s prepare lease. Do not change gains or U to satisfy this request.
- Data client connects and sends one bounded UTF-8 JSON line `{protocol:1, boot_id, recording_id, ticket}` ≤512 bytes within 3 seconds. Server validates ticket/session once and responds `OK\n`, or closes on error. Handshake is not written into `.nblog`. Status now reports ready. No sample generation yet.
- `POST /recording/start`: `{request_id, expected_boot_id, recording_id}`. Require matching prepared session and ready data client. Apply in loop at next capture boundary, generate first sample there, return actual device t_start_us. META carries the same authoritative t_start_us. A retry cannot start a second session.
- `POST /recording/stop`: same identifiers; idempotent stop, does not change motor state. Prepared session cancels without a file; recording session stops at next boundary and drains.
- `POST /recording/ack`: identifiers plus received record count and records_crc32. Sent after PC validation/fsync/rename; matches END. Idempotent, marks receiver-confirmed completion. Lost ACK response does not invalidate an already verified PC file; status reports confirmation separately.
- `POST /recording/release`: terminal session only; release ownership/reusable buffers, not the PC file. Active requests rejected, use stop first.

State machine: `IDLE → PREPARED → RECORDING → DRAINING → AWAIT_ACK → COMPLETE`; failure terminals `FAILED` and `UNCONFIRMED` (valid END sent but no receiver ACK). User stop follows draining with reason USER_STOP. Prepare lease 15 s; drain limit 5 s after stop; ACK lease 10 s. Expiry releases parameter lock and sender resources while retaining a small terminal summary. Allow new prepare after terminal cleanup; repeated runs must not leak resources. Never reuse the ring while an old sender still references it.

Duration measured on the robot. Before enqueueing each sample, stop if its boundary is at/after `t_start_us + duration_s*1e6`; record actual first/last/stop times. Do not stop based on number of nominal 1-kHz samples or PC wall clock. Stop and queue-full errors cannot be lost because the data ring is full; keep terminal status/counters separately.

HTTP errors: 400 invalid fields, 401 unauthorized, 409 wrong state/ID/boot/config/unsupported mode, 413 body too large, 503 resource/queue busy, 504 command expired. All responses have `ok`, `request_id` where applicable, `boot_id` and stable error code. Loop applies bounded commands with deadlines; network callbacks never mutate recording/control state directly. Duplicate mutating request IDs return cached outcome or current same-session outcome; conflicting reuse rejected. Client must not blindly retry START after uncertainty: query status/ID first.

## 8. Configuration consistency and fault behavior

From prepare through capture/drain, reject network gain/calibration/flash mutations with `RECORDING_BUSY`. If a parameter service exists, add one centralized mutation gate. Do not disable read-only tuning/status.

For legacy Serial, intercept known mutating commands before execution while the session is locked; continue consuming input in a bounded manner, reject rather than defer it. No accumulated commands that execute unexpectedly after capture. Read-only queries may remain if they cannot cause high-rate output. Prevent calibration/FlashSave initiation during recording. Do not pause SBUS reception or disable RC commands; mode/sollvalue changes are recorded in flags/data. Avoid a general Commander rewrite: use a narrow session gate at its dispatch boundary, with tests against actual callback paths.

Manual U=1 avoids automatic gain reassignment. Include all relevant configured values in a small fixed snapshot/fingerprint and check for unexpected changes at the same capture boundary. An unguarded mutation ends acquisition with `CONFIG_CHANGED` before a sample using the changed configuration can be reported as part of a stable-config run. Ordinary RC values/modes are not configuration mutations. Read-only status must distinguish configured gains from mode-dependent effective gains (e.g. yaw Ki suppression).

- Ring full: fail recorder with `BUFFER_FULL`, never overwrite, block or lower rate. Drain existing valid records if connection permits and send failure END; file remains partial.
- Temporary blocked sends on still-open TCP: keep acquiring until buffer limit, then fail as above. Record stall/high-water statistics.
- Disconnect/PC crash: fail session, preserve PC partial file and terminal summary. No same-session replay/reconnect promise, because TCP-accepted bytes may no longer exist on the robot.
- No Wi-Fi: robot still boots/runs normally; reconnect attempts have bounded backoff off the loop. Recording prepare fails clearly.
- Disk full/PC validation failure: client tries STOP, closes data socket and retains partial file. Independent device duration remains a bound even if STOP is lost.
- Robot reboot: new boot ID; client never appends new-session samples to an old file.
- CTRL-C: first interrupt sends STOP, drains with a finite timeout and marks USER_STOP; second exits promptly preserving partial output.

No logger error triggers a motor-off command or rewrites safety logic. Existing RC/fall responses still govern movement.

## 9. Linux client and useful outputs

Use Python standard library (`socket`, HTTP client, `struct`, `zlib`, `json`, filesystem) unless repository dependencies already justify reuse. Suggested `scripts/wifi_record.py` plus `scripts/decode_wifi_trace.py`. Reuse host/token config from parameter terminal when present; otherwise one local config file with restrictive permissions. No tokens on CLI arguments or in outputs.

```text
python3 scripts/wifi_record.py --host ROBOT_IP --config ~/.config/navbot/connection.json --seconds 30 --output trial.nblog
python3 scripts/wifi_record.py --host ROBOT_IP --config ~/.config/navbot/connection.json --status
python3 scripts/decode_wifi_trace.py trial.nblog --csv trial.csv --report trial-summary.json
```

Before PREPARE create output exclusively and check destination; refuse overwrite by default, optional explicit `--force`. Receiver starts before START so a short race cannot discard META. Consume/validate/write concurrently with the HTTP start request; a single receiver thread or bounded event loop is sufficient, not an unbounded in-memory recording list. No per-sample fsync, plotting or terminal printing. Low-rate progress only, based on device samples/status. File flush/fsync at successful completion before ACK; failure handling must preserve useful partial data.

Decoder supports explicit `--allow-partial` to recover complete valid frames/records up to first error, reports incomplete and exits nonzero. Never silently treat a repaired prefix as a complete experiment. Keep raw device timestamps in CSV, plus relative seconds, flags and units in report metadata. NaN for inactive PID values is expected and distinguished from numeric-fault flag.

Summary must include completion/reason, requested/observed duration, record count, actual dt median/p95/p99/max, sequence/CRC failures, invalid-data counts, ring high-water mark, active/inactive segments, and min/max/RMS tilt within valid active segments. Do not add FFT or claim a vibration frequency from irregular data in this phase. CSV is sufficient for later frequency analysis.

Do not fabricate a legacy K58 log to satisfy `analyze_drive_trace.py`: it expects `voltage_min_raw_v`, which is not measured in this schema. Keep the new decoder/report separate, preserve old scripts and run their tests. An explicit adapter can be a later task.

CLI exits: 0 validated complete requested-duration run (or status success), 2 input/config error, 3 transport failure, 4 device refusal, 5 incomplete/corrupt/user-stopped recording. Document USER_STOP behavior. `--help`, no-argument usage, EOF/interrupts and invalid configs must terminate predictably.

## 10. Secrets, setup and builds

Station-only home network; no AP fallback, router forwarding or Internet dependency. Use one ignored local firmware credentials header and a nonsecret example. Random device token ≥128 bits, no fallback default. Feature-enabled build without configuration must fail clearly; disabled legacy build must not require the header. Dummy credentials for compile tests only. HTTP/TCP are not TLS: this assumes a trusted home network and must not be described as confidential against network observers. One-use tickets prevent accidental stream cross-association, not wire encryption.

Avoid new async-server libraries unless necessary; use installed core/ArduinoJson facilities. No credentials in compile command lines, build IDs, fixtures, manifests or Git. Reuse existing feature flags where present, otherwise one explicit WLAN-recording flag default OFF. Arduino FQBN: `esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default`; PSRAM stays off. Build baseline, feature disabled, and feature enabled with dummy configuration into separate output directories. Record exact versions; never auto-upgrade dependencies to get a pass.

Document initial USB installation and subsequent cable-free operation. Upload helper is `python3 scripts/upload_firmware.py BUILD_DIR`, but only execute on a separately authorized hardware turn after rereading USB notes and rechecking port/board. Merely compiling is not a hardware result.

## 11. Execution order and optional agents

1. Isolate correct baseline and compile. Inspect whether shared Wi-Fi service exists; choose the integration path.
2. Freeze wire schema/API in a small protocol document and golden fixtures. Implement pure pack/unpack, state machine and ring ownership tests.
3. Implement loop snapshot/parameter lock and bounded sender; add minimal controls or reuse existing service. No moving regulator computations.
4. Implement PC receiver/decoder using fake fragmented TCP streams before hardware access.
5. Integrate, run tests/builds, security/diff/CLI reviews and document setup/failures. Then perform hardware acceptance only when separately authorized.

Recommended: one **gpt-6-luna / high** implementer plus independent Luna/high reviewer. Optional parallel implementers after shared protocol is fixed: A owns firmware and C++ tests; B owns PC scripts and Python/mock tests; lead owns integration/protocol/docs. Separate file ownership; same isolated worktree allowed. Separate agent worktrees must start from the shared baseline/protocol commit. Do not delegate two agents to edit the main sketch simultaneously. Give each this full document and explicit worktree path; do not assume chat context.

## 12. Mandatory verification and acceptance

Automated tests must exercise actual C++ encoder/ring/state logic as well as Python decoder; a Python simulation alone is not firmware evidence. Shared known-byte fixtures verify endianness, IEEE floats, NaN, CRC and exact 128/32-byte layouts. Test partial writes, one-byte TCP reads, multiple frames per read, queue/ring wrap, sequence gaps, malformed/oversize payloads, wrong IDs/token/ticket, repeated START/STOP, config change, lease expiry, disconnect, timeout, and resource cleanup across repeated sessions.

Test control loop progress under a deliberately stalled fake sender: no overwritten records, no waits and bounded failure. Test only completed records published to consumer; no ring reuse before sender termination. Test duration by device time with variable tick spacing, not sample count. Test `.partial` retention, disk-full injection, CRC/footer mismatch, ACK loss, CTRL-C and no automatic START replay. Old Python analysis tests still pass.

Hardware targets: repeat 20/30/40-second recordings over home Wi-Fi on battery, disconnected USB, first supported/stationary then ordinary RC movement. Every claimed complete run must have contiguous sequence, valid CRC/footer, zero dropped records and confirmed effective config. Measure baseline vs Wi-Fi idle vs streaming for actual tick median/p99/max, producer execution time, heap/stack headroom, resets and ring occupancy. Aim for producer overhead below 5% of baseline median control interval. Do not accept a new missed control deadline, reset, sustained backlog or material max-jitter regression just because average Hz looks good; establish the robot's acceptable deadline/jitter budget before a driving pass is claimed.

Verify goodput reserve with at least twice the actual sample payload rate under representative load, and deliberately test stalls shorter/longer than measured buffer reserve. Synthetic high-rate transport tests must be bounded and not performed as unreviewed driving experiments. If hardware absent, finish mock/host tests, builds and instructions, label hardware acceptance **not executed**, and do not claim 40-second real-time performance.

Required downstream: `security-review` (high), `cli-qa` full for new scripts, `review` of feature diff, `document-release`. Supporting checklists: [test plan](wlan-logging-test-plan.md), [risk register](wlan-logging-risk-register.md). No ship/flash implied.

Deliver worktree/branch, baseline and feature commits/diff, protocol/golden fixtures, scripts/tests, build evidence, setup and acceptance instructions, concise known limitations. No logging data loss may be hidden; no promise of a complete recording after disconnect.

## Reference sources, not substitutes for installed code

- [Espressif ESP32-S3 lwIP sockets](https://docs.espressif.com/projects/esp-idf/en/v5.1/esp32s3/api-guides/lwip.html): nonblocking sockets, partial/error handling and supported options.
- [Espressif Wi-Fi performance](https://docs.espressif.com/projects/esp-idf/en/v5.3.6/esp32s3/api-guides/wifi.html): laboratory throughput supports plausibility of the data budget, not a robot timing guarantee.

## Copyable launch instruction

> Implement the complete assignment in `dev_reference/plans/wlan-logging-implementation.md` using GPT-6 Luna with reasoning high. Read AGENTS.md first. Create an isolated worktree preserving the current dirty source baseline as specified. Build buffered binary TCP recording for 20–40 seconds, PC-triggered from Linux Mint, using only internal RAM. Reuse Wi-Fi tuning infrastructure if present; otherwise add only the minimal recording transport, not a second parameter-terminal project. Preserve all regulator behavior. Follow the exact schema, bounded producer/sender ownership, configuration locking, partial-file/error semantics and tests in the brief. Optionally delegate the PC client and an independent review to Luna/high with disjoint file ownership. Run host tests, builds and required reviews; document hardware checks that were not executed. Do not flash, add OTA or change motor control. Deliver a reviewable worktree, protocol, Linux client/decoder, tests and setup instructions.
