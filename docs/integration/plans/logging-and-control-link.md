# Logging, telemetry and control over the NavBot link

Status: **proposal, not implemented.** It replaces the `K` log selectors, the CSV log profiles and the letter commands with one table of signals, parameters and actions, served over the [NavBot link protocol](../software/ros2-uart-protocol.md) on both the ros2_pc UART and the USB port. An independent review on 2026-10-27 checked an earlier version against the code; its findings are folded in.

## Goals

1. Telemetry to ROS 2: odometry, IMU, battery, status.
2. Control from ROS 2: parameters, test procedures, later drive commands.
3. Log and event messages for `/rosout` and a dashboard.
4. Control debugging: pick one or more variables at runtime and stream them. Debugging typically needs up to about 20 at a high rate; logging may use more at a low rate. Adding or removing a loggable variable is one line and leaves no dead code.
5. The USB port (CH340 on UART0) is a backup with the same functions.
6. Simple: one mechanism per job, no special cases.

## What it replaces

| Today | Problem | Replaced by |
| --- | --- | --- |
| `K1`–`K48`, `K60`–`K75` selected debug | One new variable touches `TelemetryInputs`, the `DebugPayload` union, a `case` in `DebugTelemetry.cpp`, a formatter in `LogFormat.cpp`, and a new number | One `SIGNAL` line |
| `K55`–`K58` TRACE, CTRL, BAL, DRIVE profiles | Fixed structs in firmware; text CSV uses about 3× the bytes of binary | Presets on the PC (signal list + rate) |
| `Q`, `I`, `Z`, `A`–`D` letter commands | One handler per value | `PARAM` lines |
| `T`, `M`, `E` commands | Own grammar each | `ACTION` lines |
| `Y` yaw PID command, `K9`, `K33`, CTRL yaw columns | Dead: `Y` writes `yawPidSettings`, which nothing reads, and `yawPid` stays all zeros (`OllieFOCdrive.ino`, `yawPidSettings`/`yawPid`) | Deleted first, not migrated |

## Overview

```text
            signals.def  (SIGNAL / SIGNAL_MIN / SIGNAL_MAX / PARAM / ACTION)
                 │
   control loop (one task) ──── owns all state ─────────────────────────┐
     tick start: apply queued requests (set, call, subscribe)           │
     tick end:   sample due subscriptions, odometry, IMU → TX queues    │
                 ▲                                   │                  │
         request queue                         TX queue per port        │
                 │                                   ▼                  │
   link task per port: COBS RX decode ──┘     writer: frames → UART     │
        UART1 (ros2_pc, 2 MBd)                UART0 / CH340 (USB backup, 576000 baud)
```

### One protocol, two ports

Both ports carry the same frames: protobuf payload, CRC-16, COBS with a `0x00` delimiter. A reply goes to the port the request came from. Each port has its own `seq` counter, so a gap on a port means a lost frame on that port. Today a single global counter in `link_uart.cpp` would show gaps on both ports.

### Threading: the control loop owns all state

- **RX:** one link task per port decodes frames and pushes requests into a request queue. It never touches robot state.
- **Apply:** the control loop takes the queued requests at the start of a tick and applies them. It is the only writer of controller state, config and motor outputs, so a tick never sees a half-applied change. Sampling happens in the same task, so sampled rows are never torn.
- **TX:** each port has one TX queue and one writer task. The loop, odometry and IMU only push into the queue (non-blocking; a full queue drops and counts). The writer sends only what `availableForWrite()` allows. The loop never writes to a UART. This avoids the class of stall measured on 2026-10-22, where a blocking 314-byte write gave 4.5–7.3 ms loop passes ([lab note](../../../lab/robot/2026-10-22-package1-motor-diagnostic-first-run.md)).

### The entry table

`signals.def` is the only place that names loggable or settable values. An X-macro expands it into the sampler's pointer table and the catalog.

```cpp
// signals.def
SIGNAL    (speed_target,      "m/s", balanceOutput.diag.shapedSpeedMps)
SIGNAL    (tilt_clipped,      "",    balanceOutput.diag.tiltClipped)
SIGNAL_MAX(loop_interval_max, "us",  loopEntryIntervalUs)
SIGNAL_MIN(battery_min,       "V",   batteryInstantRawV)

PARAM(speed_kp,   "rad/(m/s)", balanceSettings, speedKpRadPerMps)
PARAM(mahony_kp,  "",          imuSettings,     mahonyTwoKp)
PARAM(servo1_trim,"deg",       servoTrimSettings, servo1Degrees)

ACTION(motor_current, motorCurrentTest,
       ARG(left_a,   -MotorDiagnostic::kMaximumCurrentA, MotorDiagnostic::kMaximumCurrentA),
       ARG(right_a,  -MotorDiagnostic::kMaximumCurrentA, MotorDiagnostic::kMaximumCurrentA),
       ARG(lease_ms, 1, MotorDiagnostic::kMaximumLeaseMs))
ACTION(motor_reset, motorReset)
ACTION(calibrate,   startCalibration, ARG(step, 1, kLastCalibrationStep))
```

Some example names stand for state that does not exist yet: `loopEntryIntervalUs` and `batteryInstantRawV` are computed only inside `collectTelemetryInputs()` today and become state members in migration step 3; `kLastCalibrationStep` stands for the range of `calibrationSelect`.

| Kind | Meaning |
| --- | --- |
| `SIGNAL` | Read-only, streamable. The value at the sampled tick. |
| `SIGNAL_MIN`, `SIGNAL_MAX` | The minimum or maximum since the previous row of the same subscription; the sampler keeps the running value per subscription and resets it on each row. Covers short spikes that a sampled stream misses: loop-interval peaks, battery dips, servo range windows. |
| `PARAM` | A field of a settings group. Readable and streamable like a signal, and writable through the group (below). |
| `ACTION` | A call with bounded arguments. Bounds refer to the existing constants, so the table cannot drift from them. |

Rules:

- An entry points at state that already exists: a controller's output or state struct, a global, a settings group. There is no `TelemetryInputs` struct. A value that exists only as a local becomes a member of its controller's state or diagnostics struct; that member is real state, not debug code.
- No limit on the table size. An entry that is not subscribed costs a few bytes of flash and no run time.
- Duplicate names fail the build (`constexpr` check over the table).
- Values are 4 bytes on the wire, typed by the catalog: `f32`, `i32`, `u32`, `bool`. Integers are never sent as `float`, which would lose precision above 2^24 (a `micros()` value after about 17 s).

### Parameters: settings groups

A `PARAM` names a field of a settings group. A group is a struct plus one apply function:

| Group | Struct | Apply |
| --- | --- | --- |
| `balanceSettings` | `Config` | `configValid()` on the candidate; swap at the next tick start |
| `imuSettings` | `imu::Settings` | `imu::setSettings()`, which re-initialises the filters |
| `servoTrimSettings` | `ServoTrimsDegrees` | range check per trim |

`SetParam` copies the group's current struct, changes one field in the copy, and calls the apply function. On failure the robot replies with an error and the running values stay as they were. A per-field min/max alone cannot express cross-field rules such as `trimEnvelopeRad <= tiltRefLimitRad` (`BalanceController.cpp`, `configValid`); without this step one bad value would latch `InvalidConfig`, switch the wheels off and drop a balancing robot.

Not exposed as parameters: `Config` fields that copy motor constants SimpleFOC uses separately (`kvRating`, `phaseResistanceOhm`, voltage and current limits, wheel signs). Changing only the copy would make the effort envelope disagree with the drive.

`balanceConfig()` returns a `static const Config` today; it becomes the `balanceSettings` group.

### Actions and safety

- Safety stays in the firmware handlers, as today: `MotorDiagnostic::start` refuses while balance or calibration is active, and every motor test has a lease of at most `kMaximumLeaseMs`. A test whose lease is not renewed stops, so link loss is safe.
- Confirmation is a user-interface step: `navlink shell` shows the checked arguments and asks `y/N` before it sends an action. The firmware and protocol have no confirmation tokens.
- `motor_reset` replaces `T RESET`/`M RESET`, including the overspeed re-arm.
- A long action replies with `Ack` at once and reports the result as a `Log` event. Calibration blocks the loop for at least 3 s, and longer when the robot moves; streams pause meanwhile. It is refused while balancing.

### Subscriptions

A subscription is a list of entry IDs and a period in µs.

- Up to 4 subscriptions at the same time, on any port, also two on one port (for example a 10 Hz dashboard log and a full-rate debug capture over UART1).
- The firmware owns the slots. `Subscribe{slot: 0}` asks for a free one, and the `Ack` returns it in `value` (u32). A slot belongs to the port that subscribed it: only that port renews, replaces or unsubscribes it, and any other port gets `slot in use`. Clients on two ports therefore never pick the same slot.
- Up to 48 entries per subscription, so one row (196 bytes) fits in a frame.
- Rate: `period_us`, scheduled like odometry (`LinkScheduleDue()` in `link_schedule.h`). A row is taken on the first tick at or after the deadline. `period_us = 0` means every tick. The tick rate is not fixed: 424, then 521, then about 710 passes per second were measured as the code changed, so the period is set in time, not in ticks.
- Each row carries `t_us`, the `micros()` value at the start of its tick. Rows from different subscriptions taken at the same tick carry the same `t_us`, which aligns them without a tick counter. Rows are not evenly spaced; analysis uses `t_us`.
- Lease: a subscription expires after 5 s unless the client renews it. A crashed capture stops streaming by itself.
- Budget check: on `Subscribe` the firmware adds up all subscriptions on that port, `(4 + 4 × entries) × rate`, plus about 5 % frame overhead, plus 5 kB/s for odometry and IMU on UART1. It rejects a subscription that would take the port over 70 % of its line rate, with the numbers in the error: 140 kB/s of 200 kB/s on UART1 at 2000000 baud, 40 kB/s of 57.6 kB/s on USB at 576000 baud. The rest stays free for `Log` and `Ack`.

| Example | Load | UART1 (135 kB/s free after odometry) | USB (40 kB/s) |
| --- | --- | --- | --- |
| 40 entries at 10 Hz (logging) | 2 kB/s | fits | fits |
| 8 entries every tick, about 710 Hz | 27 kB/s | fits | fits |
| 20 entries every tick, about 710 Hz | 62 kB/s | fits | rejected; fits at a 3 ms period |
| 48 entries every tick, about 710 Hz | 146 kB/s | rejected; fits at a 2 ms period | rejected |

UART1 runs at 2000000 baud since 2026-10-27 ([protocol page](../software/ros2-uart-protocol.md)), so the USB port at 576000 baud is the tighter one. A 921600-baud CH340 capture on 2026-10-07 lost rows ([USB/serial facts](../../robot/software/usb-serial-flashing.md)), so the USB rate stays where it is.

### Catalog

The PC never hard-codes names or layouts. On connect it reads the catalog one entry at a time (`GetCatalog{index}` → `CatalogEntry`), stop-and-wait with retries, so a large table never overruns the 512-byte TX buffer. An entry carries index, name, unit, kind, type and, for actions, the argument names and bounds. The first reply also carries the entry count and the firmware build ID.

Wire IDs are table indices. They shift when a line is added or removed, so the PC resolves names against the catalog of the current connection and stores only names (CSV headers, presets). A reboot shows as the port's `seq` starting over; the PC then reads the catalog again and reports added and removed names.

### Logs and events

`Logging::message` sends a `Log` frame on every port, whatever subscriptions are active; today it is suppressed whenever a log profile is active (`Logging.cpp`). Every applied `SetParam` and every started action also goes out as a `Log` event on all ports, so a capture on UART1 records a change made over USB.

`Log` gains a `tag` field. Firmware text that today goes to `Serial` directly (setup messages, `IMU_TIMING`, SimpleFOC debug output, IMU prints) moves to `Logging::message`.

### USB backup port

- Same frames, same functions, 576000 baud.
- The writer sends a `0x00` before each frame, so stray text never merges with the next frame. The PC decoder shows any chunk that is not a valid frame and is printable as raw text. ROM boot output and panic dumps stay unframed and stay readable this way.
- Opening the CH340 can reset the board, even with DTR and RTS off before opening ([lab note](../../../lab/robot/2026-09-27-diagnostic-mode-history.md), [lab note](../../../lab/robot/2026-10-25-uart1-ros2-link.md)). Use USB only on a supported or idle robot unless the auto-reset circuit is changed.
- A port held open by `navlink` blocks flashing; close it first.

## Protocol additions

Additions to `navbot_link.proto`; field numbers are placeholders. nanopb needs a size for every repeated or bytes field.

```proto
message GetCatalog   { uint32 index = 1; }
message CatalogEntry { uint32 index = 1; uint32 count = 2; string name = 3; string unit = 4;
                       Kind kind = 5; Type type = 6; repeated Arg args = 7; string build = 8; }
message Subscribe    { uint32 slot = 1; repeated uint32 ids = 2; uint32 period_us = 3; } // slot 0 = allocate; renew = same slot
message Unsubscribe  { uint32 slot = 1; }
message SignalBatch  { uint32 slot = 1; bytes rows = 2; }  // rows: [t_us u32][value 4 B]... little-endian
message GetParam     { uint32 id = 1; }
message SetParam     { uint32 id = 1; bytes value = 2; }
message Call         { uint32 id = 1; repeated float args = 2; }
message Ack          { uint32 command_id = 1; bool accepted = 2; string error = 3; bytes value = 4; } // value: param value, or the subscribed slot
message Log          { Level level = 1; uint32 code = 2; string text = 3; string tag = 4; }
```

`WheelOdometry`, `Imu`, `Battery`, `Status` and `Command` stay unchanged: they are the ROS contract and are verified on the robot.

## PC side

- **`navbot_link` library:** `framing.py` and the decoder from `navbot_link_bridge`, moved into a module without a ROS dependency. `Link(port)` offers `catalog()`, `subscribe(names, rate_hz)`, `get(name)`, `set(name, value)`, `call(name, **args)` and a log stream; it renews leases itself.
- **`navlink shell`:** interactive, on `/dev/ttyAMA3` (stop the bridge first) or `/dev/ttyUSB0`. Commands: `list [pattern]`, `get`, `set`, `call` (with `y/N` prompt), `sub <names|@preset> @<Hz> -o file.csv`, `unsub`, `log`.
- **`navlink capture`:** `navlink capture --preset speed_loop --seconds 30 -o speed-01.csv`.
- **Presets:** a YAML file of named signal lists with a rate. They replace `K55`–`K58`. A preset that names a missing signal fails with `unknown signal <name>`.
- **CSV:** a header comment with firmware build and all parameter values at the start, a column header `t_us,<names>`, and a comment line for each parameter change or action seen as a `Log` event. The analysis scripts read the column header, as today.
- **ROS bridge:** `Log` → `/rosout`; `SignalBatch` → a `/navbot/signals` topic; parameters as ROS parameters. The dashboard tool is an open decision on the [roadmap](roadmap.md#decide-the-dashboard-for-logs-and-debug-signals).

## Example: tuning the speed loop

The speed loop in `BalanceController::speedLoopStep` runs every `speedLoopPeriodUs` (20 ms). Its values are already in `Diagnostics` except the measured speed, which `measuredSpeedMps()` computes but does not output; add it as `Diagnostics::measuredSpeedMps`.

```cpp
SIGNAL(speed_target,   "m/s", balanceOutput.diag.shapedSpeedMps)
SIGNAL(speed_measured, "m/s", balanceOutput.diag.measuredSpeedMps)
SIGNAL(speed_p,        "rad", balanceOutput.diag.speedP)
SIGNAL(speed_i,        "rad", balanceOutput.diag.speedI)
SIGNAL(tilt_ref,       "rad", balanceOutput.diag.tiltRefRad)
SIGNAL(tilt_clipped,   "",    balanceOutput.diag.tiltClipped)
PARAM (speed_kp,       "rad/(m/s)",   balanceSettings, speedKpRadPerMps)
PARAM (speed_ki,       "rad/(m/s*s)", balanceSettings, speedKiRadPerMpsS)
PARAM (speed_i_on,     "",            balanceSettings, speedIntegralEnabled)
```

```text
$ navlink shell --port /dev/ttyAMA3
navlink> get speed_kp speed_ki speed_i_on
navlink> sub speed_target speed_measured speed_p speed_i tilt_ref tilt_clipped @100 -o speed-01.csv
slot 1: 6 entries @ 100 Hz, 2.8 kB/s -> speed-01.csv
navlink> set speed_kp 0.12
speed_kp: 0.08 -> 0.12
navlink> unsub 1
```

100 Hz shows every 20 ms speed-loop step; a faster rate repeats the same values. After debugging, delete the `SIGNAL` lines that are no longer needed.

## Migration

Each step ends in a state that can be checked on the robot.

1. Delete the dead yaw path (`Y`, `yawPid`, `yawPidSettings`, `K9`, `K33`, CTRL yaw columns).
2. Firmware: request queue, TX queue and writer per port, per-port `seq`. Odometry and IMU move to the TX queue. Check: 50 Hz odometry unchanged, no loop stalls in the loop-interval maximum.
3. Entry table, catalog, subscriptions on both ports; `navbot_link` library and `navlink shell`/`capture`; presets that reproduce TRACE, CTRL, BAL and DRIVE. The `K` commands still work in parallel. Check: a preset capture against a `K58` capture of the same run.
4. Settings groups and actions replace `Q`, `I`, `Z`, `A`–`D`, `T`, `M`, `E`.
5. Delete `K`, the profiles, `LogSamples.h` payloads, `LogFormat.cpp`, the selector code in `DebugTelemetry.cpp` and `RobotLogCapture.cpp`, and Commander. Switch the analysis scripts and the trace-capture docs to the new CSV.
6. Sensor diagnostic mode: run the link endpoint and the sampler from `diagnosticLoop()`, or delete the mode. Today that build returns from `setup()` before `LinkUartInit()` and never runs the control tick.
7. ROS bridge: `/rosout`, `/navbot/signals`, parameters, then the dashboard.

## Limitations

- Calibration stalls the loop and all streams for at least 3 s.
- Rows are not evenly spaced, and a signal that changes every 20 ms aliases against a sampling period that is not a multiple of it. Use `t_us`.
- USB carries less than a third of UART1: about 10 entries at the full tick rate.
- ROM boot output and panic dumps are not framed.
- The firmware does not act on ROS `Command` yet; RC/ROS arbitration and a command timeout are not part of this proposal.
