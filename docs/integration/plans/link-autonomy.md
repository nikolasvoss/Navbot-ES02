# Plan: link features for autonomous development and debugging

Status: **proposal, not implemented.** Branch `feat/link-firmware`, worktree `/home/niko/Navbot-wt/fw`, base commit `d9802212`. When it is implemented, task T8 moves the content into the software pages and deletes this file.

Goal: an agent on the ros2_pc can flash, identify, reboot, observe, tune and (later) drive the robot over the link without stopping other programs, and the robot never starts or changes motion by accident.

## Decisions (user, 2026-10-30)

| # | Feature | Decision |
| --- | --- | --- |
| 1 | Balance state and fault reason readable | Yes |
| 2 | Build ID names the git revision | Yes, simple: a generated header, no extra tooling |
| 3 | Crash dump and reset reason over UART1 | Not now |
| 4 | Reboot over the link | Yes, an action, only with CH5 in position 0 and balance off |
| 5 | ROS drives the robot (`Command`, `/cmd_vel`) | Yes, implement now so it works later |
| 6 | Balance on/off from the link | Yes, but never by accident: only with CH5 in the middle position 1, the link mode (see [CH5 positions](#ch5-positions)) |
| 7 | Some parameters settable while balancing | Yes, an allowlist with a step limit per parameter |
| 8 | Save parameters on the robot | No: change the defaults in the firmware and reflash |
| 9 | Link counters | As signals in the table, not as a periodic log line |
| 10 | Status and battery in ROS | Yes, through a health subscription in the bridge; the unused `Status` and `Battery` messages are deleted |
| 11 | System health (heap, stack, uptime) | Yes, without CPU load |
| 12 | Bridge and navlink at the same time | Yes, a link multiplexer that owns the UART |
| 13 | navlink one-shot commands with exit codes and JSON | Yes |
| 14 | Fault-triggered recording (black box) | Not now; an optional item on the [roadmap](roadmap.md#optional-fault-triggered-recording-black-box) |
| 15 | Time sync | Not now |
| 16 | Log level at runtime | Yes, one global level |

## CH5 positions

CH5 is a 3-position switch (`kRcEnableOnRaw` = 592 today; the middle and the far end both mean "balance on"). The new meaning puts the link mode in the middle, so a switch moving from one end to the other always passes a position that starts nothing on its own:

| Position | Raw S.BUS value | Balance enable | Motion request | Link may |
| --- | --- | --- | --- | --- |
| 0 (end) | below 592 | off | none | `reboot`, every `set` |
| 1 (middle) | 592 to 1401 | the link flag `link_balance` | link `Command`, zero when older than 250 ms | `balance`, `Command`, live `set` |
| 2 (other end) | 1402 and above | on | RC sticks (CH3, CH4) | live `set` only |

- The RC stays the authority and the dead man: position 0 always switches balance off, an RC loss behaves as today (the request counts as lost), and CH6, CH7 and CH9 keep their RC functions in every position. RC balance now needs the far end (position 2), not the middle.
- Entering position 1: `link_balance` becomes true only when the controller is in `Run` or `Commissioning` at that tick. Leaving position 1 clears `link_balance` and the stored command.
- What the rule does on each move:
  - 2 → 1: a balancing robot is handed to the link and keeps balancing, with zero demand until a `Command` arrives.
  - 2 → 0 (passes 1): the controller runs when 1 is entered, so balance stays on for the few milliseconds of the transit and ends at 0.
  - 0 → 2 (passes 1): the controller is off when 1 is entered, so `link_balance` stays false and nothing starts; position 2 then gives the enable edge, which is today's RC start.
  - 0 → 1: nothing starts until the link calls `balance on=1`.
- A link `Command` outside position 1 is refused with `CH5 not in link position (1)`. A missing command stream does not drop the robot: the demand goes to zero and the controller keeps balancing in place.
- The value 1402 is the midpoint between 992 and 1811. Both thresholds must be checked on the robot (`ch5_position` while flipping the switch, [robot checks](#robot-checks)).

## Rules for every agent

1. Work in `/home/niko/Navbot-wt/fw` on branch `feat/link-firmware`. Do not switch branches, commit, push, flash, open `/dev/ttyAMA3` or `/dev/ttyUSB0`, or run `systemctl`. The orchestrator commits each task when it is done.
2. Before you start, read `AGENTS.md`, [signal-table.md](../software/signal-table.md), [ros2-uart-protocol.md](../software/ros2-uart-protocol.md), this file's sections up to [Shared names](#shared-names), and your task.
3. Change only the files your task lists. Other tasks run at the same time on other files. If you need a change outside your list, stop and report it.
4. No legacy: delete what a task replaces (named in the task). No compatibility shims, no fallbacks for old firmware.
5. Match the surrounding code: naming, comment density, test style (C++ host tests with a `run_host_test.sh` per folder, pytest with literal expected values).
6. Do not edit generated files by hand (`navbot_link.pb.*`, `navbot_link_pb2.py`); regenerate them (T0).
7. Run the tests your task names and report the exact pass/fail output. A test that fails because a later task has not run yet is listed with the reason.
8. Docs pages change only in T8. Keep code comments correct in your own files.
9. Several agents edit this worktree at the same time, some of them the same file (`OllieFOCdrive.ino`, `signals.def.h`, `link.py`, `tests/link/test_signals_def.cpp`). Change an existing file only with Edit, never by writing the whole file; when an Edit fails because the file changed, read it again and retry. Do not run `git checkout`, `git stash`, `git reset` or a formatter over whole files. A test that fails in a file another running task owns: run it again later, then report it.
10. The sketch (`OllieFOCdrive.ino`) cannot be compiled on the ros2_pc. Put logic in headers with host tests where you can; the orchestrator compiles the sketch remotely and sends compile errors back.

## Shared names

Every task uses exactly these names.

### Protocol (`src/navbot_link/navbot_link.proto`)

- `CatalogEntry.live_step` (`float`, field 9): PARAM only. The largest change one `SetParam` may make while balance is requested. `0` = refused while balancing, `inf` = any change.
- `Command { float speed_mps = 2; float turn_rad_s = 3; }` with `reserved 1;`. The `Ack` of a `Command` echoes `Message.seq` like every other request. `turn_rad_s` is positive to the left (counter-clockwise, REP 103).
- Delete the messages `Battery` and `Status` and their `oneof` fields; add `reserved 11, 12;` to `Message`.

### Constants

| Name | Value | Where |
| --- | --- | --- |
| `kLinkMaxSlots` | 8 (was 4) | `link_subscriptions.h` |
| `RcMode::kCh5LinkRaw` | 592 (replaces `kRcEnableOnRaw`): position 1 at or above | `rc_mode.h` (new) |
| `RcMode::kCh5RcRaw` | 1402: position 2 at or above | `rc_mode.h` |
| `RcMode::kLinkCommandTimeoutUs` | 250000 | `rc_mode.h` |
| `kRebootDelayMs` | 300 | `OllieFOCdrive.ino` |
| Multiplexer address | `127.0.0.1:7650`, port string `tcp:127.0.0.1:7650` | `navbot_link` |

### New and renamed entries in `signals.def.h`

Signals (type follows the expression):

| Name | Unit | Expression | Task |
| --- | --- | --- | --- |
| `ch5_position` (replaces `ch5_mode`) | | `ch5Position` (`uint8_t`, 0, 1 or 2) | T6 |
| `rc_valid` | | `rcValid` (bool: RC frame fresh and no failsafe) | T6 |
| `balance_state` | | `balanceOutput.state` | T4 |
| `balance_fault` | | `balanceOutput.fault` | T4 |
| `overspeed_tripped` | | `wheelOverspeed.tripped` | T4 |
| `motor_test_active` | | `motorDiagnosticState.active` | T4 |
| `link_tx_drop_uart` | | `LinkCountersNow().txDropUart` | T4 |
| `link_tx_drop_usb` | | `LinkCountersNow().txDropUsb` | T4 |
| `link_rx_bad_uart` | | `LinkCountersNow().rxBadUart` | T4 |
| `link_rx_bad_usb` | | `LinkCountersNow().rxBadUsb` | T4 |
| `link_req_dropped` | | `LinkCountersNow().reqDropped` | T4 |
| `link_undecodable` | | `LinkCountersNow().undecodable` | T4 |
| `uptime_s` | s | `systemHealth.uptimeS` | T4 |
| `heap_free_b` | B | `systemHealth.heapFreeB` | T4 |
| `heap_min_free_b` | B | `systemHealth.heapMinFreeB` | T4 |
| `stack_free_loop_b` | B | `systemHealth.stackFreeLoopB` | T4 |
| `stack_free_link_b` | B | `systemHealth.stackFreeLinkB` | T4 |
| `link_balance` | | `linkBalance` | T6 |
| `cmd_speed_mps` | m/s | `linkCommand.speedMps` | T6 |
| `cmd_turn_rad_s` | rad/s | `linkCommand.turnRadPerSec` | T6 |
| `cmd_age_ms` | ms | `linkCommandAgeMs()` (`UINT32_MAX` before the first command) | T6 |

`balance_state` values: 0 Disabled, 1 Arming, 2 Run, 3 Commissioning, 4 Fault. `balance_fault` values: 0 None, 1 StaleImu, 2 StaleWheels, 3 StaleBus, 4 InvalidInput, 5 BadDt, 6 Fall, 7 Overspeed, 8 InvalidEnvelope, 9 InvalidConfig, 10 AuthorityExhausted, 11 TiltSaturated (the order of the enums in `BalanceController.h`).

Parameters settable while balancing (`PARAM_LIVE`, T4). The step is about a quarter of the current default; the user confirms the values during the [robot checks](#robot-checks):

| Parameter | Default | `live_step` |
| --- | --- | --- |
| `speed_kp` | 0.25 | 0.05 |
| `speed_ki` | 0.08 | 0.02 |
| `tilt_p` | 8.0 | 1.0 |
| `gyro_damping` | 0.4 | 0.1 |
| `yaw_p` | 0.6 | 0.15 |
| `speed_cap` | 0.3 | 0.1 |
| `yaw_rate_cap` | 1.0 | 0.25 |
| `log_level` (new) | 2 | `INFINITY` |

All other parameters stay `PARAM` and are refused while balancing. `configValid()` still checks every live change.

`log_level` (T4): `u32`, group `LogGroup`, field `minLevel`. Values are the proto levels: 1 DEBUG, 2 INFO, 3 WARN, 4 ERROR. Default 2. Other values are refused with `takes 1 (DEBUG) to 4 (ERROR)`. It filters `Logging::message` only; the `param`, `action` and `link` Log lines of the endpoint always go out.

Actions:

| Name | Arguments | Refused when (exact text) | Task |
| --- | --- | --- | --- |
| `reboot` | none | balance requested: `CH5 must be 0 and balance off`; motor test active or applied: `motor test running`; calibration selected: `calibration running` | T6 |
| `balance` | `ARG(on, 0, 1)` | diagnostic build: `sensor diagnostic build has no motor control`; CH5 not 1: `CH5 not in link position (1)`; overspeed latched: `wheel overspeed latched, send motor_reset` | T6 |

## Schedule

Tasks start as soon as what they need is committed, not in fixed waves. T0 fixes the interfaces (proto, table macro, command handler hook) the other tasks build on, so six tasks can run at the same time after it.

| Task | Starts when | Model |
| --- | --- | --- |
| T0 interfaces | now | Haiku, effort medium |
| T2 build ID | now | Haiku, effort medium |
| T3 multiplexer | now | Haiku, effort medium |
| T1 endpoint logic | T0 committed | Haiku, effort medium |
| T4 firmware observability | T0 committed | Haiku, effort medium |
| T5 Python library and navlink CLI | T0 committed | Haiku, effort medium |
| T6 firmware control | T0 committed | Haiku, effort medium |
| T7 bridge | T5 committed | Haiku, effort medium |
| Review of one task | that task is committed | Sonnet |
| Remote build, flash, first robot checks | T1, T2, T4 and T6 committed | orchestrator |
| T8 docs | every code task committed | Haiku, effort medium |
| T9 final cross-check | every code task committed, in parallel with T8 | Sonnet |
| Robot checks with the user | T7 committed and the firmware flashed | orchestrator and user |

Critical path: T0 → T4 and T6 → remote build → fixes → T8 and T9.

Orchestrator:

1. When a task reports done, run its tests and the full suite below, and commit its files. T4 and T6 share files, so they are committed together. Then start a Sonnet review of that commit against this plan (findings with file and line); accepted findings go to a Haiku fix agent at once, while other tasks keep running.
2. When T1, T2, T4 and T6 are committed: `git push` and `scripts/build_remote.sh`. Compile errors go to a Haiku fix agent with the compiler output. Then flash and run robot checks 1, 3, 4 and 6 while T7, T8 and T9 run.
3. Never run two agents whose file lists overlap, except T4 with T6 and T3 with T5 (rule 9).

Test commands:

```bash
for s in tests/*/run_host_test.sh; do bash "$s" || echo "FAIL $s"; done
python3 -m pytest -q tests/navbot_link tests/scripts
PYTHONPATH=src:src/ros2_pc/ros2/src/navbot_link_bridge python3 -m pytest -q src/ros2_pc/ros2/src/navbot_link_bridge/test
python3 tests/layout/test_repo_layout.py
```

Prompt for each agent: "Read `docs/integration/plans/link-autonomy.md` (rules, shared names, schedule and task T<n>) and do only task T<n>."

## T0: interfaces (starts first)

Files: `src/navbot_link/navbot_link.proto`, `src/navbot_link/navbot_link.options` (only if a deleted message is named there), the regenerated `src/ES-02/OllieFOCdrive/src/navbot_link/navbot_link.pb.c` and `.pb.h` and `src/navbot_link/navbot_link_pb2.py`, `src/ES-02/OllieFOCdrive/signal_table.h`, `signal_table_expand.h`, `link_endpoint.h`, `link_endpoint.cpp` (only the handler storage), `link_subscriptions.h`, `link_uart.h` and `link_uart.cpp` (only `LinkSetCommandHandler`), `tests/link/*` (only what the slot count breaks).

Four tasks wait for this one: keep it to the interfaces, no new behaviour.

1. Generator: `python3 -m venv work/nanopb-venv && work/nanopb-venv/bin/pip install nanopb==0.4.9.1`, then `PATH="$PWD/work/nanopb-venv/bin:$PATH" bash scripts/generate_link_proto.sh`. `protoc` 3.21.12 is `/usr/bin/protoc`. If the generator fails, stop and report; do not edit generated files.
2. Proto: the changes under [Protocol](#protocol-srcnavbot_linknavbot_linkproto). Update the comments of `Ack.command_id` ("echoes Message.seq of the request") and `Command`. Regenerate.
3. `signal_table.h`: add `float liveStep;` as the last member of `SignalEntry`, with the comment from the shared names. `SIGNAL_TABLE_ROW`, `ACTION` and `ACTION_NO_ARGS` set it to `0.0f`. Add `constexpr bool SignalLiveStepsValid(const SignalEntry*, size_t)`: `liveStep` is 0 for every non-PARAM entry and `>= 0` (not NaN) for PARAM.
4. `signal_table_expand.h`: one shared row macro for `PARAM(name, unit, group, field)` (step 0) and the new `PARAM_LIVE(name, unit, group, field, step)`; `#undef PARAM_LIVE`; `static_assert(SignalLiveStepsValid(...), "a live step is negative or set on an entry that is not a parameter")`. Describe `PARAM_LIVE` in the comments of `signal_table_expand.h`; the header comment of `signals.def.h` belongs to T4.
5. Handler hook: `typedef const char* (*LinkCommandHandler)(float speedMps, float turnRadPerSec);` in `link_endpoint.h` and `LinkEndpoint::SetCommandHandler(LinkCommandHandler)`, which only stores it (T1 uses it). `link_uart.h`/`.cpp`: `void LinkSetCommandHandler(LinkCommandHandler handler)` forwards to the endpoint instance.
6. `link_subscriptions.h`: `kLinkMaxSlots = 8`. Fix the existing tests that assume 4.

Tests: `bash tests/link/run_host_test.sh`, `python3 -m pytest -q tests/navbot_link tests/scripts`. Everything that passed before passes.

## T1: endpoint logic for live steps and Command (after T0)

Files: `src/ES-02/OllieFOCdrive/link_endpoint.cpp`, `tests/link/*` (in `test_signals_def.cpp` only the printed columns), `tests/navbot_link/endpoint_server.cpp`, `scripts/signals_catalog.py`, `tests/scripts/test_signals_catalog.py`.

1. SetParam: replace the blanket refusal. While `balanceRequested`: `liveStep == 0` → the current text `<name>: refused while balancing`; otherwise decode the candidate with the entry's type, compare `|new - old|` as `double`, and refuse a larger change with `<name>: step <d> over <step> while balancing` (`%g` for both). Bool and integer parameters use the same rule. Everything else (size check, apply, Log) stays.
2. Catalog: `live_step = entry.liveStep` for PARAM entries.
3. Command: the `command` case calls the stored handler and acks with its reason (`nullptr` = accepted). Without a handler the Ack says `this build takes no commands`. Delete the comment "Command and anything unknown: the firmware does not act on them yet".
4. Tests in `tests/link/`: add a `PARAM_LIVE` entry to `fixture_signals.def.h`; in `test_link_endpoint.cpp` check with literal bytes and texts: a live set inside the step while balancing is accepted, one over the step is refused with the exact text, a plain PARAM is refused while balancing, the catalog entry carries `live_step`, a `Command` without handler is refused, with a test handler it is accepted and refused with the handler's text, the 8th slot is accepted and the 9th refused. Add `reject_live_step.def.h` (a negative step) to the build-must-fail list in `run_host_test.sh`.
5. `tests/navbot_link/endpoint_server.cpp`: register a handler that accepts finite values and refuses others with `speed and turn must be finite numbers`.
6. Live step in the catalog comparison: `scripts/signals_catalog.py` parses `PARAM_LIVE` (kind PARAM, `live_step` as a float; `INFINITY` → `float("inf")`) and prints the live step as a fifth column; `tests/link/test_signals_def.cpp` prints the same column and `run_host_test.sh` compares five columns. Update `tests/scripts/test_signals_catalog.py`.

Tests: `bash tests/link/run_host_test.sh`, `python3 -m pytest -q tests/navbot_link tests/scripts/test_signals_catalog.py`.

## T2: build ID with the git revision (starts first)

Files: `scripts/build_firmware.py`, `.gitignore`, `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino` (only the `kBuildId` block, lines 576 to 578), `tests/scripts/test_build_firmware.py` (new).

1. `build_firmware.py`: a pure function `build_id(revision: str | None, dirty: bool, now: datetime) -> str` returning `<8-char revision>[-dirty] <YYYY-MM-DD HH:MM>`, or `nogit <YYYY-MM-DD HH:MM>` without a revision; at most 32 characters (raise `ValueError` otherwise). Before the compile, read the revision with `git -C <sketch> rev-parse --short=8 HEAD`, dirty = `git -C <sketch> status --porcelain --untracked-files=no` is not empty, and write `<sketch>/build_id.h`:

   ```cpp
   // Written by scripts/build_firmware.py before each build. Not in git.
   #define NAVBOT_BUILD_ID "d9802212-dirty 2026-10-30 14:05"
   ```

2. `.gitignore`: `src/ES-02/OllieFOCdrive/build_id.h`.
3. `OllieFOCdrive.ino`: replace the comment and `constexpr char kBuildId[] = __DATE__ " " __TIME__;` with

   ```cpp
   // Git revision and build time, written by scripts/build_firmware.py. A build without the script says "unknown".
   #if __has_include("build_id.h")
   #include "build_id.h"
   #else
   #define NAVBOT_BUILD_ID "unknown " __DATE__
   #endif
   constexpr char kBuildId[] = NAVBOT_BUILD_ID;
   ```

   Keep the `static_assert`.
4. `scripts/build_remote.sh` needs no change (it runs `build_firmware.py` on the desktop from a clean, pushed checkout). Check that and say so.
5. Test `build_id()` with literal values, including the 32-character limit, and that the header text is written (use a temporary sketch folder and a stubbed git call).

Tests: `python3 -m pytest -q tests/scripts/test_build_firmware.py tests/scripts`.

## T3: link multiplexer and TCP transport (starts first)

Files: `src/navbot_link/mux.py` (new), `src/navbot_link/transport.py` (new), `src/navbot_link/link.py` (only: delete `SerialTransport`, import from `transport`, and the line in `Link.__init__` that picks a transport), `tests/navbot_link/test_mux.py` (new), `src/ros2_pc/systemd/navbot-link-mux.service.in` (new), `src/ros2_pc/systemd/navbot-link-bridge.service.in`, `scripts/install_link_bridge_service.py`, `tests/scripts/test_install_link_bridge_service.py`.

Why: only one program can open `/dev/ttyAMA3`. The multiplexer owns it and lets any number of local clients (the bridge, `navlink`, scripts) share it. Clients speak the unchanged frame format over TCP, so `Link` works through it unchanged.

1. `transport.py`: move `SerialTransport` here from `link.py` unchanged; add `SocketTransport(host, port)` with `read()` (returns the bytes available, `b""` after a 50 ms timeout, raises `OSError` on a closed connection, like `SerialTransport` on a port error), `write()` (`sendall`) and `close()`. `open_transport(port: str, baud: int)`: `tcp:HOST:PORT` → `SocketTransport`, anything else → `SerialTransport`. In `link.py`, delete `SerialTransport`, import it from `transport`, and let `Link.__init__` use `open_transport` for a `str`.
2. `mux.py`, run as `python3 -m navbot_link.mux --port /dev/ttyAMA3 --baud 2000000 --listen 127.0.0.1:7650`. One thread, one `selectors` loop over the serial port (`serial.Serial(port, baud, timeout=0)`, DTR and RTS off as in `SerialTransport`) and the sockets, so the routing state needs no lock. It decodes both directions with `FrameDecoder` and re-encodes with `encode_frame`.
3. Client → robot: for each decoded `Message`, remember `(client, client_seq = msg.seq)`, set `msg.seq` to the next value of one multiplexer counter, and write `b"\x00" + frame` to the serial port. Record a pending entry `mux_seq → (client, client_seq, body name)`; drop entries older than 5 s. For `get_catalog`, also record `index → clients waiting`. Slot ownership: a `subscribe` with a nonzero slot, or an `unsubscribe`, naming a slot that another client owns is answered by the multiplexer itself with `Ack(command_id=client_seq, accepted=False, error="slot in use")` and not forwarded.
4. Robot → clients:
   - `ack`: look up `command_id`; rewrite it to `client_seq` and send it only to that client. An accepted `subscribe` Ack records `slot (from value) → client`; an accepted `unsubscribe` deletes it. An Ack without a pending entry is dropped.
   - `catalog_entry`: to the clients waiting for that index, then clear the index.
   - `signal_batch`: to the owner of `slot` only.
   - Everything else (`log`, `odom`, `imu`, unknown bodies): to every client.
   - A text chunk (`Text` from `FrameDecoder`): its bytes plus `b"\x00"` to every client.
5. Sequence numbers to clients: each client has its own counter, so frames routed to other clients do not look like losses. A frame sent to a client gets that client's next `seq`. A real gap on the robot side (seen with `SeqTracker`) of `n` frames adds `n` to every client's counter. A robot restart (non-increasing robot `seq`) clears the slot table and restarts every client's counter at 1, so each client sees the restart. `t_ms` is forwarded unchanged.
6. Client disconnect: send `Unsubscribe` for each slot it owns (pending entry without a client, its Ack dropped), forget its pending entries and catalog waits.
7. Serial error: log it to stderr and exit with status 1 (systemd restarts the service).
8. `navbot-link-mux.service.in`: `ExecStart=/usr/bin/python3 -m navbot_link.mux --port @PORT@ --baud @BAUD@ --listen 127.0.0.1:7650`, `Environment=PYTHONPATH=@REPO@/src`, `User=@USER@`, `Restart=always`, `RestartSec=1`. Use the placeholder style of the existing template.
9. `navbot-link-bridge.service.in`: `Requires=` and `After=navbot-link-mux.service`; the bridge `port` becomes `tcp:127.0.0.1:7650`. `install_link_bridge_service.py` installs and restarts both units (mux first) and keeps `--port` for the mux's serial port. Update its test.
10. `tests/navbot_link/test_mux.py`: run the multiplexer loop in a thread against the fake firmware over a pseudo-terminal (`PtyFirmware` in `tests/navbot_link/fake_firmware.py`, read-only for you) and two `Link` clients over TCP on a free port. Check: both read the catalog; each subscribes and gets only its own rows; `get`/`set` replies reach the asking client only; a `Log` reaches both; client B cannot unsubscribe client A's slot (`slot in use`); closing client A frees its slot on the robot; a robot restart gives both clients `Restarted`; neither client reports `FramesLost` in normal operation.

Tests: `python3 -m pytest -q tests/navbot_link tests/scripts/test_install_link_bridge_service.py`.

## T4: firmware observability (after T0)

Files: `src/ES-02/OllieFOCdrive/signals.def.h`, `OllieFOCdrive.ino`, `BalanceController.h`, `BalanceController.cpp`, `Logging.h`, `Logging.cpp`, `link_uart.h`, `link_uart.cpp`, `tests/balance_controller/test_balance_controller.cpp`, `tests/link/test_signals_def.cpp` (only stand-ins for T4's entries). T6 edits `OllieFOCdrive.ino` and `signals.def.h` at the same time (rule 9); the RC and CH5 code is T6's.

1. Names: `const char* StateName(State)` and `const char* FaultReasonName(FaultReason)` in `BalanceController.h`/`.cpp`, returning the enum names (`"Run"`, `"Fall"`, ...). Test every value. In `logBalanceChange()` replace the `state=%d fault=%d` line with `"%s -> %s (%s)"` (old state, new state, fault name); level `Warning` when the new state is `Fault`, otherwise `Info`. Keep the other three lines.
2. Link counters: `struct LinkCounters { uint32_t txDropUart, txDropUsb, rxBadUart, rxBadUsb, reqDropped, undecodable; };` and `LinkCounters LinkCountersNow();` in `link_uart.h`. Delete the `Link seq=... dropped=...` log line, `kLinkStatsEverySends` and its counter in `LinkPublishOdometry`.
3. Stack: store the reader task handles (pass a handle pointer to `xTaskCreatePinnedToCore` instead of `nullptr`), and add `uint32_t LinkMinStackFreeBytes()`: the minimum of `uxTaskGetStackHighWaterMark()` over the two reader and two writer tasks (bytes on ESP-IDF).
4. System health in the sketch: `struct SystemHealth { uint32_t uptimeS, heapFreeB, heapMinFreeB, stackFreeLoopB, stackFreeLinkB; } systemHealth;` and `updateSystemHealth()`, which refreshes it once per second (`millis()`): `millis() / 1000`, `esp_get_free_heap_size()`, `esp_get_minimum_free_heap_size()`, `uxTaskGetStackHighWaterMark(nullptr)` (it runs in the loop task) and `LinkMinStackFreeBytes()`. Call it from `runControlTick()` and `diagnosticLoop()`.
5. Log level: in `Logging`, a minimum level (default `Info`) with `void setMinLevel(Level)` and `Level minLevel()`; `message()` returns before formatting when the level is below it. In the sketch, `struct LogSettings { uint32_t minLevel; };` and `LogGroup` (current/apply like the other groups) converting between the proto numbers 1 to 4 and `Logging::Level`.
6. `signals.def.h`: the T4 rows of [Shared names](#shared-names) in a new "Balance state" section and a new "Link and system health" section, both after "Control loop timing"; `PARAM` → `PARAM_LIVE` with the steps from the table; `PARAM_LIVE(log_level, "", LogGroup, minLevel, INFINITY)` after the servo trims. Extend the header comment with `PARAM_LIVE`. Add stand-ins for every new expression and for `LogGroup` to `tests/link/test_signals_def.cpp`.

Tests: every `tests/*/run_host_test.sh`, `python3 -m pytest -q tests/navbot_link tests/scripts/test_signals_catalog.py`. `tests/scripts/test_navlink_presets.py` passes only once T5 and T6 are both in; report it.

## T5: Python library, navlink CLI, presets (after T0)

Files: `src/navbot_link/protocol.py`, `src/navbot_link/link.py` (everything except the transport lines T3 changes), `src/navbot_link/capture.py`, `src/navbot_link/health.py` (new), `scripts/navlink.py`, `scripts/navlink_presets.yaml`, `scripts/summarize_balance_trace.py`, `scripts/analyze_drive_trace.py`, `scripts/remote_status.py`, `tests/navbot_link/fake_firmware.py`, `tests/navbot_link/test_link.py`, `tests/scripts/*` and its fixtures.

1. `protocol.Entry` gets `live_step: float` from the catalog entry.
2. `Link.command(speed_mps, turn_rad_s) -> None`: sends a `Command` and returns at once (no retry, no wait). It remembers the request seq (at most the 64 newest); an Ack for one of them that is refused puts `CommandRefused(reason)` on `events`; an accepted one is dropped silently.
3. `health.py`: `HEALTH_SIGNALS` (tuple of names): `balance_state`, `balance_fault`, `overspeed_tripped`, `motor_test_active`, `rc_valid`, `ch5_position`, `voltage_filtered_v`, `voltage_min_raw_v`, `loop_interval_max_us`, the six `link_*` counters, `uptime_s`, `heap_free_b`, `heap_min_free_b`, `stack_free_loop_b`, `stack_free_link_b`. `BALANCE_STATES` and `FAULT_REASONS` (tuples of names in enum order from [Shared names](#shared-names)).
4. Presets: `ch5_mode` → `ch5_position` everywhere; new preset `health` at 1 Hz with exactly `HEALTH_SIGNALS` (a test checks they are equal).
5. Analysis scripts and fixtures: `ch5_mode` → `ch5_position`; "balance switch on" is a value of 1 or more. Rename the column in the fixtures; keep the values.
6. `navlink.py`:
   - `--port` defaults to `tcp:127.0.0.1:7650` (the multiplexer). `--port /dev/ttyUSB0` still opens the USB port directly. Delete the "stop the bridge first" help text.
   - New one-shot subcommands next to `shell` and `capture`: `info`, `list [pattern]`, `get NAME...`, `set PARAM VALUE`, `call ACTION [arg=value...] --yes`, `log --seconds S [--min-level DEBUG|INFO|WARN|ERROR]`.
   - `--json` on every one-shot command, output exactly:
     - `info`: `{"build": "...", "entries": 70, "port": "tcp:127.0.0.1:7650"}`
     - `list`: `[{"name": ..., "kind": "SIGNAL", "type": "F32", "unit": ..., "live_step": 0.05, "args": [{"name": ..., "min": ..., "max": ...}]}]` (`live_step` only for PARAM, `args` only for ACTION; infinity as the string `"inf"`)
     - `get`: `{"roll_deg": -0.4, "ch5_position": 0}`
     - `set`: `{"name": "speed_kp", "old": 0.25, "new": 0.3}`
     - `call`: `{"name": "reboot", "args": {}, "accepted": true}`
     - `log`: one JSON object per line, `{"t_ms": ..., "level": "INFO", "tag": ..., "text": ...}`
   - Without `--json`, the same text the shell prints.
   - `call` without `--yes` exits with 2 and `call needs --yes`.
   - `--wait SECONDS` for every one-shot command and `capture`: retry opening the link and reading the catalog until it works or the time is up (after a reboot or a flash).
   - Exit codes: 0 done; 1 the robot refused (`LinkError` with a robot reason); 2 usage error or unknown name; 3 no connection (`NoReply`, open failure, `Disconnected`).
   - `list` in the shell shows `live ±<step>` or `live` (infinite step) for live parameters.
7. `fake_firmware.py`: `FakeEntry.live_step`; a `balancing` flag that makes `SetParam` follow the live-step rule with the firmware's texts; a `link_position` flag: `Command` is accepted only when it is set, otherwise refused with `CH5 not in link position (1)`; 8 slots.
8. Tests: the new subcommands with literal outputs and exit codes, `--json`, `--wait` against a fake that answers late, `Link.command` and `CommandRefused`, the preset rename.

Tests: `python3 -m pytest -q tests/navbot_link tests/scripts`.

## T6: firmware control (after T0)

Files: `src/ES-02/OllieFOCdrive/signals.def.h`, `OllieFOCdrive.ino`, `OllieFOCdrive.h`, `rc_mode.h` (new), `tests/rc_mode/` (new: `run_host_test.sh`, `test_rc_mode.cpp`), `tests/link/test_signals_def.cpp` (only stand-ins for T6's entries). T4 edits `OllieFOCdrive.ino` and `signals.def.h` at the same time (rule 9).

1. `rc_mode.h` (pure C++, no Arduino, namespace `RcMode`): `kCh5LinkRaw`, `kCh5RcRaw`, `kLinkCommandTimeoutUs` and these functions, all tested in `tests/rc_mode/` in the style of `tests/overspeed_guard/`:
   - `uint8_t Ch5Position(int16_t raw)`: 0 below 592, 1 below 1402, else 2 (test 591/592, 1401/1402, 172, 992 and 1811).
   - `bool EnableSwitch(uint8_t position, bool linkBalance)`: `position == 2 || (position == 1 && linkBalance)`.
   - `bool LinkBalanceOnEntry(bool controllerDriving)`: returns `controllerDriving`.
   - `bool CommandFresh(bool seen, uint32_t nowUs, uint32_t receivedUs)`: `seen` and at most `kLinkCommandTimeoutUs` old (wrap-safe).
2. Replace `kRcEnableOnRaw`, `pidGainsMode` and the `REMOTE_CONTROL_PID_GAINS_MODE_*` defines in `OllieFOCdrive.h` with `uint8_t ch5Position`, set in `applyRcChannels()` from `RcMode::Ch5Position(rcChannels[4])`. `balanceRequested()` is `ch5Position != 0 || balanceActive()`. The overspeed re-arm in `balanceRequest()` uses `ch5Position == 0`. `balanceRequest()` stores `rcValid = request.sourceValid`.
3. Sketch state: `bool linkBalance`, `struct LinkCommand { bool seen; float speedMps; float turnRadPerSec; uint32_t receivedUs; } linkCommand;`, `uint8_t previousCh5Position`, `uint32_t linkCommandAgeMs()`.
4. `balanceRequest()`, only while the RC frame is fresh: on a change into position 1, `linkBalance = LinkBalanceOnEntry(state is Run or Commissioning)`; on a change out of position 1, `linkBalance = false` and `linkCommand = {}`. Then `request.enableSwitch = RcMode::EnableSwitch(ch5Position, linkBalance) && !wheelOverspeed.tripped`. In position 1, `speedMps` and `yawRateRadPerSec` come from `linkCommand` when `CommandFresh`, else 0; the controller's `shapeDemand` caps and ramps them, so the sketch does not clamp. In positions 0 and 2 the sticks stay as they are. Test the four moves of [CH5 positions](#ch5-positions) through the pure functions in `tests/rc_mode/`. `sourceValid` and `receivedTimeUs` stay the RC values: the RC is the dead man in every position.
5. `const char* handleLinkCommand(float speedMps, float turnRadPerSec)`: refusals in this order: `sensor diagnostic build has no motor control` (diagnostic build), `CH5 not in link position (1)`, `speed and turn must be finite numbers`. Accepted: store with `micros()`. Register it with `LinkSetCommandHandler()` in `setup()`.
6. `bool setLinkBalance(const float* args, const char** error)` and `ACTION(balance, setLinkBalance, ARG(on, 0, 1))`, refusals as in [Shared names](#shared-names); `linkBalance = args[0] >= 0.5f`.
7. `bool requestReboot(const float*, const char** error)` and `ACTION_NO_ARGS(reboot, requestReboot)`, refusals as in [Shared names](#shared-names). Accepted: `rebootPending = true; rebootAtMs = millis() + kRebootDelayMs`, so the Ack and the action Log leave first. At the top of `loop()`, before the diagnostic branch: when pending and due, `ESP.restart()`. It works in both images.
8. `signals.def.h`: `ch5_mode` → `ch5_position` and `rc_valid` in the receiver section; the other T6 rows in a new "Link control" section right after the receiver section; the two actions after `calibrate`. Add stand-ins for every new expression and handler to `tests/link/test_signals_def.cpp`.

Tests: every `tests/*/run_host_test.sh`, `python3 -m pytest -q tests/navbot_link tests/scripts`.

## T7: bridge (after T5)

Files: `src/ros2_pc/ros2/src/navbot_link_bridge/` (`navbot_link_bridge/*.py`, `package.xml`, `setup.py`, `test/*`).

1. Parameter `port` defaults to `tcp:127.0.0.1:7650`.
2. `/cmd_vel` (`geometry_msgs/msg/Twist`): each message → `link.command(linear.x, angular.z)` while a link is attached; no repetition, the firmware's 250 ms timeout handles a silent publisher. `CommandRefused` events → a warning at most every 5 s with the reason.
3. Health: after the catalog is read, subscribe `HEALTH_SIGNALS` (from `navbot_link.health`) at 1 Hz, as a second subscription next to the `signals` parameter one. On each row publish:
   - `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`) with one `DiagnosticStatus`: name `navbot_firmware`, `hardware_id` = build ID, level `ERROR` when `balance_fault != 0` or `overspeed_tripped`, `WARN` when a `link_*` counter rose since the previous row, else `OK`; message = state name, plus `: <fault name>` with a fault; one `KeyValue` per signal (state and fault as names).
   - `/battery_state` (`sensor_msgs/msg/BatteryState`): `voltage` = `voltage_filtered_v`, `present` true, `percentage` NaN, `power_supply_status` UNKNOWN, stamp = arrival time.
   - A health name the firmware does not have: one error log, no health topics, the bridge keeps running.
4. Delete any code that decodes `Battery` or `Status`.
5. `package.xml`: `geometry_msgs`, `diagnostic_msgs`, `sensor_msgs` as exec dependencies.
6. Tests with the existing fakes: `/cmd_vel` produces the `Command` bytes, a refusal produces one warning, a health row produces the expected `DiagnosticArray` levels and `BatteryState` voltage.

Tests: `PYTHONPATH=src:src/ros2_pc/ros2/src/navbot_link_bridge python3 -m pytest -q src/ros2_pc/ros2/src/navbot_link_bridge/test`.

## T8: docs (after every code task)

Files: `docs/integration/software/navlink.md`, `navlink-quick-start.md`, `ros2-uart-protocol.md`, `signal-table.md`, `navbot-link-bridge.md`, `docs/robot/software/trace-capture.md`, `docs/robot/software/sensor-diagnostic-mode.md`, `docs/integration/plans/roadmap.md`, `docs/integration/README.md`, this file.

- Describe the implemented state only: the CH5 positions, live parameters and steps, `reboot`, `balance`, `Command` and its timeout, the new signals with the enum tables, the multiplexer (no more "stop the bridge first"), the one-shot CLI with JSON and exit codes, the build ID, `log_level`, `/cmd_vel`, `/diagnostics`, `/battery_state`, 8 slots, and the deleted `Status`/`Battery` messages and stats log line.
- Roadmap: add the robot checks that are still open; mark the items already verified on 2026-10-28 (catalog, parameter get/set, diagnostic image, bridge `/rosout` and `/navbot/signals`) as done.
- Delete this file and its line in `docs/integration/README.md`.
- Run `python3 tests/layout/test_repo_layout.py`.

## T9: final cross-check (Sonnet, in parallel with T8)

Each task already had its own review. Review `git diff d9802212..HEAD` as a whole against this plan, above all that names and refusal texts agree between firmware, fake firmware, library, CLI and bridge. Report findings with file and line, most severe first. Check above all: no path lets the link start or change motion outside CH5 position 1; live steps are enforced for every type; `reboot` is refused in positions 1 and 2; the moves 0 → 2 and 2 → 0 through position 1 and 2 → 1 while balancing; the multiplexer's seq renumbering, Ack routing and slot ownership; exit codes; deleted legacy (`ch5_mode`, `pidGainsMode`, stats log, `Status`, `Battery`, `Command.id`). Haiku agents fix the findings the orchestrator accepts.

## Robot checks

Orchestrator only. Robot supported, wheels free, balance off unless a step says otherwise. Checks 1, 3, 4 and 6 run as soon as the firmware is flashed; 2, 5 and 7 need T7.

1. `scripts/build_remote.sh`, flash, `navlink info`: the build ID starts with `git rev-parse --short=8 HEAD`.
2. Rebuild the bridge in the worktree, reinstall both services, start them. `navlink get uptime_s` and a 10 s `navlink capture` run while the bridge publishes `/odom`; no `FramesLost` in either.
3. Flip CH5 through 0, 1, 2 and read `ch5_position` each time (checks `kCh5LinkRaw` and `kCh5RcRaw`).
4. `navlink call reboot --yes` is refused in positions 1 and 2 and accepted in 0; `navlink info --wait 40` returns with a new `uptime_s`.
5. Health: `navlink capture --preset health --seconds 5` shows plausible heap, stack, uptime and zero counters; `/diagnostics` and `/battery_state` arrive at 1 Hz.
6. `navlink set log_level 3`: no INFO lines in `navlink log --seconds 10`; back to 2.
7. With the user: balance in position 2, live `set speed_kp` within and over the step; 2 → 1 keeps balancing; `call balance on=0` and `on=1`; `/cmd_vel` at 0.1 m/s with `teleop_twist_keyboard`; stop publishing and the robot stops within the timeout; CH5 from 1 to 0 ends balance; 0 → 2 in one move starts balance only at 2, and 2 → 0 in one move ends it. Record the results in a lab note.
