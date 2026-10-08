# Task state: project-local logging module

Goal: implement the accepted project-local typed logging architecture from `work/logging-module-implementation-plan.md`.
Baseline: `e18c734dbf4c9417678e0768742b4e2c01f39b83`; integration branch `codex/logging-module-implementation`; checkout was clean before task changes.
Scope: `/home/niko/Dokumente/Bastelei/roboter/Navbot-ES02/work/logging-module-implementation.scope.md`.
Frozen plan: `/home/niko/Dokumente/Bastelei/roboter/Navbot-ES02/work/logging-module-implementation-plan.md`.
Architecture sketch: `/tmp/logging-architect/synthesis.md` (Candidate 3 with recorded in-scope grafts).

Decisions:
- Use one private tagged record queue and one sender. Capture builders are separate in `RobotLogCapture`.
- Publish profile plus 32-bit epoch in a brief critical section. Drop queued stale records at dequeue. An already-started UART write may finish as pre-boundary bytes; do not flush or wait in a control path.
- Preserve current output layouts, selectors, producer boundaries and rates, queue depth 32 and 5,376-byte queue budget if the named record fits. Any budget increase needs measured size and a scope review.
- Keep serial input and command processing. Route or suppress actual app/library output at its source during structured capture.
- No compatibility layer, Wi-Fi/perimeter integration, flash, drive, merge or push.

Progress:
- Product review: HOLD_SCOPE; scope stays as requested.
- Independent pre-edit scope check: `STATUS: OK`.
- Source/how exploration complete. Three Luna high architecture candidates and independent Luna cross-judge complete; Gemini returned `ok:false` after 90 seconds and is unavailable.
- Public sample/API types and private queue record declarations are frozen in
  `Logging.h`, `LogSamples.h`, and `LoggingInternal.h`.
- Host C++11 compile probe measured `LoggingInternal::Record` at 128 bytes,
  `Payload` at 120 bytes, and the largest debug payload at 44 bytes. The
  existing 168-byte slot guard passes, preserving the 5,376-byte queue budget.
- Commander/SimpleFOC library headers are not installed in the available
  Arduino cache; the installed SimpleFOC source confirms `Commander` accepts a
  `Stream&`. A local forwarding Stream retains serial input and drops Commander
  output outside Idle, and SimpleFOC monitoring is gated to Idle.
- Implemented the typed queue/sender and formatter. Host C++11 checks report a
  128-byte record, 32 slots, and 4,096 bytes of queue storage.
- Migrated selected debug, diagnostic, trace, control, balance, and drive
  sketch producers to named samples. Kept K8, K60–K75, and every-seventh-gate
  pacing at their existing call boundaries.
- Routed active diagnostic text through `Logging::message`; functional
  calibration output remains direct only in Idle. Runtime gyro calibration
  output is gated without changing its sample or storage path.
- Replaced the old logger host test with C++11 tests for all format fixtures,
  bounded FIFO/overflow, message filtering and profile boundary behavior,
  sender startup failure, short writes, and selected-debug pacing. Added
  independent capture-policy and drive-trace parser checks.
- Removed `SerialLogger.*`, `SerialLogFormat.h`, and the unreferenced
  `Telemetry.*` after repository-wide reference checks. Updated current USB
  serial and diagnostic guides.
- Current normal firmware build passed with 737,332 bytes of program storage
  (56%) and 39,656 bytes of globals (12%). The diagnostic build passed from an
  isolated temporary sketch copy with 407,074 bytes of program storage (31%)
  and 32,112 bytes of globals (9%).
- Current host checks passed: logger formatter/queue, capture policy, and drive
  trace parser. `git diff --check` passed.
- Fresh independent correctness review found no actionable findings. The final
  independent scope guard returned `STATUS: OK`; it verified the touchscreen
  serial blocks cited by an earlier report are commented out and unchanged from
  baseline.
- No-comments review found 12 redundant namespace-closing labels; all 12 were
  removed. No other added comments were flagged.
- Gemini review was attempted once with the correct request and timed out after
  90 seconds, so it is unavailable for this workflow.

Open items:
- None.

Delivery boundary: no hardware flash or motor test, merge, push, or PR, as
excluded by the scope contract.

Next step: report the branch, checks, review results, and hardware limitation.
