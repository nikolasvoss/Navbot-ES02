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
  Arduino cache; integration will gate known sketch output sites and retain
  serial command processing.
- Gemini implementation review has not run yet; it remains required once the
  shared implementation diff is available.

Open items:
- Commit the interface scaffold and dispatch disjoint engine/formatter and robot-capture workers in separate worktrees.
- Map every frozen debug selector and sample type to baseline field construction and serializer format fixtures.
- Build, test, inspect, and obtain independent correctness and final scope reviews.

Next step: commit the frozen interface scaffold, then dispatch engine/formatter and robot-capture work to Luna high workers in separate worktrees.
