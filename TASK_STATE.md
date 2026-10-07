# Task state: sensor roadmap review fixes

Goal: fix the two confirmed findings from the full review of codex/sensor-data-roadmap.
Baseline: 49fbc524d96f80a29ebc36e95aeb677c0e165ef8; target worktree was clean.
Scope and frozen plan: work/review-fixes.scope.md.

Decisions:
- Pace selected debug records to at most 50 Hz within SerialLoggerSubmit. The first record is immediate; intentional pacing does not increment failure counters. Trace/diagnostic rates and sticky real-failure behavior remain unchanged.
- Correct README to state that the default is normal motor-control firmware and diagnostics require setting SENSOR_DIAGNOSTIC_MODE to 1.

Completed:
- Baud-limited 1 kHz K9 regression failed before the fix with `1 kHz selected debug submissions overflowed the UART queue`.
- The same regression passes after the fix with `selected_debug_accepted=50 trace_after_1khz=verified`.
- Existing queue, formatter, and filter host tests pass.
- Normal and diagnostic firmware builds pass. The diagnostic build uses a separate temporary sketch copy; the verified snapshot defaults to normal mode. A separate local edit now sets SENSOR_DIAGNOSTIC_MODE to 1; it remains unstaged and excluded from this commit.
- Independent Luna correctness review returned PASS. Gemini CLI was unavailable after a 90-second timeout.

Delivery: the tested fixes were applied after an independent scope verdict of OK. The user then requested a logging-command cleanup note and a local commit. The roadmap records that follow-up; cleanup itself is not implemented.
Next step: commit the verified fixes and roadmap note after the final scope check, preserving the separate local diagnostic-mode edit. No firmware upload or hardware action was performed.
