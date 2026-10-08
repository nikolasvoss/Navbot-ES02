# Task state: diagnostic defaults explanation and servo pin naming

Goal: explain the reasons for the diagnostic tuning defaults and IMU sampling cadence, then rename the four servo signal pin macros.

Active contract: `work/servo-diagnostic-followup.scope.md`.

## Decisions

- Keep `DIAGNOSTIC_LIVE_TUNING_DEFAULTS=1`. The flag supplies the documented startup profile, live tuning, CH3 scaling, and measured wheel timing. Its name understates its normal-control effects.
- Keep the 10 ms diagnostic IMU interval. The sensor-only logger reads at nominal 100 Hz while the IMU continues producing samples at 1 kHz. The 100 Hz cadence aligns its 20 Hz filters and keeps the serial log at 100 rows/s; docs do not establish why 100 Hz is uniquely best.
- Rename the servo constants to `LEG_SERVO_n_SIGNAL_PIN` to express that they select four leg-servo signal GPIOs in constructor order.
- Make no control or timing changes.

## Progress

- Read the project workflow, refactoring playbook, source, hardware reference, control/tuning docs, and sensor diagnostic guide.
- Recorded the task baseline and pre-existing worktree edits in `work/servo-diagnostic-followup.scope.md`.
- Read-only how review confirmed that disabling the tuning flag also restores fixed wheel-speed timing, and confirmed why diagnostic filter coefficients follow the 100 Hz software cadence.
- Renamed all four servo pin constants and the constructor references without changing GPIO values or order.
- Source search found no old macro names. `git diff --check` passed. The default firmware build passed at 741,908 bytes program storage and 39,648 bytes global RAM.
- Gemini CLI review timed out after 90 seconds and returned `ok: false`; no findings were used.
- Final independent scope check returned `STATUS: OK` with no scope gaps.

## Next step

Task complete. No control or timing behavior was changed.
