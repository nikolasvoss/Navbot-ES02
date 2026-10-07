# Task state: merge filter correction into sensor roadmap

Goal: merge commit `c61c3f9` from `codex/filter-unit-correction` into `codex/sensor-data-roadmap`, preserving the target branch's SerialLogger format and behavior.

Target baseline: `59a35d0b485a6b111762d25490dfc92e16ad846b`. The source commit shares ancestor `a9c2131f5a8bce64755541241d8f7cd793e1601e` and was independently scope-reviewed before integration.

Decision: retain the target branch's baud values, 17-field asynchronous diagnostic logger, and microsecond IMU schedule. Merge the filter coefficient and caller updates. Treat the live 25-field capture as evidence for the earlier diagnostic image only; it does not validate the target branch's current SerialLogger transport.

Progress:
- [x] Verify target branch and worktree are clean.
- [x] Merge the scoped filter correction and resolve conflicts in `TASK_STATE.md`, `sensor-diagnostic-mode.md`, and `OllieFOCdrive.ino`.
- [x] Host filter regression passed: gain 0.707107 at 20 Hz / 100 Hz.
- [x] Normal firmware build passed: 737064 bytes program storage and 40848 bytes globals.
- [x] Diagnostic firmware build passed: 407262 bytes program storage and 33328 bytes globals.
- [x] Inspect all filter call sites and the complete merge diff; `git diff --check` passed.
- [ ] Run the independent final scope check and commit the merge.

Hardware: no new hardware action during this merge. The existing diagnostic image remains from the earlier live test; no firmware was flashed as part of this integration.
