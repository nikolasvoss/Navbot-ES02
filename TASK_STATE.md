# Task state: merge bounded serial logging

Goal: merge source commit `5aa6f78cad350b6058b18ef35d7841934db1a0cb` into `codex/sensor-data-roadmap` while preserving the target branch's existing serial formats, capture tools, rates, and control behavior.

Target baseline: `f4e16591aaed9f4354f32242f9265d729bc33685`. The source commit was based on `a9c2131f5a8bce64755541241d8f7cd793e1601e`; the branches diverged after that base.

Decision: extend the target branch's existing `SerialLogger` as the sole bounded queue and asynchronous logger. A second `Telemetry` sender would duplicate queue ownership and risk changing the target CSV contracts. The logger now accepts selected-debug snapshots, uses one statically allocated 32-record internal-RAM queue, formats in its sender, and latches a sticky incomplete reason on failure. The target's diagnostic and TRACE/CTRL/BAL/DRIVE formats, CH340 baud, trace cadence, diagnostic interval, motor logic, and safety behavior remain the target contracts. No incomplete marker is emitted.

Progress:
- [x] Inspect both branch histories, target logger, telemetry helper, producer paths, capture consumers, and serial notes.
- [x] Resolve the merge conflict with one queue and one asynchronous log sender.
- [x] Move selected `print_data()` cases 1–45 to fixed snapshots; retain the target TRACE/CTRL/BAL/DRIVE producer rows and their cadence.
- [x] Add bounded overflow status and blocked-sender host checks.
- [x] Host logger test passed: blocked write, immediate producer return, depth 32, sticky overflow/rejection, FIFO, and queue reuse.
- [x] Normal firmware build passed: 737,236 bytes flash; 40,976 bytes globals.
- [x] Diagnostic firmware build passed with the source macro restored to normal mode afterward: 407,414 bytes flash; 33,456 bytes globals.
- [ ] Finish independent scope review and final diff check.
- [ ] Create the merge commit on `codex/sensor-data-roadmap` (no push requested).

Hardware activity: none during this merge. Balance timing quality under motor load remains unmeasured.
