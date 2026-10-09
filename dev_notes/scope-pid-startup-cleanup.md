# PID startup cleanup

Original Outcome (verbatim): "implement with subagnet"
Source: user request following the approved proposal below.
Referenced proposal (verbatim):
1. Define startup gains once and use them directly in the configuration constructors, including Roll `0.08, 1.5, 0.005`.
2. Remove the startup `PidParameter()` block.
3. Rename the configuration objects to `anglePidSettings`, etc.
4. Keep `PidParameter()` for explicitly selected automatic gain mode.

Acceptance Criteria: Current build startup settings equal the former post-preload settings for all six controllers. No setup preload remains. Configuration objects have distinct settings names throughout the sketch. Automatic gain selection, gain scaling, serial commands and runtime controller state retain their behavior. The roadmap no longer requests review of the removed preload. Compile the firmware and compare old/new startup and automatic presets in a host harness.
Approved Approach: Reuse existing gain constants for Angle, Speed and Roll; give repeated Yaw/Touch defaults one source where needed. Remove redundant diagnostic gain assignments only when identical to selected presets. Keep SimpleFOC settings objects and custom runtime controllers.
Explicit Non-Goals: Runtime controller renames or constructor fixes; replacing Commander or PID types; changing gain values, CH3 scaling, timing or the diagnostic flag; flashing; unrelated edits; commits or PRs.
Compatibility / Migration Requirements: NONE

Baseline: HEAD 28041f0c92d203088a06c2760fa5e9af8784f14f. Pre-existing sketch modifications rename zeroBias and Select and edit unrelated TODO comments. Existing untracked scope-debug-log-selector and scope-zero-bias-name notes belong to other tasks. Exact task baseline snapshots are /tmp/navbot-pid-cleanup/baseline.ino and baseline-roadmap.md.

Frozen plan:
1. Independently trace the initialization contract and capture baseline host outputs before edits.
2. Delegate mechanical settings renames, canonical startup default initialization, preload removal and narrowly related roadmap adjustment.
3. Inspect task-only diff; compare baseline/new startup and automatic gains; build using scripts/build_firmware.py. Review scoped comments and correctness.
4. Obtain a fresh independent scope-guard verdict on the complete task diff before delivery.

Expected area: src/ES-02/OllieFOCdrive/OllieFOCdrive.ino and the preload sentence in docs/robot/ROBOT_ROADMAP.md. Working artifacts in this note, dev_notes/.scope-pid-startup-cleanup.todo.md, TASK_STATE.md and /tmp/navbot-pid-cleanup. Expected source extent: approximately 100-220 changed lines including mechanical renames, at most 20 named scalar defaults, no new runtime abstraction.

Progress: implementation integrated from isolated Luna output. Startup/automatic gain outputs match for both diagnostic flag values, including automatic resets after edited gains. Runtime posture and downstream loop exactly match after settings renames; callback bodies and command bindings match. Firmware build passed with scripts/build_firmware.py. Scoped comment review accepted seven deletions, no reinstatements or findings. Gemini was unavailable because its adapter rejected request validation before a review. Independent correctness review and final scope verdict are recorded in /tmp/navbot-pid-cleanup/review-results.txt. No flash or hardware test performed.
