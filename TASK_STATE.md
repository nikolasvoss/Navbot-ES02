# Kinematics, calibration, and persistence extraction

## Goal

Implement the user-selected `work/kinematics-calibration-implementation-plan.md` as a behavior-preserving refactor from baseline `e18c734dbf4c9417678e0768742b4e2c01f39b83`.

## Decisions

- Keep all work in the managed integration worktree on `codex/kinematics-calibration`.
- Preserve `zeroBias_t` as owner of attitude offsets and servo trims, and the ICM driver globals as owners of raw gyro biases.
- The current baseline already includes `SerialLogger`; it has structured telemetry APIs, not a suitable text diagnostic API. Preserve the current calibration messages within their extracted owner; do not create a second logging abstraction.
- No hardware mutation or flash. The current user request authorizes pushing this branch and opening a PR against `main`; do not merge.

## Progress

- Main baseline verified clean at the plan's revision; existing logging worktree remains untouched.
- Worktree created and task brief/scope contract copied in.
- Scope precheck returned `OK`.
- Grounding complete: solver/caller behavior, calibration lifecycle, storage keys/readback, build and host-test conventions traced.
- Architecture synthesis complete: selected live-owner references with typed per-operation store snapshots; exact declarations are in `work/kinematics-calibration-design.md`.
- Independent design cross-judge selected Candidate A (25/25); Candidate B scored 21/25 and C 10/25. Gemini CLI was unavailable after a 90-second timeout.
- Shared interface freeze is recorded. Three implementation worktrees, integration, verification, and reviews remain.
- Shared interface/sketch scaffold committed as `e35e362`.
- Three isolated worker worktrees were created at the scaffold commit. Developer code map updated in `control-flow.md`.
- Worker branches: `codex/kinematics-worker` at `/tmp/navbot-kinematics-worker`, `codex/calibration-worker` at `/tmp/navbot-calibration-worker`, and `codex/calibration-store-worker` at `/tmp/navbot-calibration-store-worker`.
- Store, calibration, and kinematics commits are integrated sequentially as `8ecb14c`, `ecc8d89`, and `10ebd77`.
- Final sketch is 2,166 lines, down from the baseline's 2,529 (363 fewer lines).
- Focused host checks pass: kinematics baseline comparison (648 finite poses per side, 9 nonfinite cases per side, clamp boundaries), calibration lifecycle, and calibration store.
- Normal firmware build passes: ESP32-S3, 738,124 bytes program storage (56%), 40,736 bytes globals (12%).
- Sensor-diagnostic firmware build passes from a temporary sketch copy: 407,558 bytes program storage (31%), 33,336 bytes globals (10%).
- `git diff --check` passes. No hardware was flashed or calibrated.
- Independent comment review removed three redundant namespace or provenance comments across the test fixture and store implementation.
- Fresh correctness review found no concrete issues in the complete branch diff, including the current comment cleanup.
- A scope audit found a stale delivery decision in this file; this update aligns it with the current user request.
- Re-ran all three focused host checks from the repository root with Bash. Kinematics baseline equivalence, calibration lifecycle, and calibration-store checks pass.
- Re-ran the normal firmware build and built sensor-diagnostic mode from a temporary sketch copy. Both ESP32-S3 builds pass with the recorded program and globals sizes above.
- No hardware was flashed. Physical behavior was not verified.

## Open items

- None.

## Next step

Review the open [kinematics and calibration extraction PR](https://github.com/nikolasvoss/Navbot-ES02/pull/3). Keep the worktree available. No hardware behavior was physically verified.
