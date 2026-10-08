# Kinematics, calibration, and persistence extraction

## Goal

Implement the user-selected `work/kinematics-calibration-implementation-plan.md` as a behavior-preserving refactor from baseline `e18c734dbf4c9417678e0768742b4e2c01f39b83`.

## Decisions

- Keep all work in the managed integration worktree on `codex/kinematics-calibration`.
- Preserve `zeroBias_t` as owner of attitude offsets and servo trims, and the ICM driver globals as owners of raw gyro biases.
- The current baseline already includes `SerialLogger`; it has structured telemetry APIs, not a suitable text diagnostic API. Preserve the current calibration messages within their extracted owner; do not create a second logging abstraction.
- No hardware mutation, flash, push, or merge.

## Progress

- Main baseline verified clean at the plan's revision; existing logging worktree remains untouched.
- Worktree created and task brief/scope contract copied in.
- Scope precheck returned `OK`.
- Grounding complete: solver/caller behavior, calibration lifecycle, storage keys/readback, build and host-test conventions traced.
- Architecture synthesis complete: selected live-owner references with typed per-operation store snapshots; exact declarations are in `work/kinematics-calibration-design.md`.
- Independent design cross-judge selected Candidate A (25/25); Candidate B scored 21/25 and C 10/25. Gemini CLI was unavailable after a 90-second timeout.
- Shared interface freeze is recorded. Scaffold, three isolated implementation worktrees, integration, verification, and reviews remain.

## Open items

- Create the shared scaffold and confirm fake-Preferences semantics in host tests.
- Determine baseline build availability and establish focused host comparison fixtures.

## Next step

Create and commit the shared interface/sketch scaffold, then create each worker worktree from that exact commit.
