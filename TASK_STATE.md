# Task state: PR4 integration review

Goal: review PR4 against `main`, fix confirmed blockers, and merge only a verified head. The active contract is `work/pr4-review.scope.md`; compatibility and migration requirements are none.

## Decisions

- Integrate the current `main` kinematics and calibration extraction with PR4's typed logging migration. Keep `LegKinematics`, `Calibration`, and `CalibrationStore` as owners of their extracted behavior.
- Keep the `LoggingCommandStream` input path for Commander. Send calibration and solver text through `Logging::message` so measurement profiles suppress it.
- Preserve producer selectors and cadence. Debug selectors 1–45 and 60–75 use their existing gates; trace selectors 55–58 use the existing every-seventh-control-gate cadence.
- No hardware operation or flashing. The lead owns final verification and merge; this worktree step makes no commit or push.

## Progress

- PR head: `094c4528bf1c45beefd303cfb4b090183abe683c`. The integration fetch brought in `main` at `c1950d2`.
- PR4's typed logger, formatter, capture policy, and migrated producers were implemented and previously reviewed. The earlier firmware and host-check results are recorded in the branch review materials.
- `main` contributes the `LegKinematics`, `Calibration`, and `CalibrationStore` modules. Their focused host checks and normal and sensor-diagnostic firmware builds passed on the extraction branch; no hardware was flashed.
- Reconciled the sketch to load persisted calibration through `CalibrationStore`, update it through `Calibration`, and use `LegKinematics` results for servo angles. Removed the duplicate inline calibration and old inverse-kinematics paths from the conflict resolution.
- Routed startup calibration values, calibration completion values, and solver warnings through `Logging::message`.
- Fixed the `print_data` selector fall-through. Selectors 55–58 now reach the existing trace switch after the 1–45 debug block is skipped.
- Added `scripts/test_print_data_dispatch.py`. It extracts the real selector guard and trace switch from `print_data`, compiles them with a focused host harness, and checks all four trace selectors and their 7-gate cadence. The harness failed before the fix and passes after it.

## Verification

- Integrated logger, capture, dispatch, analyzer, kinematics, calibration, and calibration-store host checks passed.
- The normal firmware build passed after repairing the comment opener at `print_data`.
- Independent correctness review returned PASS+NOTES. Gemini CLI timed out and supplied no usable review.
- No hardware was flashed or operated.

## Open items

- Complete the final scope check and confirm the guarded merge.
- Merge and confirm landing only if the lead's verification is clean.

## Next step

Finish the final scope check, publish the verified integration commit, and merge PR4 with the verified head condition.
