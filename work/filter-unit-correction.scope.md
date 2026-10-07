# Scope contract: biquad filter units

Source: user request in this task, 2026-10-07.

Original Outcome (verbatim): "Implementiere die Filterkorrektur im Navbot-ES02-Projekt."

Acceptance Criteria:
- The actual biquad filter functions produce the expected response for 20 Hz cutoff at 100 Hz sampling.
- A focused test invokes the real filter functions and detects the prior unit error before the fix.
- All directly affected call sites and parameter updates use explicit sampling-rate-in-Hz units.
- Diagnostic and normal firmware builds succeed.
- Filter execution cadence and consequences for Mahony fusion and the battery filter are documented.
- No hardware test, movement, or firmware transfer is performed.

Approved Approach: Correct the coefficient calculation in the existing biquad API using an explicitly named sampling rate in Hz; audit every direct initializer/update and runtime parameter-change path; add a small host test that compiles the production filter implementation; document the cadence evidence and resulting Mahony/battery filter behavior.

Explicit Non-Goals: No changes to PID gains, motor enablement, safety limits, logging transport, or USB configuration. No hardware movement or firmware upload. No additional legacy interface.

Compatibility / Migration Requirements: NONE.

Task baseline: source ref `codex/sensor-data-roadmap`, commit `a9c2131f5a8bce64755541241d8f7cd793e1601e`, checked out in this dedicated worktree. The source branch's other worktree has pre-existing uncommitted changes in `OllieFOCdrive.ino` and untracked `work/`; these are not present in this task worktree and must remain untouched.

Frozen implementation plan:
1. Add a host-side regression test that compiles and calls the production `filter.cpp`; verify 20 Hz / 100 Hz transfer behavior and capture the pre-fix failure.
2. Rename the internal biquad sampling-rate parameters to `samplingRateHz` and calculate angular frequency as `2*pi*cutoffHz/samplingRateHz`; audit and correct directly affected call sites and runtime changes without adding a compatibility API.
3. Trace where each affected filter is initialized and applied, and document the actual cadence evidence plus the effect of corrected coefficients on Mahony input fusion and battery smoothing.
4. Run normal and diagnostic firmware builds and inspect the complete diff.

Expected change area: `src/ES-02/OllieFOCdrive/filter.cpp`, `filter.h`, directly affected call sites in `OllieFOCdrive.ino`, `OllieFOCdrive.h`, and `touchscreen.cpp`, one focused host test and its minimal support/build invocation, plus the relevant robot software documentation. Any extra file or wider change requires a scope check before implementation.

## Approved plan amendment

An independent scope gate approved the following narrowly scoped additions before implementation:

- Rename the parameter in `src/ES-02/OllieFOCdrive/touchscreen.h` from `cutoffFreq` to `samplingRateHz`. The declaration is directly affected by the Hz-units acceptance criterion; this changes only the parameter identifier, not the function type or behavior.
- Remove the newly added inline rate comments from `OllieFOCdrive.ino` because the 1 ms gate is a threshold rather than a guaranteed sample rate, and `ReadVoltage()` also runs every 10 ms in diagnostic mode. Cadence details remain in the required documentation.
- Retain `work/filter-unit-correction.todo.md` as uncommitted PStack process metadata because poteto-mode requires a todo checklist when task tracking tools are unavailable. It is not part of the product change.

## User amendment: live hardware diagnostic

The user explicitly requested changing the diagnostic baud to 576000 and proceeding with the live test. Set the diagnostic UART to 576000 baud and its frame interval to 10 ms so the existing CSV stream can carry one row per nominal 100 Hz IMU update; preserve the CSV schema and the normal/live-tuning baud settings. Build and flash only the sensor-diagnostic image, keeping motor enables low. No motion or normal control image is authorized by this amendment.

## User authorization and scope gate for live results

The user later said, "change to 576000 baud. lets do the live test." The user then confirmed, "ROBOTER IST GESICHERT. los", and confirmed reconnecting the USB-C cable. These instructions authorize the diagnostic-only build and flash, plus a stationary sensor capture with motor enables low. They override the original no-hardware-transfer criterion only for this diagnostic image. The normal control image, motor actuation, and hardware movement remain excluded.

The independent scope gate approved these documentation additions before editing: update `control-flow.md`, `sensor-diagnostic-mode.md`, and `TASK_STATE.md` with the observed capture and its limits, and add one hardware finding under `agent_notes/robot/hardware-findings/` as required by `AGENTS.md`. No source, test, build, or transport changes are included in this plan amendment.

## Approved merge-resolution amendment

A fresh independent scope gate approved two necessary integration fixes after the target branch introduced a diagnostic sampling interval of 10,000 µs:

- Define the diagnostic filter sampling rate in Hz from `DIAGNOSTIC_IMU_INTERVAL_US` and pass it to the two diagnostic filter initializers. This keeps target-branch call sites consistent with the corrected API and the 100 Hz nominal cadence.
- Correct `sensor-diagnostic-mode.md` to state that `SENSOR_DIAGNOSTIC_MODE` currently defaults to `0`, matching the merged source and its safe normal-mode default.

The gate found no other required additions. It returned `OK` before these edits.
