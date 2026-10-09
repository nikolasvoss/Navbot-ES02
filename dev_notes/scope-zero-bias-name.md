Original Outcome (verbatim, user request 2026-10-09):
> zeroBias_t zeroBias;  // Zero offset TODO of what?
>
> rename variable
Acceptance Criteria: The variable name communicates that the values are calibration offsets, and every reference to that variable in the active sketch uses the new name.
Approved Approach: Rename the `zeroBias` object to `calibrationOffsets` throughout `OllieFOCdrive.ino`; remove the now-obsolete TODO comment.
Explicit Non-Goals: Do not rename the type or serial command handlers, change calibration behavior, or alter unrelated existing edits.
Compatibility / Migration Requirements: NONE

Task baseline: `28041f0c92d203088a06c2760fa5e9af8784f14f`.
Pre-existing changes: `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino` already had two user edits removing TODO comments from `DRIVE_TILT_REDUCTION_START_DEG` and the IMU low-pass filter comment. Preserve them.

Frozen implementation plan:
1. In `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`, rename the `zeroBias` variable and its references to `calibrationOffsets`, removing the declaration's vague TODO comment. Expected extent: a mechanical same-file identifier rename plus one comment removal.
2. Verify all identifier references in that sketch use the new name, inspect the task diff, and run the final scope checkpoint.

Progress: Step 1 is complete. Source search found no remaining `zeroBias` object identifier under `src`; `git diff --check` passed. No build or tests were run because this was a mechanical variable rename.

User amendment (2026-10-09): "name it imuCalibrationOffsets"
Amended Acceptance Criteria: Rename the variable `calibrationOffsets` to `imuCalibrationOffsets` throughout the active sketch, with no behavior change.
Plan amendment (scope-guard verdict: OK): Replace the current variable identifier `calibrationOffsets` with `imuCalibrationOffsets` in `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`. Expected extent: same-file mechanical identifier rename only.

User amendment (2026-10-09): "my fault, name it allCalibrationOffsets"
This latest naming instruction supersedes the preceding `imuCalibrationOffsets` amendment.
Plan amendment (scope-guard verdict: OK): Replace the variable identifier `calibrationOffsets` with `allCalibrationOffsets` in `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`. Expected extent: same-file mechanical identifier rename only.

Progress: The latest requested rename is complete. No `calibrationOffsets` or `imuCalibrationOffsets` identifier remains in the active sketch, and `git diff --check` passed. No build or tests were run because this was a mechanical variable rename.

User amendment (2026-10-09): "us `imuAndServoOffsets`"
This latest naming instruction supersedes the preceding `allCalibrationOffsets` amendment.
Plan amendment (scope-guard verdict: OK): Replace `allCalibrationOffsets` with `imuAndServoOffsets` throughout `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`. Expected extent: same-file mechanical identifier rename only.

Progress: The latest requested rename is complete. No superseded object name remains in the active sketch, and `git diff --check` passed. No build or tests were run because this was a mechanical variable rename.
