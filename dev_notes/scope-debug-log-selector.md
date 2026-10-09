Original Outcome (verbatim, user request 2026-10-09):
> debugLogSelector rename, then remove the todo
Acceptance Criteria: The `Select` variable is renamed to `debugLogSelector` in the active sketch, and the TODO attached to its declaration is removed.
Approved Approach: Rename the variable and its code references in `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`, update the nearby explanatory comment, and remove the declaration's TODO. Preserve the registered serial command label `"my Select"`.
Explicit Non-Goals: Do not change selector behavior, serial command protocol, unrelated existing edits, or other TODO comments.
Compatibility / Migration Requirements: NONE

Task baseline: `28041f0c92d203088a06c2760fa5e9af8784f14f`.
Pre-existing changes: `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino` already has edits concerning `DRIVE_TILT_REDUCTION_START_DEG`, `imuAndServoOffsets`, complementary-filter comments, and `accCoef`; `dev_notes/scope-zero-bias-name.md` is untracked. Preserve all of them.

Frozen implementation plan:
1. Record this Scope Contract and its short playbook checklist as required local workflow notes. Expected extent: `dev_notes/scope-debug-log-selector.md` and `dev_notes/.scope-debug-log-selector.todo.md` only for local task tracking.
2. In `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`, mechanically rename the `Select` variable and its code references to `debugLogSelector`, update the explanatory comment, and remove the declaration's TODO. Preserve the registered serial command label. Expected extent: one sketch file, identifier and comment edits only.
3. Verify the identifier references and inspect the final task diff, then run the independent final scope checkpoint.

Progress: The variable and its code references are renamed. The declaration TODO is removed, and the serial command label is preserved. Source search and `git diff --check` passed. No tests or build were run for this identifier-only rename.
