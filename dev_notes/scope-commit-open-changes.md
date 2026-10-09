# Commit open changes

Original Outcome (verbatim): "commit every open change separate into usable commits"
Source: user request, 2026-10-09.
Acceptance Criteria: Every tracked and untracked open change is committed in focused, independently compilable commits. The working tree and index are clean. The final firmware source equals the starting working copy. No source changes, resets, rebases or pushes are introduced to organize the commits.
Approved Approach: Create cumulative snapshots for the two variable renames, existing comment edits and PID startup cleanup; stage exact snapshots using Git's index without changing the working sketch. Include each task's notes in its relevant commit. Compile each snapshot before its commit. Update current task state and commit this scope record with the final cleanup.
Explicit Non-Goals: New firmware behavior, additional cleanup, source repairs, history rewriting, pushing, flashing or hardware tests.
Compatibility / Migration Requirements: NONE

Baseline: HEAD 28041f0c92d203088a06c2760fa5e9af8784f14f. Index is clean. Open tracked files: TASK_STATE.md, docs/robot/ROBOT_ROADMAP.md, src/ES-02/OllieFOCdrive/OllieFOCdrive.ino. Open untracked files: dev_notes/.scope-debug-log-selector.todo.md, dev_notes/.scope-pid-startup-cleanup.todo.md, dev_notes/scope-debug-log-selector.md, dev_notes/scope-pid-startup-cleanup.md, dev_notes/scope-zero-bias-name.md.

Frozen plan:
1. Snapshot all starting open files under /tmp/navbot-commit-split. Generate cumulative source snapshots and verify the first two groups are complete renames and the third group contains only comments.
2. Compile and commit the calibration-offset rename with its scope note.
3. Compile and commit the debug-selector rename with its scope note and checklist.
4. Compile and commit the existing firmware comment edits.
5. Compile and commit the PID startup cleanup, roadmap change, PID scope note/checklist, current TASK_STATE.md and this commit-organization contract. Verify committed source bytes equal the original open sketch, all open changes are represented and the working tree is clean.
6. Obtain a fresh independent scope review over all commits since the baseline before delivery.

Expected area: Only the starting open files listed above, this task-specific note and local verification artifacts under /tmp/navbot-commit-split. Four new commits; no prior commits rewritten. Task tracking uses this note and TASK_STATE.md; no additional checklist is required for this commit-only follow-up to the refactoring playbook's commit step.

Verification records: /tmp/navbot-commit-split. Each intermediate snapshot must compile; the final snapshot also retains the prior PID gain-equivalence proof.

Verification progress: All four cumulative snapshots compiled successfully. The first three commits were checked to contain exactly their compiled sketch snapshots. The fourth commit includes the final PID cleanup and remaining task artifacts. Final commit byte equality and clean-tree checks are performed immediately after it; the independent scope verdict is recorded in /tmp/navbot-commit-split/scope-verdict.txt.
