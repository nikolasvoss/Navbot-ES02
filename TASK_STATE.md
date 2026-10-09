# Task state: commit open firmware changes

Goal: commit every open change separately into usable commits.
Contract: dev_notes/scope-commit-open-changes.md.
Decisions: group calibration-offset naming, debug-selector naming, existing comment edits and PID startup cleanup into four commits. Include related task notes with each change. Stage exact cumulative snapshots without changing the working sketch.
Progress: all four firmware snapshots compiled successfully with scripts/build_firmware.py. Each committed sketch is checked against its compiled snapshot. The final source equals the original open sketch.
PID verification: startup and automatic gains matched the pre-cleanup baseline for both diagnostic flag values. The prior independent correctness and scope reviews passed.
Verification records: /tmp/navbot-commit-split/build-stage1.log through build-stage4.log and commits.txt. Final scope verdict: /tmp/navbot-commit-split/scope-verdict.txt.
Hardware: no flash or drive test performed.
Next: final independent scope check and report the commit hashes.
