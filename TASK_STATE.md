# Task state: SLAM map persistence and autostart

Goal: points 1 to 4 from the SLAM branch review (single saved map loaded at start, save/delete from the web, SLAM autostart with the LiDAR service, stale detection, radar port over SSH).
Contract and plan: dev_notes/scope-slam-map-autostart.md.
Baseline: feat/slam d8226de.
Test venv (dev PC): scratchpad venv with pytest, flask, lds2d[viz]==0.7.0; baseline 16 passed, 3 skipped.

Progress: contract written; implementation delegated.
Open: implementation, verification, final scope check, commit.
Next step: review the delegate diff.
