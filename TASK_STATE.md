# Task state: SLAM map persistence and autostart

Goal: points 1 to 4 from the SLAM branch review (single saved map loaded at start, save/delete from the web, SLAM autostart with the LiDAR service, stale detection, radar port over SSH).
Contract and plan: dev_notes/scope-slam-map-autostart.md.
Baseline: feat/slam d8226de.
Test venv (dev PC): scratchpad venv with pytest, flask, lds2d[viz]==0.7.0; baseline 16 passed, 3 skipped.

Progress: done. Implemented, 26 passed / 3 skipped, demo service probed live, final scope check OK.
Open: first run on the ros2_pc with ROS + slam_toolbox (serialize result field, map_file_name loading, TF pose age, supervisor shutdown are unverified).
Next step: ros2_pc agent follows dev_notes/handoff-slam-ros2_pc.md.
