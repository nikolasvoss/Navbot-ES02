# Task state: biquad filter unit correction

Goal: Correct biquad frequency units for diagnostic and active regulation on top of codex/sensor-data-roadmap, with real-function regression coverage and builds. Scope contract and frozen plan are in work/filter-unit-correction.scope.md.

Baseline: a9c2131f5a8bce64755541241d8f7cd793e1601e. The source branch's pre-existing uncommitted OllieFOCdrive.ino change and work/ files belong to its other worktree and are excluded.

Completed: Dedicated worktree and branch codex/filter-unit-correction created. Corrected biquad coefficient math and Hz parameter names. Audited and updated all production initializer calls and runtime reinitialization paths, including the TouchscreenInit declaration. Added a host test that compiles the actual filter.cpp and checks the 20 Hz / 100 Hz response. Documented source-proven cadences and Mahony/battery effects. Normal and diagnostic firmware builds succeeded. The diagnostic image was later flashed under the user's live-test authorization. Independent scope review and comment review completed; their accepted findings were resolved and scope-approved.

Verification: Before the production fix, the real-function test printed gain 0.000075 against 0.707107 +/- 0.030 and failed. After the fix and final review updates it printed gain 0.707107 and passed. Final normal build used 729644 bytes program storage and 35368 bytes global memory. The diagnostic build used 395680 bytes program storage and 27856 bytes global memory. The diagnostic image was flashed from `/tmp/navbot-filter-live/output`; the flash tool verified hashes for the bootloader, partition table, and application. No normal control image was flashed. `git diff --check` passed.

Hardware capture: On 2026-10-07, a 60000-byte motor-disabled diagnostic capture produced 250 complete numeric rows, all with `imu_ok=1`. The 249 timestamp intervals had 9.966 ms minimum, 10.000 ms median, 10.040 ms maximum, and no interval over 15 ms. Filtered-to-unfiltered standard-deviation ratios were 0.641 for gyro X and 0.733 for acceleration Z. This confirms the stationary diagnostic stream timing and smoothing only. No controlled frequency sweep was done, so the 20 Hz physical cutoff response remains unmeasured. Normal-mode IMU cadence and behavior under motor load also remain unmeasured. The diagnostic image remains on the board; the normal control image was not flashed.

Open: The normal IMU path has no fixed software sampling gate; RATE_HZ shapes coefficients but does not schedule samples. Its actual cadence and cutoff under load remain unmeasured. No architecture cross-judge was available. The Gemini review adapter did not return a usable review result.

Next: Run final source/test/diff checks and the independent complete-diff scope gate, then deliver the verified local diff with the hardware measurement limits stated.
