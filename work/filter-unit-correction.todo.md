# Bug fix playbook

1. Reproduce it yourself on the matching surface via the driver skill (Non-negotiables), even when a debug or instrumentation protocol says to ask the user to reproduce. Ask the user only with a stated, specific reason the control surface cannot reach the target, and only after driving it as far as it goes. If it won't reproduce directly, synthesize the trigger, tighten conditions, or instrument until it fires.
2. Binary-search the cause. Form the candidate hypotheses, then rule them out until one survives. Seed them with `how` over the affected subsystem and the **why** skill for regression history. Each pass, take the split that cuts the most remaining problem space, get runtime evidence, eliminate. When program state is unclear, add instrumentation or logging and read it as the code runs. Don't guess. Drive a long or stubborn hunt with Claude Code's `loop` skill. Confirm the surviving *mechanism* with runtime evidence before the step-3 architect/interrogate fan-out.
3. Plan the fix. If it crosses a function boundary, `architect` first. Delegate implementation to a subagent using your configured bug-fix model (default in poteto-mode's Models section) with a specific scope.
4. Verify on the same surface. The original repro now passes. "Inconclusive" or wrong-surface is not a pass. Flag it.
5. Stage the commits so the failing repro lands before the fix in git history. See the **tdd** skill for the failing-test-first cadence when the bug has a cheap local test path. Skip it when the test would be expensive, integration-heavy, or unclear.
6. Run **Opening a PR**.

# Work record

- Source branch checked: `codex/sensor-data-roadmap`, commit `a9c2131`; existing dirty changes are preserved in its other worktree.
- Dedicated worktree created at `/home/niko/.codex/worktrees/filter-unit-correction/Navbot-ES02` on `codex/filter-unit-correction`.
- Contract and frozen plan recorded in `work/filter-unit-correction.scope.md`.
- Caller/cadence investigation, regression test, firmware builds, independent scope reviews, and no-comments review have run. The review findings were applied under the recorded scope amendment.

# Completion record

- The initial independent scope check returned OK for the frozen plan.
- How exploration covered filter math, all source callers, and actual gates. The formula has existed since the initial project commit; the current branch commit only preserved diagnostic rate precision.
- Architecture selected the existing scalar API with cutoffHz and samplingRateHz. A tagged config type was rejected as wider than the task. A third runner and independent cross-judge could not start after the host returned an agent thread limit error. The Gemini adapter returned no visible JSON, so no Gemini findings are claimed.
- Regression test failed before the fix with gain 0.000075 against 0.707107 +/- 0.030, then passed with gain 0.707107.
- Normal and diagnostic builds succeeded without firmware transfer.
- Independent scope reviews and the no-comments review ran. The scope reviews approved the parameter-name-only declaration update and misleading-comment removals; later reviews found stale process notes, which have been corrected.
- Hardware sample rate and jitter remain unmeasured. The normal IMU call is per main-loop pass; RATE_HZ does not schedule it.
- The touchscreen.h declaration now uses samplingRateHz, matching its definition and the Hz-based filter initializer calls.
