# WLAN logging implementation run

## Figure-it-out playbook

- [x] Read the Principles section of the poteto-mode skill.
- [x] Phase A: Frame the falsifiable definition of done, scope, blockers and rigor.
- [x] Phase B: Design the workflow and verification harness before implementation.
- [x] Phase C: Run each unit as an experiment and verify it before continuing.
- [x] Phase D: Keep the decision trail as each unit lands in `dev_reference/plans/wlan-logging-decisions.tsv`.
- [x] Phase E: Verify the full result against the Phase A predicate and hand it back; hardware acceptance is explicitly unexecuted.

## Designed workflow

- [x] Record branch, HEAD, dirty state and dependency versions without changing existing user files.
- [x] Compile the existing firmware baseline with the documented board profile.
- [x] Write the protocol contract and golden byte fixtures; add portable codec/state/ring tests.
- [x] Add bounded firmware recording state, memory ownership, and data transport behind the opt-in WLAN flag.
- [x] Add the Linux recorder and decoder with fragmented-stream, partial-file, interruption and corruption coverage.
- [x] Integrate loop snapshots and parameter locking; build feature-disabled and feature-enabled images.
- [x] Run security review, CLI QA, feature/code review and documentation release checks; see `dev_reference/plans/wlan-logging-gates.md`.
- [x] Record hardware acceptance as not executed; do not flash.

## Skips and constraints

- skip: isolated worktree and baseline commit; the user explicitly requested work on the current branch.
- skip: parallel architecture agents; this runtime's higher-priority collaboration policy forbids spawning agents unless explicitly requested.
- skip: write artifacts under `~/.gstack/projects`; that path is outside the writable roots, so equivalent review artifacts stay under `dev_reference/plans/`.

## Current verification checkpoint

- Feature-on compile passed with both feature macros enabled and a clean Arduino build.
- Feature-off compile passed with recording disabled; this checkout's ignored local header keeps WLAN tuning enabled.
- C++ telemetry codec/ring/state and tuning parameter tests passed.
- Full Python script suite passed (29 tests), including 15 WLAN tuning-client tests; drive analyzer tests passed (3 tests).
- CLI QA, security review, feature/code review, and documentation release outcomes are recorded in `dev_reference/plans/wlan-logging-gates.md`.
- Fresh-device token setup now uses the physical USB-Serial workflow in `dev_reference/wifi-parameterterminal.md`. HTTP/TCP remain plaintext for trusted WLAN use.
- Hardware acceptance was not run and no firmware was flashed.

## Live follow-up on 30.09.2026

- [x] Flash the recording-enabled build to the discovered CH340 port and verify written hashes.
- [x] Confirm the live recording API, two complete 20-second captures, decoder output, ACK, and session reuse.
- [x] Confirm that `SIGINT` saves a valid `USER_STOP` capture and returns the device to `COMPLETE` with an empty ring.
- [ ] Compare loop timing with a feature-off build and measure heap and task-stack margin under representative movement.
- [ ] Inject a stalled sender, network loss, and disk-full conditions on hardware.

The earlier checkpoint above describes the original implementation run. The later bench evidence is recorded in `agent_notes/wireless-logger-hardware.md`.
