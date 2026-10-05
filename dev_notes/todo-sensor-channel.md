# Reusable sensor channel

- [x] 1. `how` over the affected subsystem.
- [x] 2. `architect` for parallel design exploration.
- [x] 3. Write the throughput checkpoint as four todo items.
- [x] 4. Delegate code-writing with a specific scope; owner completed and reviewed the work after the writer hit its usage limit.
- [x] 5. Verify on the matching surface, including synthetic browser publish and service request.
- [x] 6. Commit the coherent feature in the target worktree as `4a8e854`.
- [x] 7. If the design is contested, `interrogate` before shipping. Not needed; independent design review selected the approach.
- [ ] 8. Run **Opening a PR**. No PR requested or created.
- [x] Final independent scope check returned `STATUS: OK`.

Architect phases: Ground, Sketch, Agree, Implement, Scrap.
Arena phases: Frame, Fan out, Cross-judge, Pick, Graft, Verify.

Throughput checkpoint:
- [x] Blocking first steps. Grounding, design candidates and independent cross-judge complete.
- [x] Independent workstreams. One implementation writer; owner verification follows.
- [x] Shared mutable state. Implementation was isolated at `/tmp/navbot-sensor-channel`, then copied into the target HMMD worktree after Git metadata writes were denied.
- [x] Smallest safe decomposition. One worker owns transport, manifest, UI, startup and directly related docs/tests because these share the same endpoint contract.

Implementation and verification are complete in the target worktree. See `handoff-sensor-channel.md` for scope and checks. The next task's agent brief is in `dev_reference/plans/hmmd-radar-config-agent-instructions.md`.
