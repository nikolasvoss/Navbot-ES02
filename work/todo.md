### Feature

**You own the design. Plan, review, verify.** Delegate implementation. Stay in the lead.

1. `how` over the affected subsystem.
2. `architect` for parallel design exploration.
3. Write the throughput checkpoint as four todo items. A dimension that genuinely does not apply (single file, no fan-out) keeps its item with `n/a: <reason>` rather than being dropped:
   - **Blocking first steps.** Gates run before fan-out.
   - **Independent workstreams.** Disjoint files, services, or layers parallelize. Shared writes serialize.
   - **Shared mutable state.** Default to splitting the target (the **separate-before-serializing-shared-state** principle skill). Serialize only for real invariants.
   - **Smallest safe decomposition.** If one worker is best, name why.
4. Delegate code-writing to a subagent using your configured feature model (default in poteto-mode's Models section) with a specific scope (file paths, named data shape and its organizing structure per **principle-model-the-domain**, a state machine over scattered booleans, a table/registry over branching, a typed model over repeated shape assumptions, chosen before the delegate writes logic, and success criteria). When the implementation admits multiple valid shapes (error handling, abstraction layer, test structure), delegate via the **arena** skill instead so the runners surface the alternatives and the cross-judge guards the pick. Mandatory: no skip-with-reason escape, and Laziness Protocol does not override it (the gain is review separation, not lines saved). A subagent forbidden to spawn satisfies this by owning the diff directly with the same review separation. No "standing by" reply that waits on a nested agent. **Give every file-writing delegate its own worktree** (spawn it with `isolation: "worktree"`, or hand it an exclusive branch), and do not write files or run a suite in a worktree a delegate still holds. Fencing a file in the brief's prose is not a lock (**principle-separate-before-serializing-shared-state**). Comments per **Comments**. Surgical edits, re-ground against the source for upstream-derived files. Port shared-primitive improvements to all consumers and verify each. Commit liberally.
5. Verify on the matching surface. "Inconclusive" or wrong-surface is not a pass. Flag it.
6. Rebase into small, ordered commits. Stack follow-ups.
   Use the **sequence-verifiable-units** principle skill, building, verifying, and committing each small unit before the next.
7. If the design is contested, `interrogate` before shipping.
8. Run **Opening a PR**.

**Throughput checkpoint.**

- **Blocking first steps.** Scope contract, frozen plan, and architecture already exist; no fan-out gate needed.
- **Independent workstreams.** Production changes and host harness depend on a shared API, so one owner keeps them sequenced.
- **Shared mutable state.** The production module and harness API must agree; serialize edits in this worktree.
- **Smallest safe decomposition.** One owner is best because producer and consumer formats must agree; the task is already assigned to one implementation agent.

**Feature steps.**

- [x] Ground the affected subsystem using the approved architecture and source.
- [x] Keep a typed record shape with one producer and one consumer.
- [x] Implement the production queue and firmware call sites. Host compilation of `Telemetry.cpp` confirmed lock-free 32-bit atomics on the host toolchain.
- [x] Add and run the focused host test. `g++ -std=c++11 -Wall -Wextra -Wno-format-nonliteral -pthread ... && timeout 10s ...` passed.
- [x] Inspect the diff, run `git diff --check`, and commit for lead integration.

**Open decisions.** None. Plan is frozen.
