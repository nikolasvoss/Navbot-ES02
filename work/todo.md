### Merge task checkpoint

- **Blocking first steps:** inspect both branch histories, the frozen source scope, target logger, output formats, and capture consumers before resolving the logger conflict.
- **Independent workstreams:** read-only source/target exploration and architecture candidates ran independently; implementation and harness updates were sequenced because they share the logger record API.
- **Shared mutable state:** one target `SerialLogger` queue and one sender own the asynchronous log stream. The separate source `Telemetry` sender was not retained.
- **Smallest safe decomposition:** port only selected snapshots and bounded failure behavior into the target logger; preserve the target schemas and existing cadence.

### Verification

- [x] Host blocked-sender test, including capacity 32, FIFO, reuse, and sticky overflow.
- [x] Normal firmware build.
- [x] Diagnostic firmware build with default macro restored.
- [ ] Independent scope gate, final diff check, merge commit.
