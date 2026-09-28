# Test plan: buffered Wi-Fi logging

Planning artifact, 2026-09-28. No tests have run yet. The [implementation brief](wlan-logging-implementation.md) contains the complete contract. QA surfaces: firmware/internal, network protocol and Python CLI; no UI.

| Area | Cases | Pass condition |
|---|---|---|
| Baseline | Dirty tracked/untracked sources, absent/present tuning service | Snapshot preserved in isolated worktree; correct integration path |
| C++/Python wire | All 26 fields, endian, u64 timestamps, NaN, CRC known vector, frame CRC vs whole-record CRC | Independent implementations agree on golden bytes and sizes |
| Ring ownership | Wraparound, full, partial consumer progress, reset/cleanup race | No overwrite, torn record, use-after-free or producer wait |
| State | Prepare lease, start before data ready, duplicate start/stop, wrong boot/session, drain/ACK expiry | One session, deterministic bounded transitions |
| Clock | Irregular ticks, deadline crossing, long uptime | Device duration honored; actual sample spacing retained |
| Config | U0, active calibration, network/Serial mutation during prepare/capture | Reject or explicit CONFIG_CHANGED; no deferred writes |
| TCP | Partial send, EAGAIN/EINTR, one-byte reads, combined frames, stall/close | Correct progress, bounded memory, explicit recorder failure |
| Integrity | Gap/duplicate, altered header/payload, oversized lengths, missing END, mismatched counts/CRC | Never complete or rename a corrupt file |
| PC storage | No overwrite, disk full, bad directory, fsync error, CTRL-C, ACK timeout | Useful partial retained; validated file not invalidated solely by lost ACK |
| Auth | Missing token, wrong/expired/reused ticket, unauthenticated slow client | Bounded resources; no data access/start |
| Regression | Legacy feature off, tuning if present, existing analysis tests | Prior command/RC behavior preserved |

Build baseline/feature off/feature on with same installed core and libraries, USB CDC disabled, PSRAM off, dummy secrets only. Run CLI full QA (`--help`, status, capture against bounded fake server, decode good/partial/bad files, all exit codes). Do not substitute Python-model tests for actual C++ encoder/ring tests.

Hardware acceptance after separate upload authorization: repeated 20/30/40-s battery-powered no-USB captures; compare baseline/Wi-Fi idle/streaming tick distribution, max latency, producer time, heap/stack, high-water and watchdog behavior. Zero unexplained gaps for successful runs. Deliberate stalls below/above measured reserve. RC still controls robot; STOP only stops logger. No hardware available means these remain explicitly unexecuted, not that host implementation must stop.

Record commands, versions, results and artifacts. End review with proven capabilities vs untested timing limits. See main brief for full acceptance criteria and downstream skills.
