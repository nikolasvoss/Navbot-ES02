# WLAN logging implementation gates

## Security review

**Risk tier: high. Status: issues found, no critical code finding.**

- **Resolved: no terminal bearer token.** The parameter and recording control APIs accept requests from any client that can reach the robot over Wi-Fi. This is an accepted development-tool behavior; leave the Wi-Fi feature disabled in normal firmware builds.
- **Medium: plaintext on the WLAN.** HTTP control requests and TCP telemetry are plaintext and unauthenticated. Use only on a trusted development WLAN. Do not expose or port-forward these ports.
- **Low: source revision absent from metadata.** Metadata sets `source_hash_available` to false. The guide and protocol document this reproducibility limit.

No critical code issue was found in the targeted manual review. The protocol has bounded frames and payloads, validates IDs and checksums, uses a one-use stream ticket, and leaves retained partial output on client-side transfer failures. These checks do not establish timing, heap, task-stack, or throughput margins on hardware.

## CLI QA

**Status: pass for the local/offline test matrix.** The 21 WLAN/tuning Python tests passed, including a mocked status request, fragmented stream input, malformed/truncated input, partial-file retention, and no-overwrite publication. The decoder passed a synthetic complete fixture and a truncated fixture with `--allow-partial`; missing config returned its documented input-error status. Both scripts' help commands passed. Existing drive analyzer tests passed (3 tests).

On 30.09.2026, live status, prepare, start, ACK, repeated recording, and a stop triggered by `SIGINT` also passed. Three new host regression tests cover directory `fsync`, invalid UTF-8, and invalid JSON `NaN`; the current WLAN/tuning suite has 30 passing tests. Disk-full and network loss during recording remain untested.

## Feature and code review

**Status: manual code review complete; hardware acceptance open.** Review corrections made during implementation include publishing stream readiness only after the receiver handshake response is sent, setting the ACK deadline before publishing the state, retaining failure state while draining, rejecting late output overwrite atomically, and including calibration state in the configuration fingerprint. The targeted comment review also found and fixed a stale feature-flag comment. No suppressions were introduced.

Remaining integration risks are listed in [the risk register](wlan-logging-risk-register.md), including device configuration drift, stalled-sender control timing, and memory headroom. No second independent cross-model reviewer was run during the original implementation because the repository requires all subagents to use GPT-6 Luna and this environment provided no independent model configuration.

## Documentation release

**Status: complete for this implementation.** The README links to the user setup guide. Protocol, test plan, risk register, and implementation run record are maintained in `dev_reference/plans/`. The repository has no changelog. Pre-existing `.agents/` and root `todo.md` user files were left untouched; this feature's progress checklist is `wlan-logging-todo.md` at the repository root.

## Hardware gate

**Status: partial live acceptance on 30.09.2026.** The logging-enabled firmware was flashed to the connected ESP32-S3 through the CH340, and the upload helper verified the written hashes. Initial secured 20-second runs produced 4,905 and 4,931 records; a third run stopped by `SIGINT` saved 1,218 valid records with reason `USER_STOP`. Follow-up tests capped capture at 100 Hz and batched five records per sender wake. With CH5 off, the final batch run produced 1,846 records, zero CRC, sequence, or numeric faults, ring high-water 5, and median `control_dt` 2.032 ms versus a prior no-recording K58 median of 2.104 ms. A post-batching CH5-on run completed with 1,768 active records, zero transport or numeric faults, median `control_dt` 2.128 ms, and p95 2.948 ms. A same-boot CH5-on comparison later measured the K58 idle baseline at median/p95 2.122/2.762 ms and active Wi-Fi recording at 2.117/2.892 ms. The median was nearly unchanged, but the K58 baseline sampled at 20 Hz, the Wi-Fi capture at about 89 Hz, and the runs were 4.3 minutes apart. This estimates timing differences but does not prove behavior-neutral balancing. Heap/task-stack margin, deliberately stalled sender, and representative movement remain untested. See [the hardware observations](../../agent_notes/wireless-logger-hardware.md#wlan-aufnahmen-am-aufgebauten-roboter-am-30092026) for commands and evidence limits.
