# Risk register: buffered Wi-Fi logging

Planning status **clean** for specified scope; runtime/hardware evidence pending. Risk tier **high**. Required downstream `security-review`, `cli-qa`, `review`, `document-release`. [Main brief](implementation-brief.md) resolves product requirements and implementation defaults; no additional user answers are prerequisites for host implementation.

| Risk / evidence | Impact | Required mitigation |
|---|---|---|
| Dirty checkout; tuning plan exists but code absent | Lost current controls or duplicated service | Snapshot baseline; inspect and select explicit reuse/minimal-transport path |
| Synchronous sender or unbounded locks | Balance timing disturbed | No network/alloc/format in producer, bounded SPSC ownership, stalled-sender test |
| Ring index/data races across cores | Silent corrupt records | Correct publication/ownership, wrap/race tests, CRC and sequences |
| Sending faster than link/receiver | Growing heap or lost samples | Fixed capacity, high-water reporting, explicit BUFFER_FULL, no rate reduction |
| Socket accepts data before PC saves it | False success | END verification, fsync/rename, separate application ACK status |
| Reconnect lacks already-sent samples | Fabricated complete run | No replay promise, fail same session, partial file retained |
| Clock/snapshot mismatch | Wrong vibration inference | MCU monotonic timestamps, end-of-control snapshot, actual dt/IMU-read-age semantics |
| Inactive controller fields stale | False PID diagnosis | Explicit validity flag and NaN runtime fields |
| Auto or serial gains change mid-run | Misleading “constant configuration” result | Manual mode requirement, mutation gate, change detector before sample |
| Existing traces/error prints/flash writes | Hidden timing perturbation | Session trace suppression and counters; calibration gate; restore afterward |
| Wi-Fi service or BLE affects RC | Unexpected motion/input source | Independent session control, feature-specific BLE policy, no motor commands |
| Oversized frames/auth abuse | Memory exhaustion or data exposure | Length caps before allocation, finite leases, one client, token/ticket checks |
| Credentials committed/logged | Unauthorized home-network access | Ignored local secrets, nonsecret fixtures, no token CLI/URL/debug |
| HTTP/TCP unencrypted | Network observer can see traffic | Explicit trusted-home-network scope, no claims of TLS confidentiality |
| Nominal kHz mistaken for real sample rate | Invalid spectral conclusions | Native tick rate only, measured timing, no FIFO/FFT claims in v1 |
| Hardware test unavailable | Unsupported performance claims | Finish host/build work; mark all physical acceptance unexecuted |

Failure handling must leave existing driving/fall logic unchanged, free resources deterministically, and report incomplete measurements honestly. No automatic flash, driving trial or safety-limit adjustment is authorized by this plan.
