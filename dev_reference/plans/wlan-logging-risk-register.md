# WLAN logging risk register

| Risk | Impact | Mitigation and evidence required | Status |
|---|---|---|---|
| Producer stalls or allocates in the balance loop | Control jitter or missed deadlines | Fixed-size preallocated ring and bounded record encoding; measured loop overhead and timing still require hardware | Host source/build checked; hardware open |
| Sender races ring reuse or publishes incomplete records | Corrupt or silently missing samples | SPSC acquire/release ring and full-capacity host checks; repeated live-session ownership remains untested | Host subset passed; live lifecycle open |
| Wi-Fi service contention | Tuning/status inaccessible or recording starves | Reuse the opt-in station service, separate data socket ownership, bounded writes | Build/source checked; live contention open |
| RAM pressure on ESP32-S3 without PSRAM | Startup failure, reset, degraded control | Allocate fixed 64-KiB ring once and fail prepare if unavailable; runtime headroom/stack measurements remain open | Build passed; runtime memory open |
| Parameter changes during capture | Mixed configuration within one recording | Network and serial mutation gate plus fixed configuration comparison at capture boundary | Source/build checked; runtime drift test open |
| TCP truncation or disk failure hidden as success | Invalid experiment presented as complete | Exact frame validation, retained `.partial`, fsync/rename before ACK, explicit error report | Host subset passed; disk-full/live fault tests open |
| Unauthenticated control API | Any client on the WLAN can invoke tuning and recording controls | Accepted for development; compile the Wi-Fi feature out of normal firmware and do not port-forward it | Build switch and docs checked; no live WLAN test |
| Insecure WLAN transport mistaken for confidentiality | Control requests or samples can be observed or changed by WLAN peers | HTTP/TCP remain plaintext; the recording stream uses a one-use session ticket only for association, not encryption | Documented development-tool limit; restrict use to a trusted development WLAN |
| Build cannot be tied to a source revision from recording metadata | A trace may be harder to reproduce from its metadata alone | Report the build ID and state explicitly that no source hash is available | Documented limitation; add revision metadata in a future build pipeline |
| Feature-on binary exceeds partition or consumes excess heap | Feature cannot be deployed or destabilizes control | Installed-version feature build passed; runtime heap/task-stack measurements remain open | Build passed; runtime resource margin open |
| Hardware acceptance inferred from host tests | Unsafe claim of real-time performance | Label timing/throughput/driving acceptance not executed; do not flash in this turn | Not executed by design |
