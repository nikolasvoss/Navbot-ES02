# WLAN-Provisioning implementation checklist

- [x] `how` over the affected subsystem.
- [x] `architect` for parallel design exploration.
- [x] **Blocking first steps.** Confirm feature branch and existing architecture without opening or searching secrets.
- [x] **Independent workstreams.** Design candidates and review can be independent; implementation and build verification follow the chosen design.
- [x] **Shared mutable state.** Keep implementation on the existing feature branch; design and review workers are read-only, so they share no writable files.
- [x] **Smallest safe decomposition.** One implementation owner is best because provisioning lifecycle, tuning server lifecycle, reset behavior, and documentation share state and sequencing.
- [x] Implement no-credentials provisioning, saved-credential reconnect, explicit reset, runtime-only tuning token storage boundary, and Linux instructions.
- [x] Compile firmware without real credentials and inspect the resulting diff.
- [x] Address review findings: add the Linux Python dependencies and make the boot message neutral about whether the SoftAP is active.
- [x] Rebuild with Core Debug Level `None`; the final independent review found no remaining concrete issue.
- [ ] Commit and push the reviewed feature branch.

Design pick: candidate C, confirmed by the cross-judge. It keeps provisioning inside the existing Wi-Fi tuning owner, uses open SoftAP provisioning for development, and keeps the tuning API fail-closed until a runtime NVS token exists. Candidate A's unauthenticated tuning API was rejected because the user's simplicity preference applied to provisioning protection. Candidate B's separate provisioning module was rejected as an unnecessary new boundary.

Build result: ESP32-S3 compiled with Arduino-ESP32 3.3.11, `DebugLevel=none`, and `PartitionScheme=no_fs`. Program size was 1,355,308 bytes in a 2,031,616-byte app partition. No credential values were used. Old build artifacts were not inspected; an attempt to delete multiple pre-existing output directories was rejected by the automatic approval review as too broad.
