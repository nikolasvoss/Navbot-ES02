Original Outcome (verbatim, user request 2026-10-07):
"Entkopple die Erfassung von der Formatierung und Übertragung. Ein langsamer oder nicht lesender PC darf den Regelzyklus nicht aufhalten."

Acceptance Criteria:
- Producer enqueues consistent snapshots at the existing diagnostic and selected trace cycle boundaries using only current values.
- Producer performs no CSV formatting, serial writes, dynamic allocation, extra sensor reads, or unbounded waiting.
- A fixed internal-RAM buffer feeds one lower-priority serial sender. A full buffer never waits, marks recording incomplete, and stops accepting samples without affecting drive/fall logic.
- Tests prove a blocked sender does not block enqueue, and prove full/empty bounds, FIFO ordering, reuse, and incomplete status after overflow.
- Diagnostic and normal firmware builds succeed. Existing transport, baud, rates, channels, motor enables, balance calculations, and safety logic remain unchanged.
- No robot drive or firmware transfer is performed. Hardware timing remains unverified.

Approved Approach:
- Extend the existing Telemetry module with a fixed-size single-producer/single-consumer queue of typed, fixed-shape records and one sender loop that formats all supported record kinds.
- Replace direct writes in the existing diagnostic CSV path and existing TRACE/CTRL/BAL/DRIVE trace cases with snapshot enqueue at their current boundaries. Preserve current formats and cadence.
- Start one low-priority sender task on the existing Serial/CH340 stream. Keep buffer statically allocated in internal RAM.
- Add a host-side deterministic test with a sink blocked in its write call.

Explicit Non-Goals:
- Wi-Fi services, native USB, FFT UI, new protocol family, recorder platform, new log rates, channel selection, 921600 baud, hardware driving, and firmware transfer.

Compatibility / Migration Requirements: NONE

Task baseline: a9c2131f5a8bce64755541241d8f7cd793e1601e (completed filter precision correction, with existing Telemetry module); isolated worktree created from this commit. The original checkout is on main at 145c5e5 with pre-existing uncommitted changes in docs/robot/hardware/electronics/reference.md, docs/robot/hardware/system-overview.md, src/ES-02/OllieFOCdrive/OllieFOCdrive.ino, agent_notes/robot/hardware-findings/native-usb-cdc.md, scripts/flash_mcp.py, scripts/flash_mcp_requirements.txt, and .worktrees/; those changes are outside this worktree and must remain untouched.

Frozen implementation plan:
1. Implement a bounded typed SPSC record queue, minimal overflow status/counter, and sender-side formatting in src/ES-02/OllieFOCdrive/Telemetry.h and Telemetry.cpp.
2. Start exactly one low-priority sender and replace current diagnostic and TRACE/CTRL/BAL/DRIVE direct logging writes in src/ES-02/OllieFOCdrive/OllieFOCdrive.ino with enqueue snapshots, preserving formats, values, and cadence.
3. Add a host test under tests/serial_telemetry/ that blocks the sender sink and verifies producer progress, queue bounds, ordering, reuse, and incomplete status.
4. Build normal and diagnostic firmware, inspect the task diff for unchanged control/safety decisions, and run the independent final scope check.

Expected change area and extent: the two Telemetry module files, the main firmware sketch, one focused host test and its minimal stub/build helper if required, plus this local work note. No other product files are expected. Keep implementation compact and avoid unrelated cleanup.

Plan amendment proposal (approved by independent scope check, STATUS: OK):
- In step 2, include all existing `print_data()` selections that write diagnostics directly to Serial, not only cases 55–58. Preserve the current `Select` behavior, formats, and cadence. Evidence: `OllieFOCdrive.ino` documents `print_data()` as the selected debug output path, and cases 1–58 contain synchronous Serial calls while normal `loop()` calls `print_data()` in the 1 ms control section. This is proposed as necessary for the acceptance criterion forbidding serial logging writes in the regulation path. No new channel selection or rate is introduced.
- Update expected extent within the same three production files plus host test; no additional product subsystem or protocol.

Gate evidence: fresh independent Luna scope reviewer returned STATUS: OK. It cited `OllieFOCdrive.ino:1390` and `2483–2542` and found the selected `print_data()` outputs directly required by the no-serial-write acceptance criterion. The original frozen plan above remains unchanged; this appended amendment is now active.

Rejected amendment record: an independent scope check returned STATUS: DRIFT for emitting a `LOG,INCOMPLETE` line because it adds a non-numeric record to the existing CSV stream and is not required by the accepted contract. No such stream-format change is active. The firmware will expose a sticky incomplete status and will not expose a successful-completion state.

Implementation amendment (explicitly approved by the user 2026-10-07): replace the hand-written ESP32 SPSC index synchronization with the platform's statically allocated FreeRTOS queue (`xQueueCreateStatic`). Send fixed-size `Record` items by value using zero wait; receive from the independent sender. Keep the host harness and producer/formatter API behavior, sticky overflow status, fixed depth, output formats, and all original scope limits. The queue's control block and storage must reside in internal DRAM. Re-run blocked-sender, capacity, overflow, FIFO/reuse checks, normal and diagnostic firmware builds, diff review, and final scope gate.
