# Serial logging architecture

## Grounded flow

`setup()` starts `Serial` at the existing 115200 baud. In the diagnostic build, `loop()` calls `DiagnosticLoop()` and returns; that function snapshots the existing 24 sensor/status channels every 50 ms and currently formats/writes the numeric CSV row inline. In the normal build, `loop()` reaches `print_data()` from its 1 ms section before the later controller calculations. `print_data()` selects existing debug fields through `Select`; cases 1–45 write multi-part text rows directly, while cases 55–58 emit TRACE/CTRL/BAL/DRIVE rows at their existing intervals. The existing values and cadence must remain at those call boundaries.

The Wi-Fi concept's relevant requirements are fixed internal RAM, bounded snapshots, no producer wait, sender-side formatting, and visible incomplete state in firmware. This serial-only task excludes its transport, start/stop session model, binary protocol, and higher-rate acquisition. The current numeric CSV rows have no completion footer. A proposed emitted overflow marker was checked independently and rejected for changing the CSV stream; no complete state is introduced by the implementation.

## Chosen record shape

Use one tagged fixed-size `Record` with two variants:

- `Diagnostic` carries the existing `Frame` timestamp and 24 channel values.
- `SelectedDebug` carries the existing `Select` value, `millis()` timestamp for the 55–58 rows, a fixed array of float values, and a fixed array of integer values. Each selector assigns only the slots its existing format uses. The sender interprets the selector and emits the same labels, values, precision, order, and newline as the current output.

Keep every field inline. The record contains no heap-backed object, pointer to mutable firmware state, or variable-length payload. The largest existing row determines array sizes. Use a 32-record `xQueueCreateStatic()` queue with its control block and storage in `DRAM_ATTR` internal memory. The Arduino `loop()` context is the sole producer in both mutually exclusive firmware modes; one lower-priority task is the sole sender for queued log records. The producer calls `xQueueSend(..., 0)` so a full queue returns immediately. The sender receives each record by value before formatting or calling `Print::write`, freeing that queue slot before any potentially slow output.

On the first full enqueue, latch `BufferFull`; reject all later records and keep a small rejected-record count. Keep status separate from the queue so it survives overflow. Provide no `Complete` state and no emitted marker. Queue or sender-task startup failure and sender-write failure also latch an incomplete reason. Logging failures do not write motor/safety state.

The consumer formats one complete record into a fixed 512-byte local buffer and sends it with one write. One sender owns all asynchronous diagnostic and selected-debug output. Existing setup messages and the SimpleFOC Commander remain on the same `Serial`; no producer mutex will couple the control loop to a blocked sender. This does not claim that unrelated pre-existing command replies cannot interleave with a log row.

## Verification shape

Compile `Telemetry.cpp` in a host test with Arduino and FreeRTOS API stubs, defining `ARDUINO_ARCH_ESP32` so the target queue path is exercised. A fake sink blocks inside `write()` while a consumer thread calls the real send function. The producer must enqueue up to queue capacity and return while that write is blocked. Verify FIFO output, full/empty boundaries, reuse after consumption, wraparound, and sticky incomplete status with rejected count. Compare representative diagnostic, selected-debug, and TRACE/CTRL/BAL/DRIVE rows to the existing formats.

## Design review

The first implementation used a hand-written SPSC ring. After review, the user explicitly selected the platform-provided statically allocated FreeRTOS queue to reduce hand-written synchronization while preserving bounded zero-wait enqueue behavior. The host test continues to exercise blocked output, exact output formatting, FIFO order, wraparound/reuse, and overflow. Firmware builds verify the actual ESP32 FreeRTOS API integration.

The Gemini review adapter was called once alongside the architecture candidates but timed out after 90 seconds and returned `ok: false`; no Gemini finding was accepted.
