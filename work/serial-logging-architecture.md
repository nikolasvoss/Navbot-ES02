# Serial logging merge architecture

## Target behavior retained

The target branch already owns the CH340 `Serial` stream at 576000 baud, a bounded `SerialLogger`, and capture scripts for diagnostic and TRACE/CTRL/BAL/DRIVE rows. Diagnostic mode samples every 10 ms and emits 17 numeric fields. Selected TRACE/CTRL/BAL/DRIVE rows are produced at the existing every-seventh-control-gate boundary. Those formats and rates remain unchanged.

Normal control invokes `print_data()` from the main loop. Selectors 1–45 previously formatted and wrote their values inline. They now copy those already available values into a fixed selected-debug record at the same call boundary. Selector 8 keeps its existing 50 ms throttle. Selectors 55–58 continue through the target's typed row builders and retain the existing CSV suffix fields consumed by capture and summary tools.

## Single queue and sender

The target `SerialLogger` is the only asynchronous logger. Its fixed record union holds either an existing target row or a selected-debug payload. One statically allocated `xQueueCreateStatic()` queue has 32 records; its control block and byte storage are marked `DRAM_ATTR`. On the ESP32, a `SerialLogRecord` is 168 bytes and the queue reserves 5,376 bytes of internal RAM.

The producer snapshots fields into the fixed record and calls `xQueueSend(..., 0)`. It does not format text, allocate memory, write to Serial, or wait for the queue. The logger sender receives a record before formatting it in a 384-byte stack buffer and writing it to Serial, so a blocked UART holds neither the producer nor a queue slot.

On first queue overflow, sender startup failure, or sender write failure, a small sticky status records the reason. Subsequent submissions are rejected and a saturating 8-bit rejected count is retained. The stream has no completion marker or successful-completion state. Queue and sender status do not change motor enables, balance calculations, or fall/safety decisions.

Selected debug output is the only new record kind; it uses the existing SerialLogger sender. The mode gate suppresses other existing BLE/robot debug prints while a selected log mode is active, avoiding another concurrent debug writer. Existing Commander and setup output are not serialized with these records and may still interleave as before.

## Verification

`scripts/test_serial_logger.py` compiles the actual formatter and sender against host FreeRTOS/UART stubs. It blocks the sender inside the UART write, fills the 32-slot queue, times the rejecting producer call, releases the writer, and checks FIFO output, reuse, overflow status, and the target CSV column counts. Normal and diagnostic firmware builds verify the target FreeRTOS integration. Hardware timing and CH340 receive behavior are not measured by these checks.
