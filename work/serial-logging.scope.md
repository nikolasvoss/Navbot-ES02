# Scope contract: serial logging merge

## User outcome

Merge bounded serial logging commit `5aa6f78cad350b6058b18ef35d7841934db1a0cb` into `codex/sensor-data-roadmap` so a slow or non-reading PC cannot hold the regulator's selected logging path.

## Baselines

- Target branch baseline: `f4e16591aaed9f4354f32242f9265d729bc33685`.
- Source commit baseline: `a9c2131f5a8bce64755541241d8f7cd793e1601e`.
- The branches diverged after the source baseline. The target branch already has `SerialLogger`, updated CSV contracts, and capture tools.

## Acceptance criteria

- Producer snapshots existing values at existing diagnostic/selected trace boundaries.
- No CSV formatting, serial writes, dynamic allocation, additional sensor reads, or unbounded waits in the logging producer.
- A bounded preallocated internal-RAM buffer feeds one independent sender.
- Full queue returns immediately, makes the recording sticky-incomplete, and rejects later records without changing drive, fall, motor-enable, or safety logic.
- Tests cover a blocked sender, queue bounds, FIFO order, reuse, and sticky incomplete state.
- Normal and diagnostic firmware build.
- Existing CH340 transport, target baud, row formats, rates, and selected channels remain unchanged. No hardware drive or flash.

## Merge-specific integration amendment

The source task's frozen design named `Telemetry.h/.cpp`, but the target branch already has a single asynchronous `SerialLogger` whose CSV format is consumed by target capture tools. Adding the source `Telemetry` queue as a second logger would violate the one-writer/one-FIFO requirement and change accepted target formats. For this merge, port the source selected-debug snapshots into the target `SerialLogger` and keep it as the sole asynchronous logging queue and sender. This is a module-placement adaptation for the explicitly requested merge; it does not add a transport, format family, or product subsystem.

Keep `TRACE`, `CTRL`, `BAL`, `DRIVE`, and diagnostic numeric rows byte-compatible with the target branch. Keep target baud 576000, diagnostic interval 10 ms, and existing trace gate/rates. Queue capacity is 32 records in internal RAM. On overflow or sender failure, retain only a sticky incomplete reason and small rejected count; do not emit an incomplete marker or a success state, consistent with the approved source-scope decision.

## In scope

- `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`
- `src/ES-02/OllieFOCdrive/SerialLogger.h/.cpp` and `SerialLogFormat.h`
- The existing SerialLogger host harness under `scripts/serial_logger_stubs/` and `scripts/test_serial_logger.cpp`, `scripts/test_serial_logger.py`
- This task's state, architecture, and scope notes under `TASK_STATE.md` and `work/`
- Existing BLE/robot debug-output gate references needed to use the one logger mode flag

## Explicit exclusions

No Wi-Fi service, native USB, FFT UI, new protocol family, channel selection, rate increase, baud increase, hardware drive, firmware upload, or balance/controller tuning. Do not alter motor, balance, fall, or safety decisions. Do not claim unchanged balance timing quality without a robot measurement.
