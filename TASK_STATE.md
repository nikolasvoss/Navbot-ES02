# Task state: bounded serial logging

Goal: decouple existing diagnostic and balance trace capture from CSV formatting and CH340 serial transmission so a blocked host cannot stall the producer/control cycle.

Baseline: `a9c2131f5a8bce64755541241d8f7cd793e1601e` in the managed worktree `/home/niko/.codex/worktrees/serial-telemetry-buffer/Navbot-ES02`. This includes the completed filter precision correction and existing `Telemetry.h/.cpp`. Original checkout has unrelated local edits that remain untouched.

Decision: preserve current channels, trace formats, and cadence. Use a statically allocated FreeRTOS queue in internal RAM and one lower-priority sender for existing diagnostic and TRACE/CTRL/BAL/DRIVE rows. Overflow ends the capture as incomplete; it never changes motion or safety state.

Progress:
- [x] Inspect actual branch, worktree baseline, telemetry module, diagnostic loop, trace modes, serial transport, and Wi-Fi logging constraints.
- [x] Complete pre-edit scope check and architecture design.
- [x] Implement producer queue and sender integration.
- [x] Prove blocked-sink behavior, bounds, order, reuse, and overflow state with the focused host test.
- [x] Build diagnostic and normal firmware.
- [x] Review complete diff and pass final scope check (`STATUS: OK`).

Known limitations: robot timing quality under motor load is unmeasured. A four-second pause in the host reader was followed by continued output, but this does not prove the MCU queue filled because the CH340/OS may have buffered data. Commander and other pre-existing non-logging Serial output remain possible sources of interleaving or blocking outside the queued logger.

Build evidence after switching to `xQueueCreateStatic`: normal firmware succeeded (739,800 bytes flash; 43,928 bytes globals) and diagnostic firmware succeeded with `SENSOR_DIAGNOSTIC_MODE=1` (410,598 bytes flash; 36,416 bytes globals). The source macro has been restored to 0. Host blocked-sink/bounds/FIFO/reuse/format checks compile the ESP32 path against a static FreeRTOS API test stub and pass; `git diff --check` passes.

Review: the implementation diff changes only setup sender startup, the existing selected log producer, and the diagnostic enqueue call; control, motor, balance, and safety calculations are unchanged. The independent final scope check after the approved FreeRTOS queue change returned `STATUS: OK`. Existing unrelated Serial users (including Commander and occasional status output) are not routed through the telemetry sender and can still interleave or block in their own call sites.

Hardware follow-up (authorized in later user turns): diagnostic firmware flashed and verified; it booted with `imu_ok=1,motors=off,servos=off` and emitted 25-field rows at about 50 ms. The normal build was then flashed for a balance trace. K57 produced 20 ms `BAL` rows; mode 1 and motor targets around -23.4 were observed even after the user reported CH5 off. The user physically disconnected motor/battery power. USB was reconnected without motor power, but the CH340 was absent from the serial port list, so the diagnostic image could not be restored. The normal firmware remains in flash; keep motor power off until the port is available and the diagnostic image is restored. No motor-load timing quality was measured.
