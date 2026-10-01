# Capture balance and drive traces

The trace tools save serial telemetry from the running firmware so an operator can inspect balance, yaw, servo, and drive-stop behavior.

## Sub-features

- `trace-55` captures actuator and balance telemetry.
- `trace-56` captures angle and yaw controller values.
- `trace-57` captures balance PID terms.
- `trace-58` captures drive and stop telemetry for trial reports.

## How to get to it (user POV)

- Run `python3 scripts/capture_balance_trace.py` for a raw K55 balance trace.
- Select a trace mode with `--trace-mode 56`, `57`, or `58`.
- Run `python3 scripts/run_drive_trial.py` when you want a K58 trace and a report in one command.
- Follow the capture guidance in [`README.md`](../../../../README.md) for RC switch and channel use.

## Driving it with the serial capture scripts

Preconditions:

- Read [`agent_notes/usb-serial.md`](../../../../agent_notes/usb-serial.md) and confirm the firmware baud rate.
- Support the robot. Close SerialPlot and all other serial monitors.
- Use a new output filename under the run evidence directory.
- For K56 or K57, leave CH5 off for a baseline. For K55 or K58, a human operator must follow the README capture sequence. The agent must not move the RC controls.

- **Capture a baseline.** Run `python3 scripts/capture_balance_trace.py --port /dev/ttyUSB0 --baud-rate 115200 --trace-mode 56 --seconds 20 --output /tmp/navbot-verify/<run-id>/control.log`. The script prints `Saved` with the output path and the file contains a `# CTRL` header and rows.
- **Capture a balance trace.** Run the same command with `--trace-mode 57` and a different output path. The file contains a `# BAL` header and rows.
- **Capture a drive trial.** Run `python3 scripts/run_drive_trial.py --port /dev/ttyUSB0 --baud-rate 115200 --seconds 40 --output /tmp/navbot-verify/<run-id>/drive.log`. A human operator follows the prompt, then the command writes `drive.md` beside the raw log.
- **Proof.** Keep the raw trace, command output, stderr, exit code, and generated report. Check that the capture process exited and the evidence files remain after cleanup.

## Gotchas

- `capture_balance_trace.py` defaults to K55 and 2,000,000 baud. Pick the mode and baud that match the firmware.
- K58 and K55 recipes can involve motion. Do not run them without a human at the robot and a safe physical setup.
- The trace rows are commanded values and sensor data. They do not prove measured servo shaft position or user-perceived movement.
- The capture script sends `K0` at normal completion. If the drive fails, run Doctor before trying again and close only the session started by that drive.
- Use one new filename per attempt. Do not overwrite a previous trace.
