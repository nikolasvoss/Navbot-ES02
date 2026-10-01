# Navbot-ES02 verification map

Read this index before driving a feature. The map covers the robot's USB status and trace tools, offline drive-log analysis, and optional Wi-Fi clients.

## Baseline preconditions

- Use a fresh output directory under `/tmp/navbot-verify/<run-id>/`.
- Treat files under `logs/` as source evidence. Never overwrite or delete them.
- For hardware runs, read [`agent_notes/usb-serial.md`](../../../../agent_notes/usb-serial.md) and use the configured serial MCP server for device discovery and direct serial access.
- Support the robot before connecting to or reading its live firmware. Do not send drive commands.
- Wi-Fi features require an already provisioned robot with the matching optional build flags enabled.

## Driving conventions

- Start a fresh process for each short-lived CLI drive.
- Run the feature's read-only status path before a Wi-Fi mutation or recording.
- Use stable source-log paths for offline analysis and unique output paths for generated files.
- Capture the command, stdout, stderr, exit code, and resulting file in the run's evidence directory.
- Do not report a hardware or Wi-Fi route as verified when its device or network prerequisite is missing.

## Features

- [Read RC status over USB](./usb-status.md) covers a one-shot SBUS snapshot through `remote_status.py`.
- [Capture balance and drive traces](./trace-capture.md) covers K55, K56, K57, and K58 capture paths.
- [Analyze drive trials](./drive-analysis.md) covers saved K58 logs and comparison reports.
- [Tune parameters over Wi-Fi](./wifi-tuning.md) covers status, parameter reads, profiles, and guarded writes.
- [Record balance telemetry over Wi-Fi](./wifi-recording.md) covers capture, validation, and CSV/report export.
