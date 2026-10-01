# Record balance telemetry over Wi-Fi

The Wi-Fi recorder saves a framed `balance_v1` stream as an `.nblog` file, then the decoder validates it and exports CSV and a JSON quality report.

## Sub-features

- `record-status` reads the recorder state.
- `record-full` captures a 20 to 40 second stream and validates the END frame.
- `decode-recording` exports CSV and a quality report from a complete recording.
- `decode-partial` exports the valid prefix of an incomplete file while marking it partial.

## How to get to it (user POV)

- Run `python3 scripts/wifi_record.py --status` to inspect recorder readiness.
- Run `python3 scripts/wifi_record.py --seconds 30 --output <new-file>.nblog` to capture.
- Run `python3 scripts/decode_wifi_trace.py <file>.nblog --csv <file>.csv --report <file>.json` to inspect the data.
- Follow the build and network requirements in [`dev_reference/wifi-recording.md`](../../../../dev_reference/wifi-recording.md).

## Driving it with the Wi-Fi recording CLI

Preconditions:

- The robot runs a build with both `WIFI_TUNING_ENABLE` and `WIFI_RECORDING_ENABLE` enabled.
- The robot is provisioned and reachable on the trusted development WLAN.
- `~/.config/navbot/wifi-tuning.json` has mode `0600` and contains the robot's IPv4 address.
- The robot is supported. The recorder does not command motion, but it samples the balance path.
- The run output directory exists and the `.nblog`, `.partial`, and `.error.json` paths do not exist.

- **Check readiness.** Run `python3 scripts/wifi_record.py --status`. Exit code `0` and a JSON status response show that the client reached the recorder API.
- **Record.** Run `python3 scripts/wifi_record.py --seconds 30 --output /tmp/navbot-verify/<run-id>/trial.nblog`. A complete capture prints `Recording saved and verified` and exits `0`.
- **Decode.** Run `python3 scripts/decode_wifi_trace.py /tmp/navbot-verify/<run-id>/trial.nblog --csv /tmp/navbot-verify/<run-id>/trial.csv --report /tmp/navbot-verify/<run-id>/trial-summary.json`. The report must show a complete recording with zero CRC, sequence, and numeric faults.
- **Proof.** Keep the `.nblog`, CSV, JSON report, commands, output, and exit codes. Check the report and files after cleanup.

## Gotchas

- Recording requires a 20 to 40 second duration. A user-stopped file is valid but shorter than requested and exits `5`.
- The client refuses to replace existing output or partial files unless `--force` is passed. Do not use `--force` during verification.
- Failed transfers retain a `.partial` file and write an `.error.json` report. Preserve both as evidence.
- The HTTP service has no authentication or TLS. Keep it on a trusted development WLAN.
- A complete transport does not prove performance during movement or establish safe balance behavior.
