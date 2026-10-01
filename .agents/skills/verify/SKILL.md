---
name: verify
description: Verify Navbot-ES02 firmware diagnostics and operator tools through USB serial, saved trace files, and optional Wi-Fi clients. Use before changing balance, receiver, tuning, or telemetry behavior.
---

# Verify Navbot-ES02

## Launch

The primary app is firmware on the ESP32-S3 robot. The repository also provides short-lived Python command-line tools for status, trace capture, tuning, and analysis. There is no local server to keep running.

For the offline analysis feature, create a new evidence directory and start a fresh process with this command:

```bash
python3 scripts/analyze_drive_trace.py logs/drive-yp4-diagnose-01.log --format json --output /tmp/navbot-verify/<run-id>/drive-analysis.json
```

For a hardware feature, power on the robot with its existing firmware. Do not compile or upload firmware as part of a routine verification run. A flash can change motor behavior.

Before any physical run, place the robot on a stable support that keeps the wheels and legs clear of the work surface. Leave the RC switches and sticks in their safe positions unless the feature recipe explicitly requires a human operator to move them. Do not send drive commands from the agent.

## Doctor

For an offline trace analysis, check the Python runtime and source log before each fresh command:

```bash
python3 --version
test -r logs/drive-yp4-diagnose-01.log
```

For a USB feature, use the configured serial MCP server. Call `list_ports` and confirm that the intended CH340 device is present. Do not assume `/dev/ttyUSB0` is stable. If you open it through MCP, set RTS and DTR low immediately, then read the startup output before sending a command. A serial open can reset the ESP32.

For a Wi-Fi feature, run its read-only `status` command first. Require a valid JSON response with `ok: true` and a current `boot_id`. The API has no authentication and uses plaintext HTTP, so only use it on the trusted development WLAN.

If a drive fails, run Doctor again before retrying. If a process is healthy but its UI or device state is stuck, close only the process or connection started for that drive, then establish a known state before continuing.

## Drive

Read [the feature map](./features/README.md) before each run. It lists every supported entry point, exact command, required device state, and visible result.

Run each CLI command in its own shell process. Use a new evidence directory for every run. Prefer a read-only status or saved-log analysis when those cover the change. Never treat a unit test or direct internal call as proof of a user path.

Use the project scripts for their documented workflows. For ad-hoc serial discovery and reads, use the configured serial MCP server. Do not open the serial device from a second process while another tool owns it.

## Evidence

Create a unique directory under `/tmp/navbot-verify/`, such as `/tmp/navbot-verify/20260930-run-01/`. Save the exact command, stdout, stderr, exit code, and generated report there. For trace capture, retain the raw trace and its summary. For Wi-Fi recording, retain the `.nblog`, decoded CSV, and JSON report.

Exercise the real user command and inspect its resulting state. A drive report must agree with the raw trace. A Wi-Fi recording counts as complete only when its END frame, record count, sequence, and checksums validate. Never include a WLAN password or other credential in evidence.

## Cleanup

Wait for every command started for the drive to exit. The capture script sends `K0` at normal completion; if a serial MCP session was used, close that connection. Stop only a recording session started by this run. Do not delete or overwrite source traces.

Cleanup may remove temporary files created outside the evidence directory. Keep all evidence under the named `/tmp/navbot-verify/<run-id>/` directory, and check that it still exists after cleanup.

## Helpers

No verification helper scripts are required. The project commands are documented in the feature files.
