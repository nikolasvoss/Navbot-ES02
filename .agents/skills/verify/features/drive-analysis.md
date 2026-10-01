# Analyze drive trials

The drive analysis command reads saved K58 logs and reports trace quality, RC activity, start tilt, and observed wheel-speed reversal after CH3 returns to neutral.

## Sub-features

- `analyze-one` summarizes one K58 drive trace.
- `compare-trials` compares multiple saved K58 logs.
- `require-stop` returns a failure when a trace lacks an interpretable stop event.

## How to get to it (user POV)

- Run `python3 scripts/analyze_drive_trace.py <K58-log>` after a drive capture.
- Pass multiple log paths to compare trials.
- Use `--format json` for a machine-readable report or `--output` to save it.

## Driving it with the offline analysis CLI

Preconditions:

- Python 3 is available.
- The input is a saved K58 log with a `# DRIVE` header. Use existing logs as read-only inputs.
- The destination directory exists and the output path is new.

- **Analyze one trial.** Run `python3 scripts/analyze_drive_trace.py logs/drive-yp4-diagnose-01.log --format json --output /tmp/navbot-verify/<run-id>/drive-analysis.json`. Exit code `0` and the JSON report include `valid_rows`, `malformed_rows`, `capture_ok`, `releases`, and `stop_available`.
- **Compare trials.** Run `python3 scripts/analyze_drive_trace.py logs/drive-yp4-diagnose-01.log logs/drive-stop-speedp005-01.log --output /tmp/navbot-verify/<run-id>/drive-comparison.md`. The Markdown report lists each log and explains whether its stop data is usable.
- **Require a stop event.** Add `--require-stop` when the caller needs an interpretable stop in every input. A log without one must return a nonzero exit status.
- **Proof.** Compare the report's row counts and stop events with the source logs. Save the exact command, output, and exit code beside the report.

## Gotchas

- This command reads files only. It does not open serial or change robot state.
- The stop metric describes logged wheel speed for up to one second after a CH3 release. It does not identify the cause or replace the operator's observation.
- Different starting tilt and command duration can make two trials unsuitable for direct comparison.
- Do not assume every file named `drive*.log` uses the current K58 column layout. Check its header first.
