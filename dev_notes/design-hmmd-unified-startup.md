Usage:
CM5: python3 scripts/start_hmmd.py
PC: python3 scripts/start_hmmd.py --ssh niko@192.168.178.28
Headless: add --no-browser. Alternate target checkout: --workspace /absolute/path.

Shape: one options record with optional SSH destination, target-side workspace, verified device, positive baud/start timeout, browser preference. One STARTUP_BASH transaction owns all three services. Python owns local/SSH dispatch, both loopback forwards and browser/tunnel lifecycle. No separate local/remote policy implementations.
Signatures: parse_args(argv) -> options; startup_command(options) -> argv; run_startup(options) -> exit code; run_tunnel(options) -> exit code; main(argv=None) -> exit code. Use a small concrete record or argparse namespace, not a transport/framework abstraction.
Synthesis: candidate A base; C strict destination validation and two-port preflight; B full Python rewrite rejected for unnecessary risk/change size. Explicit --ssh replaces host/user; no automatic role guessing. Remote services use existing deployed ROS workspace, but execute the caller startup payload. No remote launcher-file requirement. Both modes use localhost HTTP URL. Remote HTTP+WS go through SSH.
Tradeoffs: retain existing Bash transaction inside Python to minimize changes to proven identity/UART logic. Services continue after caller exits. Jazzy stays the configured ROS distribution. No systemd/stop mode/new dependencies.
Implementation reconciliation: pending implementation. Accepted deviations must update this same note.
