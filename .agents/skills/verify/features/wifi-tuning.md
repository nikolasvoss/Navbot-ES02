# Tune parameters over Wi-Fi

The Wi-Fi terminal reads firmware and controller status, changes an allowlisted set of temporary tuning values, and saves or applies local profiles.

## Sub-features

- `tuning-status` reads firmware identity, RC freshness, and CH5 state.
- `parameter-read` reads one or all tuning values.
- `parameter-write` changes an allowed value while firmware safety gates permit it.
- `profile-save-load` saves a local profile and applies its values to the connected robot.

## How to get to it (user POV)

- Run `python3 scripts/wifi_tune.py` after the robot has joined the trusted WLAN.
- Run `python3 scripts/wifi_tune.py --command status --json` for one status query.
- Enter commands such as `PP`, `show`, `PP5`, `save profile.json`, or `load profile.json` in the terminal.
- Read the setup and safety steps in [`dev_reference/wifi-parameterterminal.md`](../../../../dev_reference/wifi-parameterterminal.md).

## Driving it with the Wi-Fi tuning CLI

Preconditions:

- The robot has the Wi-Fi tuning build enabled and is provisioned on a trusted development WLAN.
- A valid robot IPv4 address is available. Keep passwords out of arguments and evidence.
- Before any gain write, verify fresh SBUS data, CH5 off, and `U=1`.
- Use a disposable profile path under the run evidence directory.

- **Check status.** Run `python3 scripts/wifi_tune.py --host <robot-ip> --command status --json`. Exit code `0` and JSON show `boot_id`, `build_id`, `rc_valid`, `rc_age_ms`, `ch5_off`, and `tuning_mode`.
- **Read values.** Run `python3 scripts/wifi_tune.py --host <robot-ip> --command show --json`. The result contains the current values and confirms the terminal can read back the device state.
- **Write and read back.** With the documented preconditions true, run `python3 scripts/wifi_tune.py --host <robot-ip> --command PP5 --json`, then run `python3 scripts/wifi_tune.py --host <robot-ip> --command PP --json`. The second response must report `PP` as `5`. Restore the prior value and read it back before ending the drive.
- **Proof.** Save both write and read-back responses, the initial status, command lines with the real IP redacted, stderr, and exit codes. Do not capture credentials.

## Gotchas

- The HTTP API has no authentication or TLS. Use only a trusted development WLAN.
- Gain writes require `U=1`, fresh SBUS frames, and CH5 off. A rejected write is not a successful test.
- Parameter writes last only until the next reboot.
- If a POST times out, first read `status` or `show`. Do not repeat a write whose outcome is unknown.
- The local config file stores the robot address with mode `0600`; it does not store a password.
- A first `--host` run can create the local config file. Use a disposable `--config` path when isolating a verification run.
