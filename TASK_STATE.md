# Task state: commit firmware build versions

Goal: keep the versions used by a successful firmware build in a file that can be committed with the source after the user confirms that the robot balances.

## Decisions

- Generate `firmware-build-environment.json` in the repository root after a successful build.
- Record the FQBN, resolved ESP32 core version, and libraries selected by the compiler.
- Keep the file observational. Do not install, download, or enforce dependency versions.
- Do not stage or commit the file from the build script. Git history associates it with source changes when the user commits both after a balance check.
- Preserve the existing motor 2 estimated-current change in the sketch.

## Progress

- The user confirmed balancing worked with ESP32 core 3.3.12, Simple FOC 2.4.0, ArduinoJson 7.4.3, and Queue 2.1.
- `scripts/build_firmware.py` now writes the tracked root JSON after a successful compile and omits timestamp and source revision fields.
- README and firmware build design docs describe the generated file and when to commit it.
- `python3 scripts/build_firmware.py` succeeded. The JSON contains the expected versions and FQBN.
- `python3 -m py_compile scripts/build_firmware.py` and `git diff --check` passed.
- No packages were downloaded or installed. No hardware flash was run.
- The final independent scope check returned STATUS OK.

## Next step

Commit `firmware-build-environment.json` with the firmware source after confirming that the robot balances.
