# Task state: FOC configuration cleanup

Goal: finish the first five FOC sketch TODOs and apply the user's follow-up cleanup across directly affected firmware files.

Active contracts: `work/first-five-ino-todos.scope.md` and `work/remove-torque-compensation.scope.md`. The original request and follow-up are preserved verbatim. The user explicitly authorized removing `SwitchUser`, moving and later removing the table-based `TorqueCompensation` feature, deleting both unsupported current-loop paths, and removing `SENSOR_SWITCH_*`. After an impact warning, the user chose to delete the dependent encoder diagnostics, direct torque/angle modes, and firmware torque-table sampling.

## Decisions

- Keep the documented I2C AS5600 setup and GPIO assignments.
- Fix both motors to velocity control and voltage torque control, which match the prior defaults.
- The table-based compensation setting was moved beside its ON/OFF definitions, then removed with the table feature after the user follow-up.
- Remove unsupported current sensing and its current-only controller setup.
- Initially preserved the two baked correction tables while removing their sampler. A later explicit user request superseded that choice; the disabled, unverified table feature and its data are now removed.
- Remove angle-only PID setup and the unused torque-sampling communication value.
- Preserve the two-wheel balance path, other communication modes, runtime `control_torque_compensation` command, current limit assignments, and pre-existing user edits.
- No tests will be added or run. Verify the default firmware build, stale-symbol searches, table-data identity, and final source diff. Do not flash or operate hardware.

## Progress

- Removed the fixed-board SPI encoder selector/path. The default firmware build passed after that first cleanup.
- Completed the how trace and architecture comparison. The native Luna cross-judge selected candidate 2 with the angle-loop cleanup from candidate 3. Gemini CLI timed out after 90 seconds and provided no review.
- Independent scope guard approved the user-amended four-file plan and the angle/sample-enum cleanup additions.
- Implemented the approved changes in `OllieFOCdrive.ino`, `OllieFOCdrive.h`, `SlotCalibration.h`, and `SlotCalibration.cpp` from an isolated copy. The baked table initializer text matched the pre-edit copy by SHA-256.
- Default firmware build passes: 741,928 bytes program storage (56%) and 39,648 bytes global RAM (12%).
- Stale-symbol search found no removed selectors, current-loop modes, user-mode selector, or sampler API in the firmware source and documentation.
- At the prior checkpoint, the diff was limited to the four planned source files plus this task-state record, and the baked tables were unchanged. A later user follow-up removed the tables.
- The previous cleanup checkpoint had two trailing-space findings on pre-existing TODO-edited `.ino` lines.
- Final independent scope check returned STATUS OK with no scope gaps.
- Follow-up scope guard approved the user's request to remove `TorqueCompensation` and `SlotCalibration`; deleted the setting, lookup path, and table source files while preserving the separate runtime wheel-target adjustment.
- Removed the table feature's setting, lookup path, and both `SlotCalibration` files. The separate runtime wheel-target adjustment remains.
- Clean default firmware build passes: 741,908 bytes program storage (56%) and 39,648 bytes global RAM (12%). The earlier incremental build used a zero-byte cached `Commander.cpp.o` left by an interrupted build; `--clean` rebuilt it successfully.
- Active-source search found no remaining table macro, array, sampler, slot-calibration file, or lookup references. The separate runtime `control_torque_compensation` command remains by explicit user choice.
- Final independent scope check for the removal follow-up returned STATUS OK with no scope gaps.

## Next step

No further task steps remain. The separate runtime `control_torque_compensation` adjustment remains.
