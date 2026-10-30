// Every value the link can log, set or call. One line per entry; this file is the only place that names them:
//   SIGNAL(name, unit, expression)        read-only. SIGNAL_MIN and SIGNAL_MAX report the lowest or highest value over
//                                         the control ticks since the previous row of the same subscription.
//   PARAM(name, unit, group, field)       a field of a settings group (settings_groups.h and the group types in
//                                         OllieFOCdrive.ino). Read, streamed and set through the group's apply function.
//   PARAM_LIVE(name, unit, group, field, step)  a PARAM that may also change while balance is requested, by at most
//                                         step in absolute value (0 refuses every change, INFINITY allows any change).
//   ACTION(name, handler, ARG(...), ...)  starts a handler with bounded arguments. ACTION_NO_ARGS(name, handler) has none.
// The expression of a SIGNAL reads existing state each time a subscription samples it.
//
// The wire ID of an entry is its position in this file, so it shifts when a line is added or removed; the host looks
// names up in the catalog of each connection. A name is at most 24 characters, a unit at most 16.
//
// The file ends in .h so that arduino-cli copies it into the build. It has no include guard: it is expanded several
// times with different macro definitions.

// Receiver and battery
SIGNAL    (ch5_position,          "",      ch5Position)
SIGNAL    (rc_valid,              "",      rcValid)
SIGNAL_MIN(voltage_min_raw_v,     "V",     batteryInstantRawV)
SIGNAL    (voltage_filtered_v,    "V",     batteryVoltage)
SIGNAL    (battery_raw_instant_v, "V",     batteryInstantRawV)
SIGNAL    (ch3_target,            "m/s",   movementSpeed)
SIGNAL    (body_turn_command,     "rad/s", bodyTurn)

// Link control: the balance enable and Command of CH5 position 1. cmd_age_ms is UINT32_MAX before the first Command.
SIGNAL    (link_balance,          "",      linkBalance)
SIGNAL    (cmd_speed_mps,         "m/s",   linkCommand.speedMps)
SIGNAL    (cmd_turn_rad_s,        "rad/s", linkCommand.turnRadPerSec)
SIGNAL    (cmd_age_ms,            "ms",    linkCommandAgeMs())

// Attitude and gyro
SIGNAL    (roll_deg,              "deg",   imu::latest().correctedRollDegrees)
SIGNAL    (pitch_deg,             "deg",   imu::latest().correctedPitchDegrees)
SIGNAL    (gyro_z_rad_s,          "rad/s", imu::latest().angularVelocityRadPerSec.z)

// Legs: the four servos get the same angle, so the leg_pose window is the servo range of each row
SIGNAL    (servo1_cmd_deg,        "deg",   servoTraceAngle[0])
SIGNAL    (servo2_cmd_deg,        "deg",   servoTraceAngle[1])
SIGNAL    (servo3_cmd_deg,        "deg",   servoTraceAngle[2])
SIGNAL    (servo4_cmd_deg,        "deg",   servoTraceAngle[3])
SIGNAL_MIN(leg_pose_min_deg,      "deg",   legPoseDegrees)
SIGNAL_MAX(leg_pose_max_deg,      "deg",   legPoseDegrees)

// Wheel motors: motor1 is the left wheel, motor2 the right (motor-local sign, see WheelBoundary.h)
SIGNAL    (motor1_target,         "A",     motor1.target)
SIGNAL    (motor2_target,         "A",     motor2.target)
SIGNAL    (motor1_current_sp,     "A",     motor1.current_sp)
SIGNAL    (motor2_current_sp,     "A",     motor2.current_sp)
SIGNAL    (motor1_voltage_q,      "V",     motor1.voltage.q)
SIGNAL    (motor2_voltage_q,      "V",     motor2.voltage.q)
SIGNAL    (motor1_shaft_velocity, "rad/s", motor1.shaft_velocity)
SIGNAL    (motor2_shaft_velocity, "rad/s", motor2.shaft_velocity)
SIGNAL    (motor1_velocity_f,     "rad/s", m1FilteredVelocityRadPerSec)
SIGNAL    (motor2_velocity_f,     "rad/s", m2FilteredVelocityRadPerSec)

// IMU timing
SIGNAL    (imu_sequence,          "",      imu::latest().sequence)
SIGNAL    (imu_read_time_us,      "us",    imu::latest().readTimeUs)
SIGNAL    (imu_dt_s,              "s",     imu::latest().dtSec)
SIGNAL    (imu_accepted_samples,  "",      imu::timingStatistics().acceptedSamples)
SIGNAL    (imu_long_gap_count,    "",      imu::timingStatistics().longGapCount)
SIGNAL    (imu_max_interval_us,   "us",    imu::timingStatistics().maxIntervalUs)

// Control loop timing
SIGNAL    (control_start_us,      "us",    controlStartUs)
SIGNAL    (control_execution_us,  "us",    controlExecutionUs)
SIGNAL    (tick_dt_s,             "s",     controlTimestepSec)
SIGNAL_MAX(loop_interval_max_us,  "us",    loopEntryIntervalUs)

// Balance state. balance_state: 0 Disabled, 1 Arming, 2 Run, 3 Commissioning, 4 Fault. balance_fault: the FaultReason
// order of BalanceController.h (0 None to 11 TiltSaturated).
SIGNAL    (balance_state,         "",      balanceOutput.state)
SIGNAL    (balance_fault,         "",      balanceOutput.fault)
SIGNAL    (overspeed_tripped,     "",      wheelOverspeed.tripped)
SIGNAL    (motor_test_active,     "",      motorDiagnosticState.active)

// Link and system health. The link counters count from boot; the health values are refreshed once a second.
SIGNAL    (link_tx_drop_uart,     "",      LinkCountersNow().txDropUart)
SIGNAL    (link_tx_drop_usb,      "",      LinkCountersNow().txDropUsb)
SIGNAL    (link_rx_bad_uart,      "",      LinkCountersNow().rxBadUart)
SIGNAL    (link_rx_bad_usb,       "",      LinkCountersNow().rxBadUsb)
SIGNAL    (link_req_dropped,      "",      LinkCountersNow().reqDropped)
SIGNAL    (link_undecodable,      "",      LinkCountersNow().undecodable)
SIGNAL    (uptime_s,              "s",     systemHealth.uptimeS)
SIGNAL    (heap_free_b,           "B",     systemHealth.heapFreeB)
SIGNAL    (heap_min_free_b,       "B",     systemHealth.heapMinFreeB)
SIGNAL    (stack_free_loop_b,     "B",     systemHealth.stackFreeLoopB)
SIGNAL    (stack_free_link_b,     "B",     systemHealth.stackFreeLinkB)

// Settings. A refused value leaves the running one as it was, and the Ack says why. Not listed: the Config fields that
// copy motor constants (kvRating, phaseResistanceOhm, voltage and current limits, wheel signs). They copy the constants
// SimpleFOC uses, so changing a copy would leave the effort envelope and the drive disagreeing.
PARAM_LIVE(speed_kp,              "rad/(m/s)",   BalanceGroup,    speedKpRadPerMps, 0.05)
PARAM_LIVE(speed_ki,              "rad/(m/s*s)", BalanceGroup,    speedKiRadPerMpsS, 0.02)
PARAM     (speed_i_on,            "",            BalanceGroup,    speedIntegralEnabled)
PARAM     (tilt_ref_limit,        "rad",         BalanceGroup,    tiltRefLimitRad)
PARAM     (trim_envelope,         "rad",         BalanceGroup,    trimEnvelopeRad)
PARAM_LIVE(tilt_p,                "A/rad",       BalanceGroup,    tiltPAPerRad, 1.0)
PARAM_LIVE(gyro_damping,          "A/(rad/s)",   BalanceGroup,    gyroDampingAPerRadPerSec, 0.1)
PARAM_LIVE(yaw_p,                 "A/(rad/s)",   BalanceGroup,    yawPAPerRadPerSec, 0.15)
PARAM_LIVE(speed_cap,             "m/s",         BalanceGroup,    speedCapMps, 0.1)
PARAM_LIVE(yaw_rate_cap,          "rad/s",       BalanceGroup,    yawRateCapRadPerSec, 0.25)
PARAM     (mahony_kp,             "",            ImuGroup,        mahonyTwoKp)
PARAM     (mahony_ki,             "",            ImuGroup,        mahonyTwoKi)
PARAM     (gyro_cutoff_hz,        "Hz",          ImuGroup,        gyroCutoffHz)
PARAM     (servo1_trim,           "deg",         ServoTrimGroup,  servo1Degrees)
PARAM     (servo2_trim,           "deg",         ServoTrimGroup,  servo2Degrees)
PARAM     (servo3_trim,           "deg",         ServoTrimGroup,  servo3Degrees)
PARAM     (servo4_trim,           "deg",         ServoTrimGroup,  servo4Degrees)
PARAM_LIVE(log_level,             "",            LogGroup,        minLevel, INFINITY)

// Actions. The bounds are the constants the handlers use, so this table cannot drift from them. The endpoint refuses an
// argument outside its bounds before the handler runs; the handler refuses what it cannot judge (balance, calibration,
// overspeed, a fault). A motor test has a lease and stops when it runs out, so a lost link stops it too.
ACTION    (motor_current,         startMotorCurrent, ARG(left_a, -MotorDiagnostic::kMaximumCurrentA, MotorDiagnostic::kMaximumCurrentA), ARG(right_a, -MotorDiagnostic::kMaximumCurrentA, MotorDiagnostic::kMaximumCurrentA), ARG(lease_ms, 1, MotorDiagnostic::kMaximumLeaseMs))
ACTION    (motor_voltage,        startMotorVoltage, ARG(motor, 1, 2), ARG(voltage_v, -MotorDiagnostic::kMaximumVoltageV, MotorDiagnostic::kMaximumVoltageV), ARG(lease_ms, 1, MotorDiagnostic::kMaximumLeaseMs))
ACTION_NO_ARGS(motor_stop,        stopMotorTest)
ACTION_NO_ARGS(motor_reset,       resetMotorFault)
ACTION    (calibrate,            startCalibration, ARG(step, 0, 3))
ACTION    (balance,              setLinkBalance, ARG(on, 0, 1))
ACTION_NO_ARGS(reboot,            requestReboot)
