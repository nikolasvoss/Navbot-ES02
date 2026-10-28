// Every value the link can log. One line per value: SIGNAL(name, unit, expression). The expression reads existing
// state and is evaluated each time a subscription samples the value. SIGNAL_MIN and SIGNAL_MAX report the lowest or
// highest value over the control ticks since the previous row of the same subscription.
//
// The wire ID of an entry is its position in this file, so it shifts when a line is added or removed; the host looks
// names up in the catalog of each connection. A name is at most 24 characters, a unit at most 16.
//
// The names are the column names of the K55 to K58 profiles (scripts/capture_balance_trace.py) where they fit the
// catalog, so scripts/navlink_presets.yaml reproduces those profiles; docs/integration/software/navlink.md lists the
// differences. The file ends in .h so that arduino-cli copies it into the build. It has no include guard: it is
// expanded several times with different macro definitions.

// Receiver and battery
SIGNAL    (ch5_mode,              "",      pidGainsMode)
SIGNAL_MIN(voltage_min_raw_v,     "V",     batteryInstantRawV)
SIGNAL    (voltage_filtered_v,    "V",     batteryVoltage)
SIGNAL    (battery_raw_instant_v, "V",     batteryInstantRawV)
SIGNAL    (ch3_target,            "m/s",   movementSpeed)
SIGNAL    (body_turn_command,     "rad/s", bodyTurn)

// Attitude and gyro
SIGNAL    (roll_deg,              "deg",   imu::latest().correctedRollDegrees)
SIGNAL    (pitch_deg,             "deg",   imu::latest().correctedPitchDegrees)
SIGNAL    (gyro_z_rad_s,          "rad/s", imu::latest().angularVelocityRadPerSec.z)

// Legs: the four servos get the same angle; the window of leg_pose is what the K profiles report as servo range
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
