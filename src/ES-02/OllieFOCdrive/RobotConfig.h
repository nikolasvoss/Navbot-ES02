#pragma once

// Robot constants and the balance settings at boot. The sketch and the host simulation (src/sim/) both use them.

#include <cstdint>

#include "BalanceController.h"

constexpr uint32_t kRcFrameTimeoutMs = 100;
constexpr float kControlTimestepMinSec = 0.001f;

// Wheel motor constants, measured 2026-10-22 (docs/robot/software/motor-commissioning.md).
// SimpleFOC uses phase_resistance as the per-phase (line-to-neutral) value in voltage.q = I * R + bemf.
// The meter reads 11.5 ohm phase to phase on both wheels; 5.75 ohm assumes a star winding, not yet verified.
struct WheelMotorParams {
  float phaseResistanceOhm;
  float kvRatingRpmPerV;  // SimpleFOC convention: V_bemf = velocity / (KV * sqrt(3) * RPM_TO_RADS)
};

constexpr float kWheelSupplyVoltageV = 8.4;
constexpr float kWheelVoltageLimitV = 4.2;  // usable q ceiling at about 7.3 V bus; the modulation limit is bus / sqrt(3)
constexpr float kWheelVelocityFilterTfSec = 0.001;
constexpr WheelMotorParams kLeftWheelMotor{5.75f, 79.5f};
constexpr WheelMotorParams kRightWheelMotor{5.75f, 76.7f};

// SimpleFOC adds the estimated back-EMF as voltage even at 0 A. If that estimate is larger than the real back-EMF
// the wheel accelerates by itself, so the rating handed to SimpleFOC is the measured KV times this factor (a larger
// KV gives a smaller estimate). Measured KV depends on the speed range (79.5 rpm/V over 0.5-2 V, about 108 rpm/V for
// 0.5-1 V). Bench test of the left wheel with motor_current 0 0 on 2026-10-24: 1.0 ran away from rest, 1.4 let the wheel speed
// up again between 17 and 30 rad/s, 1.8 slowed it down continuously. The right wheel takes the same factor on the
// operator's decision; it is not tested separately.
constexpr float kWheelKvSafetyFactor = 1.8f;

constexpr int kLegPoseDegrees = 20;  // fixed leg pose for all four servos, degrees
constexpr int kLegSagDegrees = 10;  // assumed leg angle after the robot has sagged, between 0 and kLegPoseDegrees
constexpr float kLegRampDegPerSec = 30.0f;  // leg pose ramp, 0 to 20 degrees in about 0.7 s
constexpr float kLegRampMaxDtSec = 0.02f;   // the first tick after boot reports the whole boot time as dt

// Balance settings group (the PARAM entries of signals.def.h name its fields). Built once at boot; SetParam replaces
// it through BalanceGroup, and the controller reads it at each tick. The balance and yaw gains below are the working
// values (tilt 8 A/rad, gyro damping 0.4), so the controller commands real wheel current as soon as it arms and runs.
// The envelope can also command a small braking current above roughly 50 rad/s, because the plan requires outputs
// inside the envelope. Each value is one of: existing constant (named), plan (decision or stated number), or an
// uncommissioned placeholder.
Config defaultBalanceConfig();
