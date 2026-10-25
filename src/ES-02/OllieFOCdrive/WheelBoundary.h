#pragma once

#include <cstdint>

namespace WheelBoundary {

// Measured 2026-10-22: positive motor-local q (and positive shaft_velocity) rolls BOTH wheels backward,
// so forward is a negative motor-local value on both wheels. This is the only place motor signs appear.
constexpr float kLeftMotorSign = -1.0f;
constexpr float kRightMotorSign = -1.0f;

// motor1 is the left wheel and motor2 the right wheel, in SimpleFOC motor-local sign.
struct MotorValues {
  float motor1;
  float motor2;
};

// Forward is positive.
struct WheelValues {
  float left;
  float right;
};

// The same sign applies to any torque, current or voltage target.
MotorValues toMotorTargets(float leftForward, float rightForward);
WheelValues fromMotorVelocities(float motor1RadPerSec, float motor2RadPerSec);

// Estimates forward-positive wheel speeds from successive motor angles and micros() timestamps.
struct WheelSpeedEstimator {
  double previousAngle1Rad = 0.0;
  double previousAngle2Rad = 0.0;
  uint32_t previousReadTimeUs = 0;
  bool hasPrevious = false;
  uint32_t lastIntervalUs = 0;

  // Returns false, and leaves rawForwardRadPerSec untouched, on the first call or when no time has elapsed.
  bool update(double motor1AngleRad, double motor2AngleRad, uint32_t readTimeUs, WheelValues &rawForwardRadPerSec);
};

}  // namespace WheelBoundary
