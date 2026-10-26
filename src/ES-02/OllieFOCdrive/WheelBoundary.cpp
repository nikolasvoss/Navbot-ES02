#include "WheelBoundary.h"

namespace WheelBoundary {

MotorValues toMotorTargets(float leftForward, float rightForward) {
  return {leftForward * kLeftMotorSign, rightForward * kRightMotorSign};
}

WheelValues fromMotorVelocities(float motor1RadPerSec, float motor2RadPerSec) {
  return {motor1RadPerSec * kLeftMotorSign, motor2RadPerSec * kRightMotorSign};
}

bool WheelSpeedEstimator::update(double motor1AngleRad, double motor2AngleRad, uint32_t readTimeUs,
                                 WheelValues &rawForwardRadPerSec) {
  const bool hadPrevious = hasPrevious;
  lastIntervalUs = hadPrevious ? static_cast<uint32_t>(readTimeUs - previousReadTimeUs) : 0;
  const bool valid = hadPrevious && lastIntervalUs != 0;
  if (valid) {
    const double elapsedSec = static_cast<double>(lastIntervalUs) * 1e-6;
    rawForwardRadPerSec = fromMotorVelocities(static_cast<float>((motor1AngleRad - previousAngle1Rad) / elapsedSec),
                                              static_cast<float>((motor2AngleRad - previousAngle2Rad) / elapsedSec));
  }
  previousAngle1Rad = motor1AngleRad;
  previousAngle2Rad = motor2AngleRad;
  previousReadTimeUs = readTimeUs;
  hasPrevious = true;
  return valid;
}

}  // namespace WheelBoundary
