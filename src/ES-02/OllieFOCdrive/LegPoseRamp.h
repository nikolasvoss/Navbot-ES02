#pragma once

// Moves the commanded leg angle from `current` toward `target` by at most `maxDegPerSec * dtSec` degrees.
// A hobby servo goes to a new angle as fast as it can, so the sketch feeds it this ramp instead of the target.
// Never overshoots the target; a negative or zero `dtSec` leaves `current` unchanged.
inline float stepLegPoseToward(float current, float target, float maxDegPerSec, float dtSec) {
  if (dtSec <= 0.0f) return current;
  const float maxStep = maxDegPerSec * dtSec;
  if (target > current + maxStep) return current + maxStep;
  if (target < current - maxStep) return current - maxStep;
  return target;
}
