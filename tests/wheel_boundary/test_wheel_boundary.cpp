#include <cassert>
#include <cmath>
#include <cstdio>

#include "../../src/ES-02/OllieFOCdrive/WheelBoundary.h"

namespace {
bool nearly(float actual, float expected) { return std::fabs(actual - expected) < 1e-4f; }
}  // namespace

int main() {
  using namespace WheelBoundary;

  // Forward is negative in motor-local values on both wheels.
  const MotorValues targets = toMotorTargets(1.0f, -2.0f);
  assert(targets.motor1 == -1.0f && targets.motor2 == 2.0f);
  const WheelValues forward = fromMotorVelocities(-3.0f, 4.0f);
  assert(forward.left == 3.0f && forward.right == -4.0f);

  // Round trip: a wheel target survives the motor-side mapping and back.
  const MotorValues roundTrip = toMotorTargets(0.5f, -0.25f);
  const WheelValues wheels = fromMotorVelocities(roundTrip.motor1, roundTrip.motor2);
  assert(wheels.left == 0.5f && wheels.right == -0.25f);

  // First call only stores the sample.
  WheelSpeedEstimator estimator{};
  WheelValues raw{};
  assert(!estimator.update(0.0, 0.0, 1000, raw));
  assert(estimator.hasPrevious);

  // 10 ms step: motor1 +0.5 rad is -50 rad/s forward on the left, motor2 -0.25 rad is +25 rad/s forward on the right.
  assert(estimator.update(0.5, -0.25, 11000, raw));
  assert(estimator.lastIntervalUs == 10000);
  assert(nearly(raw.left, -50.0f) && nearly(raw.right, 25.0f));

  // Wrap across the uint32 micros counter: 0xFFFFFFF0 + 10000 us wraps to 9984.
  WheelSpeedEstimator wrapped{};
  assert(!wrapped.update(0.0, 0.0, 0xFFFFFFF0u, raw));
  assert(wrapped.update(0.5, -0.25, 9984u, raw));
  assert(wrapped.lastIntervalUs == 10000);
  assert(nearly(raw.left, -50.0f) && nearly(raw.right, 25.0f));

  // Zero elapsed time is refused, and the output is left untouched.
  WheelSpeedEstimator stalled{};
  WheelValues untouched{7.0f, 8.0f};
  assert(!stalled.update(0.0, 0.0, 5000, untouched));
  assert(!stalled.update(1.0, 1.0, 5000, untouched));
  assert(untouched.left == 7.0f && untouched.right == 8.0f);
  assert(stalled.lastIntervalUs == 0);

  std::puts("wheel_boundary: all tests passed");
  return 0;
}
