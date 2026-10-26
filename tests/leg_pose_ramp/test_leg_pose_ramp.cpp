#include <cassert>
#include <cstdio>

#include "../../src/ES-02/OllieFOCdrive/LegPoseRamp.h"

int main() {
  // One step moves by rate * dt.
  assert(stepLegPoseToward(0.0f, 20.0f, 15.0f, 0.1f) == 1.5f);
  // Downward steps work the same way.
  assert(stepLegPoseToward(20.0f, 0.0f, 15.0f, 0.1f) == 18.5f);
  // The target is never overshot.
  assert(stepLegPoseToward(19.9f, 20.0f, 15.0f, 0.1f) == 20.0f);
  assert(stepLegPoseToward(0.1f, 0.0f, 15.0f, 0.1f) == 0.0f);
  // Zero or negative dt changes nothing.
  assert(stepLegPoseToward(5.0f, 20.0f, 15.0f, 0.0f) == 5.0f);
  assert(stepLegPoseToward(5.0f, 20.0f, 15.0f, -0.1f) == 5.0f);
  // At the target it stays there.
  assert(stepLegPoseToward(20.0f, 20.0f, 15.0f, 0.001f) == 20.0f);

  // 1 ms ticks at 15 deg/s reach 20 degrees in about 1.33 s and then hold.
  float pose = 0.0f;
  int ticks = 0;
  while (pose < 20.0f && ticks < 10000) {
    pose = stepLegPoseToward(pose, 20.0f, 15.0f, 0.001f);
    ++ticks;
  }
  assert(pose == 20.0f);
  assert(ticks >= 1333 && ticks <= 1335);

  std::puts("leg pose ramp tests passed");
  return 0;
}
