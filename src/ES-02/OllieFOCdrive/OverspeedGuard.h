#pragma once

#include <cstdint>

namespace OverspeedGuard {

// Wheel speed above this value counts as overspeed. Below the speed an enabled wheel reaches at a low battery, so
// the guard catches a runaway there. Lowered from 60 rad/s on 2026-10-24 before the first non-zero gain.
constexpr float kLimitRadPerSec = 30.0f;
// An over-limit run latches the guard only after it has lasted this long (measured with nowMs).
constexpr uint32_t kDebounceMs = 50;

struct State {
  bool tripped = false;
  bool pending = false;
  uint32_t overSinceMs = 0;
  // Largest |speed| of the current over-limit run, or of everything seen once tripped.
  float peakRadPerSec = 0.0f;
};

// Returns true when the guard is tripped. A tripped guard stays tripped until reset().
bool update(State &state, uint32_t nowMs, float leftRadPerSec, float rightRadPerSec);
void reset(State &state);

}  // namespace OverspeedGuard
