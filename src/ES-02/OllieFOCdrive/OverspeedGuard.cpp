#include "OverspeedGuard.h"

#include <cmath>

namespace OverspeedGuard {
namespace {
bool elapsed(uint32_t nowMs, uint32_t startMs, uint32_t durationMs) {
  return static_cast<uint32_t>(nowMs - startMs) >= durationMs;
}

void recordPeak(State &state, float leftRadPerSec, float rightRadPerSec) {
  if (!std::isfinite(leftRadPerSec) || !std::isfinite(rightRadPerSec)) return;
  const float speed = std::fmax(std::fabs(leftRadPerSec), std::fabs(rightRadPerSec));
  if (speed > state.peakRadPerSec) state.peakRadPerSec = speed;
}
}  // namespace

bool update(State &state, uint32_t nowMs, float leftRadPerSec, float rightRadPerSec) {
  if (state.tripped) {
    recordPeak(state, leftRadPerSec, rightRadPerSec);
    return true;
  }
  if (!std::isfinite(leftRadPerSec) || !std::isfinite(rightRadPerSec)) {
    state.tripped = true;
    return true;
  }

  const float speed = std::fmax(std::fabs(leftRadPerSec), std::fabs(rightRadPerSec));
  if (speed <= kLimitRadPerSec) {
    // A sample at or below the limit ends the run without tripping, so its peak is discarded.
    state.pending = false;
    state.peakRadPerSec = 0.0f;
    return false;
  }

  if (!state.pending) {
    state.pending = true;
    state.overSinceMs = nowMs;
    state.peakRadPerSec = 0.0f;
  }
  recordPeak(state, leftRadPerSec, rightRadPerSec);
  if (elapsed(nowMs, state.overSinceMs, kDebounceMs)) state.tripped = true;
  return state.tripped;
}

void reset(State &state) { state = State{}; }

bool WithinLimit(float leftRadPerSec, float rightRadPerSec) {
  return std::fabs(leftRadPerSec) <= kLimitRadPerSec && std::fabs(rightRadPerSec) <= kLimitRadPerSec;
}

bool MayRearm(const State &state, float leftRadPerSec, float rightRadPerSec) {
  return !state.tripped || WithinLimit(leftRadPerSec, rightRadPerSec);
}

}  // namespace OverspeedGuard
