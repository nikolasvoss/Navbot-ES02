#include <cassert>
#include <cstdio>
#include <limits>

#include "../../src/ES-02/OllieFOCdrive/OverspeedGuard.h"

namespace {
constexpr float kOver = 30.01f;
}  // namespace

int main() {
  using namespace OverspeedGuard;

  // A single spike does not trip.
  {
    State s{};
    assert(!update(s, 1000, kOver, 0.0f));
    assert(!update(s, 1001, 0.0f, 0.0f));
    assert(!s.tripped && !s.pending);
  }

  // 49 ms over does not trip; 50 ms over trips.
  {
    State s{};
    assert(!update(s, 2000, kOver, 0.0f));
    assert(!update(s, 2049, kOver, 0.0f));
    assert(!s.tripped);
    assert(update(s, 2050, kOver, 0.0f));
    assert(s.tripped);
  }

  // A dip to the limit restarts the timer.
  {
    State s{};
    assert(!update(s, 3000, kOver, 0.0f));
    assert(!update(s, 3030, kOver, 0.0f));
    assert(!update(s, 3031, 30.0f, 0.0f));  // exactly at the limit is not over
    assert(!s.pending);
    assert(!update(s, 3032, kOver, 0.0f));
    assert(!update(s, 3081, kOver, 0.0f));  // 49 ms since 3032
    assert(!s.tripped);
    assert(update(s, 3082, kOver, 0.0f));  // 50 ms since 3032
  }

  // Non-finite speed trips immediately.
  {
    State s{};
    assert(update(s, 4000, std::numeric_limits<float>::quiet_NaN(), 0.0f));
    assert(s.tripped);
    State t{};
    assert(update(t, 4000, 0.0f, std::numeric_limits<float>::infinity()));
    assert(t.tripped);
  }

  // The latch holds after speed returns to normal.
  {
    State s{};
    update(s, 5000, 0.0f, kOver);
    assert(update(s, 5050, 0.0f, kOver));
    assert(update(s, 5051, 0.0f, 0.0f));
    assert(update(s, 6000, 1.0f, 1.0f));
    assert(s.tripped);
  }

  // reset() clears the latch and the pending run.
  {
    State s{};
    update(s, 7000, kOver, 0.0f);
    update(s, 7050, kOver, 0.0f);
    reset(s);
    assert(!s.tripped && !s.pending && s.peakRadPerSec == 0.0f);
    assert(!update(s, 7051, 0.0f, 0.0f));
    update(s, 7100, kOver, 0.0f);
    assert(!update(s, 7149, kOver, 0.0f));  // pending restarted at 7100
    assert(update(s, 7150, kOver, 0.0f));
    reset(s);
    assert(!s.tripped && !update(s, 7151, 0.0f, 0.0f));
  }

  // Millis wrap across 0xFFFFFFFF: start at 0xFFFFFFF6, trip at 0x00000028 (50 ms later).
  {
    State s{};
    assert(!update(s, 0xFFFFFFF6u, kOver, 0.0f));
    assert(!update(s, 0x00000027u, kOver, 0.0f));  // 49 ms
    assert(update(s, 0x00000028u, kOver, 0.0f));   // 50 ms
  }

  // Either wheel alone can trip, and negative speed counts by magnitude.
  {
    State right{};
    update(right, 8000, 0.0f, -kOver);
    assert(update(right, 8050, 0.0f, -kOver));
    State left{};
    update(left, 9000, -kOver, 0.0f);
    assert(update(left, 9050, -kOver, 0.0f));
  }

  // Peak records the largest |speed| of the run, and keeps recording once tripped.
  {
    State s{};
    update(s, 10000, 65.0f, 0.0f);
    update(s, 10010, 0.0f, -70.0f);
    assert(s.peakRadPerSec == 70.0f && !s.tripped);
    assert(update(s, 10050, 0.0f, -70.0f));  // 50 ms since 10000 trips
    assert(s.peakRadPerSec == 70.0f);
    update(s, 10061, 0.0f, 90.0f);
    assert(s.peakRadPerSec == 90.0f);
  }

  // A run that clears without tripping does not leave its peak behind.
  {
    State s{};
    update(s, 11000, 65.0f, 0.0f);
    assert(!update(s, 11001, 1.0f, 1.0f));
    assert(s.peakRadPerSec == 0.0f && !s.pending);
  }

  std::puts("overspeed guard passed");
  return 0;
}
