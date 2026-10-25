#ifndef LINK_SCHEDULE_H
#define LINK_SCHEDULE_H

#include <stdint.h>

// Fixed-period send schedule on a wrapping microsecond clock (micros()). No Arduino dependency, so it is host-tested.
struct LinkSchedule {
  bool started = false;
  uint32_t deadlineUs = 0;
};

// Returns true when a send is due at nowUs. The first call is due and starts the schedule at nowUs.
// Each send advances the deadline by exactly periodUs, so the average rate is 1 / periodUs even though the caller
// only checks once per control tick. When nowUs is a full period or more past the deadline (a stall), the deadline
// restarts at nowUs + periodUs: one send, no burst of catch-up frames.
// Compares the signed difference, so it stays correct across the micros() wrap while calls are under 2^31 us apart.
inline bool LinkScheduleDue(LinkSchedule& schedule, uint32_t nowUs, uint32_t periodUs) {
  if (!schedule.started) {
    schedule.started = true;
    schedule.deadlineUs = nowUs + periodUs;
    return true;
  }
  const int32_t lateUs = static_cast<int32_t>(nowUs - schedule.deadlineUs);
  if (lateUs < 0) return false;
  schedule.deadlineUs = static_cast<uint32_t>(lateUs) >= periodUs ? nowUs + periodUs : schedule.deadlineUs + periodUs;
  return true;
}

#endif
