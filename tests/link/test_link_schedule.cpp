#include "link_schedule.h"

#include <cstdio>

namespace {

const uint32_t kPeriodUs = 20000;
const uint32_t kTickUs = 1410;  // measured control tick period, 2026-10-25

bool Check(const char* name, bool condition) {
  std::printf("%s: %s\n", name, condition ? "ok" : "FAIL");
  return condition;
}

struct RunResult {
  uint32_t sends;
  uint32_t minIntervalUs;
  uint32_t maxIntervalUs;
};

// Calls the schedule once per tick for durationUs, starting at startUs, like runControlTick() does.
RunResult RunTicks(LinkSchedule& schedule, uint32_t startUs, uint32_t durationUs) {
  RunResult result = {0, 0xFFFFFFFFu, 0};
  uint32_t lastSendUs = 0;
  for (uint32_t elapsedUs = 0; elapsedUs < durationUs; elapsedUs += kTickUs) {
    const uint32_t nowUs = startUs + elapsedUs;
    if (!LinkScheduleDue(schedule, nowUs, kPeriodUs)) continue;
    if (result.sends > 0) {
      const uint32_t intervalUs = nowUs - lastSendUs;
      if (intervalUs < result.minIntervalUs) result.minIntervalUs = intervalUs;
      if (intervalUs > result.maxIntervalUs) result.maxIntervalUs = intervalUs;
    }
    lastSendUs = nowUs;
    result.sends++;
  }
  return result;
}

}  // namespace

int main() {
  bool ok = true;

  {
    // 10 s of 1.41 ms ticks: 500 sends (50 Hz), not 355 as with every 20th tick. A send lands on the first tick at or
    // after its deadline, so each interval is within one tick of 20 ms.
    LinkSchedule schedule;
    const RunResult r = RunTicks(schedule, 1000, 10000000);
    ok &= Check("steady 1.41 ms ticks: 500 sends in 10 s", r.sends == 500);
    ok &= Check("steady 1.41 ms ticks: intervals in (18.59 ms, 21.41 ms)",
                r.minIntervalUs > kPeriodUs - kTickUs && r.maxIntervalUs < kPeriodUs + kTickUs);
    ok &= Check("steady 1.41 ms ticks: min interval 19740 us", r.minIntervalUs == 19740);
    ok &= Check("steady 1.41 ms ticks: max interval 21150 us", r.maxIntervalUs == 21150);
  }

  {
    // The same run with micros() wrapping 5 s in.
    LinkSchedule schedule;
    const RunResult r = RunTicks(schedule, 0xFFFFFFFFu - 5000000u, 10000000);
    ok &= Check("wrap during run: 500 sends in 10 s", r.sends == 500);
    ok &= Check("wrap during run: intervals in (18.59 ms, 21.41 ms)",
                r.minIntervalUs > kPeriodUs - kTickUs && r.maxIntervalUs < kPeriodUs + kTickUs);
  }

  {
    // First call is due; the deadline then advances by exactly one period.
    LinkSchedule schedule;
    ok &= Check("first call due", LinkScheduleDue(schedule, 0, kPeriodUs));
    ok &= Check("19999 us not due", !LinkScheduleDue(schedule, 19999, kPeriodUs));
    ok &= Check("20000 us due", LinkScheduleDue(schedule, 20000, kPeriodUs));
    ok &= Check("41000 us due (late by 1000)", LinkScheduleDue(schedule, 41000, kPeriodUs));
    ok &= Check("deadline 60000 after late send", schedule.deadlineUs == 60000);
  }

  {
    // A 100 ms stall: one send at the first tick after it, then the schedule restarts from that tick.
    LinkSchedule schedule;
    ok &= Check("stall: first call due", LinkScheduleDue(schedule, 0, kPeriodUs));
    ok &= Check("stall: 20000 us due", LinkScheduleDue(schedule, 20000, kPeriodUs));
    ok &= Check("stall: 140000 us due after the stall", LinkScheduleDue(schedule, 140000, kPeriodUs));
    ok &= Check("stall: deadline resynced to 160000", schedule.deadlineUs == 160000);
    const RunResult r = RunTicks(schedule, 140000 + kTickUs, kPeriodUs - kTickUs);
    ok &= Check("stall: no catch-up burst in the next 20 ms", r.sends == 0);
    ok &= Check("stall: 160000 us due", LinkScheduleDue(schedule, 160000, kPeriodUs));
  }

  {
    // Deadline past the wrap while now is still below it: `now > deadline` would send early here.
    LinkSchedule schedule;
    ok &= Check("wrap: first call at 0xFFFFC000 due", LinkScheduleDue(schedule, 0xFFFFC000u, kPeriodUs));
    ok &= Check("wrap: deadline wraps to 3616", schedule.deadlineUs == 3616);
    ok &= Check("wrap: 0xFFFFF000 not due", !LinkScheduleDue(schedule, 0xFFFFF000u, kPeriodUs));
    ok &= Check("wrap: 3615 not due", !LinkScheduleDue(schedule, 3615, kPeriodUs));
    ok &= Check("wrap: 3616 due", LinkScheduleDue(schedule, 3616, kPeriodUs));
    ok &= Check("wrap: deadline 23616", schedule.deadlineUs == 23616);
  }

  return ok ? 0 : 1;
}
