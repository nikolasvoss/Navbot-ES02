#include "RecordingStubs.h"

#include <stdio.h>
#include <string.h>

#include "Logging.h"
#include "RobotLogCapture.h"

namespace Recording {

bool dueResult = true;
static std::vector<Event> g_events;
static int g_rangeCalls = 0;

void reset() {
  g_events.clear();
  g_rangeCalls = 0;
  dueResult = true;
}

const std::vector<Event>& events() { return g_events; }

static std::string hex(const void* data, size_t size) {
  const unsigned char* bytes = static_cast<const unsigned char*>(data);
  std::string out;
  char pair[3];
  for (size_t i = 0; i < size; ++i) {
    snprintf(pair, sizeof pair, "%02x", bytes[i]);
    out += pair;
  }
  return out;
}

template <class Sample>
static Logging::Result record(const char* kind, const Sample& sample) {
  g_events.push_back({kind, sample.sequence, hex(&sample, sizeof sample)});
  return Logging::Result::Accepted;
}

}  // namespace Recording

namespace Logging {

Result submit(const TraceSample& sample) { return Recording::record("Trace", sample); }
Result submit(const ControlSample& sample) { return Recording::record("Control", sample); }
Result submit(const BalanceSample& sample) { return Recording::record("Balance", sample); }
Result submit(const DriveSample& sample) { return Recording::record("Drive", sample); }

Result submit(const DebugSample& sample) {
  char kind[16];
  snprintf(kind, sizeof kind, "K%d", static_cast<int>(sample.selector));
  Recording::g_events.push_back(
      {kind, 0, Recording::hex(&sample.payload, sizeof sample.payload)});
  return Result::Accepted;
}

}  // namespace Logging

namespace RobotLogCapture {

bool selectedDebugDue(Logging::DebugSelector selector, uint32_t nowMs) {
  char text[48];
  snprintf(text, sizeof text, "K%d now=%u -> %d", static_cast<int>(selector),
           static_cast<unsigned>(nowMs), Recording::dueResult ? 1 : 0);
  Recording::g_events.push_back({"due", 0, text});
  return Recording::dueResult;
}

void copyServoRangesAndBeginNextWindow(int32_t ranges[4]) {
  const int call = ++Recording::g_rangeCalls;
  for (int i = 0; i < 4; ++i) ranges[i] = call * 100 + (i * 37) % 11 * 7;
  char text[48];
  snprintf(text, sizeof text, "call=%d", call);
  Recording::g_events.push_back({"ranges", 0, text});
}

}  // namespace RobotLogCapture
