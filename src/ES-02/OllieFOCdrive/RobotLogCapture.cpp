#include "RobotLogCapture.h"

#include "Logging.h"

#include <limits.h>

namespace {
constexpr uint32_t kSelectedDebugIntervalMs = 50;

uint32_t lastSbusPrintMs = 0;
uint32_t lastChannelTraceMs = 0;

int32_t servoMinimum[4] = {0, 0, 0, 0};
int32_t servoMaximum[4] = {0, 0, 0, 0};
int32_t servoLatest[4] = {0, 0, 0, 0};
bool servoWindowStarted = false;

bool gateDue(uint32_t nowMs, uint32_t &lastMs) {
  if (static_cast<uint32_t>(nowMs - lastMs) < kSelectedDebugIntervalMs) return false;

  lastMs = nowMs;
  return true;
}
}

namespace RobotLogCapture {

Logging::Profile profileForSelector(int selector) {
  if (Logging::isSelectedDebugSelector(selector) || Logging::isChannelDebugSelector(selector))
    return Logging::Profile::SelectedDebug;

  switch (selector) {
    case 55: return Logging::Profile::Trace;
    case 56: return Logging::Profile::Control;
    case 57: return Logging::Profile::Balance;
    case 58: return Logging::Profile::Drive;
    default: return Logging::Profile::Idle;
  }
}

void setProfileForSelector(int selector) {
  Logging::setProfile(profileForSelector(selector));
}

bool selectedDebugDue(Logging::DebugSelector selector, uint32_t nowMs) {
  const int value = static_cast<int>(selector);
  if (!Logging::isSelectedDebugSelector(value) && !Logging::isChannelDebugSelector(value)) return false;

  if (value == 8) return gateDue(nowMs, lastSbusPrintMs);
  if (Logging::isChannelDebugSelector(value))
    return gateDue(nowMs, lastChannelTraceMs);
  return true;
}

void observeServoAngles(const int angles[4]) {
  if (angles == nullptr) return;

  if (!servoWindowStarted) {
    for (int i = 0; i < 4; ++i) {
      servoMinimum[i] = angles[i];
      servoMaximum[i] = angles[i];
      servoLatest[i] = angles[i];
    }
    servoWindowStarted = true;
    return;
  }

  for (int i = 0; i < 4; ++i) {
    if (angles[i] < servoMinimum[i]) servoMinimum[i] = angles[i];
    if (angles[i] > servoMaximum[i]) servoMaximum[i] = angles[i];
    servoLatest[i] = angles[i];
  }
}

void copyServoRangesAndBeginNextWindow(int32_t ranges[4]) {
  if (ranges == nullptr) return;

  for (int i = 0; i < 4; ++i) {
    if (!servoWindowStarted) {
      ranges[i] = 0;
      continue;
    }

    const int64_t span = static_cast<int64_t>(servoMaximum[i]) - servoMinimum[i];
    ranges[i] = span > INT32_MAX ? INT32_MAX : static_cast<int32_t>(span);
    servoMinimum[i] = servoLatest[i];
    servoMaximum[i] = servoLatest[i];
  }
}

}
