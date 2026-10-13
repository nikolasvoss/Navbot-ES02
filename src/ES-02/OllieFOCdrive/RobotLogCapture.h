#pragma once

#include "LogSamples.h"

namespace RobotLogCapture {

Logging::Profile profileForSelector(int selector);
void setProfileForSelector(int selector);
bool selectedDebugDue(Logging::DebugSelector selector, uint32_t nowMs);
void observeServoAngles(const int angles[4]);
void copyServoRangesAndBeginNextWindow(int32_t ranges[4]);

}  // namespace RobotLogCapture
