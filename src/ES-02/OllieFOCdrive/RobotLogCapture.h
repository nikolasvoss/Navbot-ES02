#pragma once

#include "LogSamples.h"

namespace RobotLogCapture {

Logging::Profile profileForSelector(int selector);
bool isMeasurementProfile(Logging::Profile profile);
void setProfileForSelector(int selector);
bool selectedDebugDue(Logging::DebugSelector selector, uint32_t nowMs);
void resetControlWindow();
void observeServoAngles(const int32_t angles[4]);
// Returns each observed max-min range, then reanchors the window at the latest angles.
void copyServoRanges(int32_t ranges[4]);

}  // namespace RobotLogCapture
