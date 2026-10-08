#pragma once

#include "LogSamples.h"

namespace RobotLogCapture {

Logging::Profile profileForSelector(int selector);
bool isMeasurementProfile(Logging::Profile profile);
void setProfileForSelector(int selector);
void resetControlWindow();
void observeServoAngles(const int32_t angles[4]);
void copyServoRanges(int32_t ranges[4]);

}  // namespace RobotLogCapture
