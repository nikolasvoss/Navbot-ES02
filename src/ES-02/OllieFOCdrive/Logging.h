#pragma once

#include "LogSamples.h"

namespace Logging {

bool begin();
void setProfile(Profile profile);
Profile profile();
void setLevel(Level level);

Result message(Level level, const char *tag, const char *format, ...);
Result submit(const DiagnosticSample &sample);
Result submit(const TraceSample &sample);
Result submit(const ControlSample &sample);
Result submit(const BalanceSample &sample);
Result submit(const DriveSample &sample);
Result submit(const DebugSample &sample);

Status status();

}
