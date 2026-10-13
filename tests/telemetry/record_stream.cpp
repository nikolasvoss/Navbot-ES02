// Prints the sample stream DebugTelemetry produces for the synthetic scenarios.
#include <stdio.h>

#include "DebugTelemetry.h"
#include "Scenarios.h"

int main() {
  const std::string stream = Scenarios::recordAll(
      [](const DebugTelemetry::TelemetryInputs& in, DebugTelemetry::TelemetryState& state) {
        if (DebugTelemetry::due(in.selection, in.nowMs, in.controlGateSequence))
          DebugTelemetry::submit(in, state);
      });
  fwrite(stream.data(), 1, stream.size(), stdout);
  return 0;
}
