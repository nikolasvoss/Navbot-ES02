// Reads "<selection> <gate> <due> <variant>" lines from stdin, one control tick each against a
// single telemetry state, and prints the samples DebugTelemetry submits. The variant selects the
// Scenarios::makeInputs values. Output per tick is "<selection> <gate> -> <Kind[#sequence]>,..."
// or "-" when nothing was submitted.
#include <iostream>
#include <sstream>
#include <string>

#include "DebugTelemetry.h"
#include "Scenarios.h"

int main() {
  DebugTelemetry::TelemetryState state{0, 0, 0};
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream words(line);
    int selection = 0, due = 0, variant = 0;
    unsigned gate = 0;
    words >> selection >> gate >> due >> variant;
    DebugTelemetry::TelemetryInputs in = Scenarios::makeInputs(variant, selection, 0);
    in.controlGateSequence = gate;
    Recording::reset();
    Recording::dueResult = due != 0;
    DebugTelemetry::submit(in, state);
    std::string submitted;
    for (const Recording::Event& e : Recording::events()) {
      if (e.kind == "due" || e.kind == "ranges") continue;
      if (!submitted.empty()) submitted += ",";
      submitted += e.kind;
      if (e.kind != "K" + std::to_string(selection)) submitted += "#" + std::to_string(e.sequence);
    }
    std::cout << selection << " " << gate << " -> " << (submitted.empty() ? "-" : submitted) << "\n";
  }
  return 0;
}
