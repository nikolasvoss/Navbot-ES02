// Expands the firmware's real signals.def.h against stand-ins for the sketch state. It proves that every expression
// type-checks and has a wire type, and prints the table for run_host_test.sh to compare with scripts/signals_catalog.py.
#include <cstdio>
#include <cstring>

#include "imu.h"

namespace imu {
const Snapshot& latest() {
  static Snapshot snapshot;
  return snapshot;
}
TimingStatistics timingStatistics() { return {}; }
}  // namespace imu

struct DqVoltage {
  float d, q;
};
struct StandInMotor {  // the fields of the SimpleFOC motor that the entries read
  float target, current_sp, shaft_velocity;
  DqVoltage voltage;
};
StandInMotor motor1, motor2;
int pidGainsMode;
int servoTraceAngle[4];
float batteryInstantRawV, batteryVoltage, movementSpeed, bodyTurn, legPoseDegrees;
float m1FilteredVelocityRadPerSec, m2FilteredVelocityRadPerSec, controlTimestepSec;
uint32_t controlStartUs, controlExecutionUs, loopEntryIntervalUs;

#define SIGNAL_DEF_FILE "signals.def.h"
#include "signal_table_expand.h"

int main() {
  static const char* const kindNames[] = {"", "SIGNAL", "SIGNAL_MIN", "SIGNAL_MAX", "PARAM", "ACTION"};
  static const char* const typeNames[] = {"-", "F32", "I32", "U32", "BOOL"};
  bool typesOk = true;
  for (size_t i = 0; i < kSignalCount; ++i) {
    const SignalEntry& e = kSignals[i];
    std::printf("%zu\t%s\t%s\t%s\t%s\n", i, e.name, e.unit, kindNames[static_cast<int>(e.kind)],
                typeNames[static_cast<int>(e.type)]);
    if (std::strcmp(e.name, "ch5_mode") == 0) typesOk &= e.type == SignalType::I32;
    if (std::strcmp(e.name, "voltage_min_raw_v") == 0) typesOk &= e.type == SignalType::F32;
    if (std::strcmp(e.name, "imu_sequence") == 0) typesOk &= e.type == SignalType::U32;
    if (std::strcmp(e.name, "servo1_cmd_deg") == 0) typesOk &= e.type == SignalType::I32;
  }
  if (!typesOk) {
    std::fprintf(stderr, "a signal has the wrong wire type\n");
    return 1;
  }
  return 0;
}
