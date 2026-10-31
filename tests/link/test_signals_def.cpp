// Expands the firmware's real signals.def.h against stand-ins for the sketch state and its settings groups and action
// handlers. It proves that every expression type-checks and has a wire type, and prints the table for run_host_test.sh
// to compare with scripts/signals_catalog.py. The groups use the real rules (settings_groups.h); the state is a stand-in.
#include <cstdio>
#include <cstring>

#include "BalanceController.h"
#include "Calibration.h"
#include "MotorDiagnostic.h"
#include "OverspeedGuard.h"
#include "imu.h"
#include "settings_groups.h"

namespace imu {
const Snapshot& latest() {
  static Snapshot snapshot;
  return snapshot;
}
TimingStatistics timingStatistics() { return {}; }
Settings standInImuSettings;
Settings settings() { return standInImuSettings; }
bool setSettings(const Settings& candidate) {
  standInImuSettings = candidate;
  return true;
}
}  // namespace imu

struct DqVoltage {
  float d, q;
};
struct StandInMotor {  // the fields of the SimpleFOC motor that the entries read
  float target, current_sp, shaft_velocity;
  DqVoltage voltage;
};
StandInMotor motor1, motor2;
uint8_t ch5Position;
bool rcValid, linkBalance;
struct StandInLinkCommand {
  bool seen;
  float speedMps, turnRadPerSec;
  uint32_t receivedUs;
} linkCommand;
uint32_t linkCommandAgeMs() { return 0; }
int servoTraceAngle[4];
float batteryInstantRawV, batteryVoltage, movementSpeed, bodyTurn, legPoseDegrees;
float m1FilteredVelocityRadPerSec, m2FilteredVelocityRadPerSec, controlTimestepSec;
uint32_t controlStartUs, controlExecutionUs, loopEntryIntervalUs;

// The settings groups and action handlers that signals.def.h names. The sketch defines the real ones.
Config standInBalance;
struct BalanceGroup {
  using Struct = Config;
  static Config current() { return standInBalance; }
  static bool apply(const Config& candidate, const char** error) {
    return ApplyBalanceSettings(standInBalance, candidate, error);
  }
};
struct ImuGroup {
  using Struct = imu::Settings;
  static imu::Settings current() { return imu::settings(); }
  static bool apply(const imu::Settings& candidate, const char** error) {
    if (imu::setSettings(candidate)) return true;
    *error = "out of range";
    return false;
  }
};
ServoTrimsDegrees standInTrims;
struct ServoTrimGroup {
  using Struct = ServoTrimsDegrees;
  static ServoTrimsDegrees current() { return standInTrims; }
  static bool apply(const ServoTrimsDegrees& candidate, const char** error) {
    return ApplyServoTrims(standInTrims, candidate, error);
  }
};
bool startMotorCurrent(const float*, const char**) { return true; }
bool startMotorVoltage(const float*, const char**) { return true; }
bool stopMotorTest(const float*, const char**) { return true; }
bool resetMotorFault(const float*, const char**) { return true; }
bool startCalibration(const float*, const char**) { return true; }

// The balance, overspeed and motor test state, the link counters and the system health are the sketch's; these are
// stand-ins with the same fields. LogSettings and LogGroup follow the sketch's rule for log_level.
Output balanceOutput;
OverspeedGuard::State wheelOverspeed;
MotorDiagnostic::State motorDiagnosticState;
struct LinkCounters {
  uint32_t txDropUart, txDropUsb, rxBadUart, rxBadUsb, reqDropped, undecodable;
};
LinkCounters LinkCountersNow() { return {}; }
struct SystemHealth {
  uint32_t uptimeS, heapFreeB, heapMinFreeB, stackFreeLoopB, stackFreeLinkB;
} systemHealth{};
struct LogSettings {
  uint32_t minLevel;
};
LogSettings standInLog{2};
struct LogGroup {
  using Struct = LogSettings;
  static LogSettings current() { return standInLog; }
  static bool apply(const LogSettings& candidate, const char** error) {
    if (candidate.minLevel < 1 || candidate.minLevel > 4) {
      *error = "takes 1 (DEBUG) to 4 (ERROR)";
      return false;
    }
    standInLog = candidate;
    return true;
  }
};
bool setLinkBalance(const float*, const char**) { return true; }
bool requestReboot(const float*, const char**) { return true; }

#define SIGNAL_DEF_FILE "signals.def.h"
#include "signal_table_expand.h"

namespace {
const SignalEntry* Find(const char* name) {
  for (size_t i = 0; i < kSignalCount; ++i)
    if (std::strcmp(kSignals[i].name, name) == 0) return &kSignals[i];
  return nullptr;
}
}  // namespace

int main() {
  static const char* const kindNames[] = {"", "SIGNAL", "SIGNAL_MIN", "SIGNAL_MAX", "PARAM", "ACTION"};
  bool typesOk = true;
  for (size_t i = 0; i < kSignalCount; ++i) {
    const SignalEntry& e = kSignals[i];
    std::printf("%zu\t%s\t%s\t%s\t%g\n", i, e.name, e.unit, kindNames[static_cast<int>(e.kind)],
                static_cast<double>(e.liveStep));
    if (std::strcmp(e.name, "ch5_position") == 0) typesOk &= e.type == SignalType::U32;
    if (std::strcmp(e.name, "rc_valid") == 0) typesOk &= e.type == SignalType::Bool;
    if (std::strcmp(e.name, "voltage_min_raw_v") == 0) typesOk &= e.type == SignalType::F32;
    if (std::strcmp(e.name, "imu_sequence") == 0) typesOk &= e.type == SignalType::U32;
    if (std::strcmp(e.name, "servo1_cmd_deg") == 0) typesOk &= e.type == SignalType::I32;
  }
  // The kinds carry their functions: a PARAM has read and write, an ACTION has call and its bounds, a SIGNAL has
  // neither write nor call.
  const SignalEntry* speedKp = Find("speed_kp");
  const SignalEntry* gyroCutoff = Find("gyro_cutoff_hz");
  const SignalEntry* motorCurrent = Find("motor_current");
  const SignalEntry* motorReset = Find("motor_reset");
  const SignalEntry* speed = Find("ch3_target");
  if (!speedKp || !gyroCutoff || !motorCurrent || !motorReset || !speed) return 1;
  typesOk &= speedKp->kind == SignalKind::Param && speedKp->type == SignalType::F32 && speedKp->read && speedKp->write;
  typesOk &= gyroCutoff->type == SignalType::U32 && gyroCutoff->write != nullptr;
  typesOk &= motorCurrent->kind == SignalKind::Action && motorCurrent->call && motorCurrent->args &&
             motorCurrent->args().count == 3;
  typesOk &= std::strcmp(motorCurrent->args().items[2].name, "lease_ms") == 0;
  typesOk &= motorCurrent->args().items[2].max == 5000.0f;
  typesOk &= motorReset->call && motorReset->args && motorReset->args().count == 0;
  typesOk &= speed->write == nullptr && speed->call == nullptr;
  if (!typesOk) {
    std::fprintf(stderr, "a signal has the wrong wire type or the wrong functions\n");
    return 1;
  }
  return 0;
}
