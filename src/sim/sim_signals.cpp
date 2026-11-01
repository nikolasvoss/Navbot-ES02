// The entries of signals.def.h as the host simulation expands them. The state is sim.h's globals and ControlTick::State.
// The settings groups, the health rows and the action handlers are the sketch's, with the behavior of the simulation:
// the motor tests, calibration and reboot are not simulated. link_server.cpp serves the table.

#include <Arduino.h>

#include "BalanceController.h"
#include "Calibration.h"
#include "ControlTick.h"
#include "Logging.h"
#include "MotorDiagnostic.h"
#include "ServoLimits.h"
#include "imu.h"
#include "link_uart.h"
#include "settings_groups.h"
#include "sim.h"

namespace {

// The sketch's SystemHealth. The simulation has no heap and no task stacks, so only the uptime is kept.
struct SystemHealth {
  uint32_t uptimeS;
  uint32_t heapFreeB;
  uint32_t heapMinFreeB;
  uint32_t stackFreeLoopB;
  uint32_t stackFreeLinkB;
} systemHealth{};
uint8_t resetReason = 1;  // esp_reset_reason_t power-on: the simulation always starts fresh

MotorDiagnostic::State motorDiagnosticState{};
ServoTrimsDegrees servoTrims{};

// The sketch's LogSettings: the level in the proto numbers, 1 DEBUG to 4 ERROR.
struct LogSettings {
  uint32_t minLevel;
};

struct BalanceGroup {
  using Struct = Config;
  static Config current() { return balanceSettings; }
  static bool apply(const Config &candidate, const char **error) {
    return ApplyBalanceSettings(balanceSettings, candidate, error);
  }
};

struct ImuGroup {
  using Struct = imu::Settings;
  static imu::Settings current() { return imu::settings(); }
  static bool apply(const imu::Settings &candidate, const char **error) {
    if (imu::setSettings(candidate)) return true;
    *error = "out of range";
    return false;
  }
};

struct ServoTrimGroup {
  using Struct = ServoTrimsDegrees;
  static ServoTrimsDegrees current() { return servoTrims; }
  static bool apply(const ServoTrimsDegrees &candidate, const char **error) {
    return ApplyServoTrims(servoTrims, candidate, error);
  }
};

struct LogGroup {
  using Struct = LogSettings;
  static LogSettings current() { return {static_cast<uint32_t>(Logging::minLevel()) + 1}; }
  static bool apply(const LogSettings &candidate, const char **error) {
    if (candidate.minLevel < 1 || candidate.minLevel > 4) {
      *error = "takes 1 (DEBUG) to 4 (ERROR)";
      return false;
    }
    Logging::setMinLevel(static_cast<Logging::Level>(candidate.minLevel - 1));
    return true;
  }
};

// The motor tests, calibration and reboot: the simulation has no hardware for them, so the link refuses them.
bool refuseNotSimulated(const char **error) {
  *error = "not simulated";
  return false;
}
bool startMotorCurrent(const float *, const char **error) { return refuseNotSimulated(error); }
bool startMotorVoltage(const float *, const char **error) { return refuseNotSimulated(error); }
bool stopMotorTest(const float *, const char **error) { return refuseNotSimulated(error); }
bool resetMotorFault(const float *, const char **error) { return refuseNotSimulated(error); }
bool startCalibration(const float *, const char **error) { return refuseNotSimulated(error); }
bool requestReboot(const float *, const char **error) { return refuseNotSimulated(error); }

// Balance on or off from the link, as the sketch does it. The simulation has no motor test or calibration to wait for.
bool setLinkBalance(const float *args, const char **error) {
  *error = ControlTick::setLinkBalance(control, args[0] >= 0.5f, false, false);
  return *error == nullptr;
}

}  // namespace

// Link counters of the simulated link: none are counted.
LinkCounters LinkCountersNow() { return {}; }

namespace sim {

// Read by link_server.cpp. Only uptime changes: the clock is the simulation clock.
void updateSystemHealth(uint32_t nowUs) { systemHealth.uptimeS = nowUs / 1000000u; }

}  // namespace sim

// The entry table of the link, expanded from signals.def.h after the groups and handlers it names.
#define SIGNAL_DEF_FILE "signals.def.h"
#include "signal_table_expand.h"

namespace sim {

const SignalEntry *signalTableEntries() { return kSignals; }

size_t signalTableCount() { return kSignalCount; }

}  // namespace sim
