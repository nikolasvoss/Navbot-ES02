// The scenario file and the --set parameters (plan, Model: Scenario file).

#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <sstream>

#include "sim.h"

namespace sim {
namespace {

constexpr long long kChannelCount = ControlTick::kRcChannelCount;
constexpr long long kSbusRawMax = 2047;  // S.BUS values are 11 bits

std::vector<std::string> tokenize(std::string line) {
  const size_t hash = line.find('#');
  if (hash != std::string::npos) line.erase(hash);
  std::istringstream in(line);
  std::vector<std::string> tokens;
  for (std::string token; in >> token;) tokens.push_back(token);
  return tokens;
}

bool parseNumber(const std::string &text, double &value) {
  char *end = nullptr;
  value = std::strtod(text.c_str(), &end);
  return !text.empty() && *end == '\0' && std::isfinite(value);
}

bool parseInteger(const std::string &text, long long &value) {
  char *end = nullptr;
  value = std::strtoll(text.c_str(), &end, 10);
  return !text.empty() && *end == '\0';
}

bool parseSwitch(const std::string &text, bool &on) {
  if (text != "on" && text != "off") return false;
  on = text == "on";
  return true;
}

// Checks the arguments of one step. error gets the reason when it returns false.
bool checkStep(const ScenarioStep &step, std::string &error) {
  const std::vector<std::string> &args = step.args;
  const std::string &command = step.command;
  long long channel = 0;
  long long raw = 0;
  double number = 0.0;
  bool on = false;
  if (command == "rc") {
    if (args.size() == 2 && parseInteger(args[0], channel) && parseInteger(args[1], raw) && channel >= 1 &&
        channel <= kChannelCount && raw >= 0 && raw <= kSbusRawMax) {
      return true;
    }
    error = "rc takes a channel 1-18 and a raw value 0-2047";
  } else if (command == "rc_frames" || command == "rc_failsafe" || command == "lift") {
    if (args.size() == 1 && parseSwitch(args[0], on)) return true;
    error = command + " takes on or off";
  } else if (command == "push") {
    if (args.size() == 1 && parseNumber(args[0], number)) return true;
    error = "push takes an impulse in N s";
  } else if (command == "battery") {
    if (args.size() == 1 && parseNumber(args[0], number) && number > 0.0) return true;
    error = "battery takes a voltage above 0";
  } else if (command == "end") {
    if (args.empty()) return true;
    error = "end takes no arguments";
  } else {
    error = "unknown command '" + command + "'";
  }
  return false;
}

}  // namespace

bool parseScenario(const std::string &text, std::vector<ScenarioStep> &steps, std::string &error) {
  steps.clear();
  std::istringstream lines(text);
  std::string line;
  for (int lineNo = 1; std::getline(lines, line); ++lineNo) {
    const std::vector<std::string> tokens = tokenize(line);
    if (tokens.empty()) continue;
    const std::string prefix = "line " + std::to_string(lineNo) + ": ";
    if (!steps.empty() && steps.back().command == "end") {
      error = prefix + "nothing may follow end";
      return false;
    }
    ScenarioStep step;
    if (tokens.size() < 2 || !parseNumber(tokens[0], step.tSec) || step.tSec < 0.0 ||
        (!steps.empty() && step.tSec < steps.back().tSec)) {
      error = prefix + "expected '<t> <command> [arguments]' with t >= 0, not before the line above";
      return false;
    }
    step.command = tokens[1];
    step.args.assign(tokens.begin() + 2, tokens.end());
    if (!checkStep(step, error)) {
      error = prefix + error;
      return false;
    }
    steps.push_back(step);
  }
  if (steps.empty() || steps.back().command != "end") {
    error = "the scenario must end with a '<t> end' line";
    return false;
  }
  return true;
}

bool applyStep(Robot &robot, const ScenarioStep &step, std::string &error) {
  if (!checkStep(step, error)) return false;
  const std::vector<std::string> &args = step.args;
  const std::string &command = step.command;
  long long channel = 0;
  long long raw = 0;
  double number = 0.0;
  bool on = false;
  if (command == "rc") {
    parseInteger(args[0], channel);
    parseInteger(args[1], raw);
    robot.rc().channels[channel - 1] = static_cast<int16_t>(raw);
  } else if (command == "rc_frames") {
    parseSwitch(args[0], on);
    robot.rc().framesOn = on;
  } else if (command == "rc_failsafe") {
    parseSwitch(args[0], on);
    robot.rc().signalOk = !on;
  } else if (command == "push") {
    parseNumber(args[0], number);
    robot.push(number);
  } else if (command == "lift") {
    parseSwitch(args[0], on);
    robot.setLifted(on);
  } else if (command == "battery") {
    parseNumber(args[0], number);
    robot.battery().openCircuitV = number;
  }
  return true;
}

namespace {

bool setNumber(const std::string &text, double &field) {
  double value = 0.0;
  if (!parseNumber(text, value)) return false;
  field = value;
  return true;
}

// An integer field of type T at least minValue.
template <typename T>
bool setInteger(const std::string &text, T &field, long long minValue) {
  long long value = 0;
  if (!parseInteger(text, value) || value < minValue || value > std::numeric_limits<T>::max()) return false;
  field = static_cast<T>(value);
  return true;
}

bool setFlag(const std::string &text, bool &field) {
  if (text != "0" && text != "1") return false;
  field = text == "1";
  return true;
}

using Setter = bool (*)(Params &, const std::string &);

const std::map<std::string, Setter> &setters() {
  static const std::map<std::string, Setter> table = {
      {"plant.bodyMassKg", [](Params &p, const std::string &v) { return setNumber(v, p.plant.bodyMassKg); }},
      {"plant.wheelMassKg", [](Params &p, const std::string &v) { return setNumber(v, p.plant.wheelMassKg); }},
      {"plant.wheelInertiaKgM2", [](Params &p, const std::string &v) { return setNumber(v, p.plant.wheelInertiaKgM2); }},
      {"plant.comHeightM", [](Params &p, const std::string &v) { return setNumber(v, p.plant.comHeightM); }},
      {"plant.bodyPitchInertiaKgM2",
       [](Params &p, const std::string &v) { return setNumber(v, p.plant.bodyPitchInertiaKgM2); }},
      {"plant.bodyYawInertiaKgM2", [](Params &p, const std::string &v) { return setNumber(v, p.plant.bodyYawInertiaKgM2); }},
      {"plant.wheelRadiusM", [](Params &p, const std::string &v) { return setNumber(v, p.plant.wheelRadiusM); }},
      {"plant.trackWidthM", [](Params &p, const std::string &v) { return setNumber(v, p.plant.trackWidthM); }},
      {"plant.yawFrictionNmPerRadPerSec",
       [](Params &p, const std::string &v) { return setNumber(v, p.plant.yawFrictionNmPerRadPerSec); }},
      {"plant.bearingFrictionNmPerRadPerSec",
       [](Params &p, const std::string &v) { return setNumber(v, p.plant.bearingFrictionNmPerRadPerSec); }},
      {"plant.groundContactPitchRad",
       [](Params &p, const std::string &v) { return setNumber(v, p.plant.groundContactPitchRad); }},
      {"plant.gravityMps2", [](Params &p, const std::string &v) { return setNumber(v, p.plant.gravityMps2); }},
      {"leftMotor.phaseResistanceOhm", [](Params &p, const std::string &v) { return setNumber(v, p.leftMotor.phaseResistanceOhm); }},
      {"leftMotor.kvRpmPerV", [](Params &p, const std::string &v) { return setNumber(v, p.leftMotor.kvRpmPerV); }},
      {"leftMotor.torquePerKe", [](Params &p, const std::string &v) { return setNumber(v, p.leftMotor.torquePerKe); }},
      {"rightMotor.phaseResistanceOhm",
       [](Params &p, const std::string &v) { return setNumber(v, p.rightMotor.phaseResistanceOhm); }},
      {"rightMotor.kvRpmPerV", [](Params &p, const std::string &v) { return setNumber(v, p.rightMotor.kvRpmPerV); }},
      {"rightMotor.torquePerKe", [](Params &p, const std::string &v) { return setNumber(v, p.rightMotor.torquePerKe); }},
      {"sensors.imuPitchOffsetRad", [](Params &p, const std::string &v) { return setNumber(v, p.sensors.imuPitchOffsetRad); }},
      {"sensors.imuHeightM", [](Params &p, const std::string &v) { return setNumber(v, p.sensors.imuHeightM); }},
      {"sensors.gyroNoiseRadPerSec", [](Params &p, const std::string &v) { return setNumber(v, p.sensors.gyroNoiseRadPerSec); }},
      {"sensors.gyroBiasRadPerSec.x", [](Params &p, const std::string &v) { return setNumber(v, p.sensors.gyroBiasRadPerSec[0]); }},
      {"sensors.gyroBiasRadPerSec.y", [](Params &p, const std::string &v) { return setNumber(v, p.sensors.gyroBiasRadPerSec[1]); }},
      {"sensors.gyroBiasRadPerSec.z", [](Params &p, const std::string &v) { return setNumber(v, p.sensors.gyroBiasRadPerSec[2]); }},
      {"sensors.accelNoiseG", [](Params &p, const std::string &v) { return setNumber(v, p.sensors.accelNoiseG); }},
      {"sensors.imuPeriodUs", [](Params &p, const std::string &v) { return setInteger(v, p.sensors.imuPeriodUs, 1); }},
      {"sensors.encoderCountsPerTurn", [](Params &p, const std::string &v) { return setInteger(v, p.sensors.encoderCountsPerTurn, 1); }},
      {"sensors.batteryAdcNoiseCounts", [](Params &p, const std::string &v) { return setNumber(v, p.sensors.batteryAdcNoiseCounts); }},
      {"sensors.flipGyroX", [](Params &p, const std::string &v) { return setFlag(v, p.sensors.flipGyroX); }},
      {"battery.openCircuitV", [](Params &p, const std::string &v) { return setNumber(v, p.battery.openCircuitV); }},
      {"battery.internalResistanceOhm",
       [](Params &p, const std::string &v) { return setNumber(v, p.battery.internalResistanceOhm); }},
      {"battery.otherLoadA", [](Params &p, const std::string &v) { return setNumber(v, p.battery.otherLoadA); }},
      {"timing.passUs", [](Params &p, const std::string &v) { return setInteger(v, p.timing.passUs, 1); }},
      {"timing.passJitterUs", [](Params &p, const std::string &v) { return setInteger(v, p.timing.passJitterUs, 0); }},
      {"timing.physicsStepUs", [](Params &p, const std::string &v) { return setInteger(v, p.timing.physicsStepUs, 1); }},
      {"seed", [](Params &p, const std::string &v) { return setInteger(v, p.seed, 0); }},
      {"flipMotorSign", [](Params &p, const std::string &v) { return setFlag(v, p.flipMotorSign); }},
  };
  return table;
}

}  // namespace

bool setParam(Params &params, const std::string &name, const std::string &value) {
  const std::map<std::string, Setter> &table = setters();
  const auto found = table.find(name);
  return found != table.end() && found->second(params, value);
}

}  // namespace sim
