// navbot_sim: the command line, the batch run with its CSV, and the exit codes (plan, Model: Command line and CSV).

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "MathConstants.h"
#include "imu.h"
#include "sim.h"

namespace {

const char *const kUsage =
    "usage: navbot_sim --scenario FILE [--out FILE.csv] [--every N] [--set NAME=VALUE ...]\n"
    "       navbot_sim --live --listen HOST:PORT [--scenario FILE] [--set NAME=VALUE ...]\n";

const char *const kCsvHeader =
    "t_us,x_m,v_mps,pitch_rad,pitch_rate_rad_s,yaw_rad,yaw_rate_rad_s,left_wheel_rad_s,right_wheel_rad_s,bus_v,"
    "left_current_a,right_current_a,tilt_meas_rad,balance_state,balance_fault,motor1_target_a,motor2_target_a,"
    "motors_enabled,overspeed_tripped,lifted,fallen";

struct Options {
  std::string scenario;
  std::string out;
  std::string listen;
  bool live = false;
  uint32_t every = 1;
  std::vector<std::string> sets;  // NAME=VALUE
};

// Returns an empty string, or the usage error.
std::string parseArguments(int argc, char **argv, Options &options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const bool hasValue = i + 1 < argc;
    if (arg == "--live") {
      options.live = true;
    } else if (arg == "--scenario" && hasValue) {
      options.scenario = argv[++i];
    } else if (arg == "--out" && hasValue) {
      options.out = argv[++i];
    } else if (arg == "--listen" && hasValue) {
      options.listen = argv[++i];
    } else if (arg == "--every" && hasValue) {
      char *end = nullptr;
      const unsigned long every = std::strtoul(argv[++i], &end, 10);
      if (*end != '\0' || every == 0) return "--every takes a whole number of 1 or more";
      options.every = static_cast<uint32_t>(every);
    } else if (arg == "--set" && hasValue) {
      options.sets.push_back(argv[++i]);
    } else {
      return "unknown or incomplete argument '" + arg + "'";
    }
  }
  if (options.live) {
    if (options.listen.empty()) return "--live needs --listen HOST:PORT";
    if (!options.out.empty() || options.every != 1) return "--out and --every are for batch runs";
  } else {
    if (options.scenario.empty()) return "--scenario FILE is required";
    if (!options.listen.empty()) return "--listen needs --live";
  }
  return "";
}

uint32_t toMicros(double tSec) { return static_cast<uint32_t>(std::llround(tSec * 1e6)); }

void writeRow(std::ostream &out, uint32_t tickUs, const sim::PlantState &plant, const sim::Truth &truth) {
  const float tilt = -imu::latest().correctedRollDegrees * kDegreesToRadiansF;  // ControlTick's input tilt
  out << tickUs << ',' << plant.xM << ',' << plant.vMps << ',' << plant.pitchRad << ',' << plant.pitchRateRadPerSec
      << ',' << plant.yawRad << ',' << plant.yawRateRadPerSec << ',' << plant.leftWheelRadPerSec << ','
      << plant.rightWheelRadPerSec << ',' << truth.busV << ',' << truth.leftCurrentA << ',' << truth.rightCurrentA
      << ',' << tilt << ',' << static_cast<int>(control.balanceOutput.state) << ','
      << static_cast<int>(control.balanceOutput.fault) << ',' << motor1.target << ',' << motor2.target << ','
      << (motor1.enabled || motor2.enabled ? 1 : 0) << ',' << (control.wheelOverspeed.tripped ? 1 : 0) << ','
      << (plant.lifted ? 1 : 0) << ',' << (plant.fallen ? 1 : 0) << '\n';
}

// Runs the scenario in batch mode. Returns the exit code.
int runBatch(sim::Robot &robot, const std::vector<sim::ScenarioStep> &steps, const Options &options) {
  std::ofstream file;
  std::ostream *out = &std::cout;
  if (!options.out.empty()) {
    file.open(options.out);
    if (!file) {
      std::cerr << "navbot_sim: cannot write " << options.out << "\n";
      return 2;
    }
    out = &file;
  }
  *out << std::setprecision(9) << kCsvHeader << '\n';

  const uint32_t endUs = toMicros(steps.back().tSec);  // the scenario ends with the end line
  size_t nextStep = 0;
  size_t printedLogs = 0;
  uint32_t ticks = 0;
  for (;;) {
    while (nextStep < steps.size() && toMicros(steps[nextStep].tSec) <= robot.nowUs()) {
      std::string error;
      if (!sim::applyStep(robot, steps[nextStep], error)) {
        std::cerr << "navbot_sim: " << error << "\n";
        return 2;
      }
      ++nextStep;
    }
    if (robot.nowUs() >= endUs) break;

    // The plant and the truth at the start of the pass are the values the control tick measures.
    const sim::PlantState plant = robot.plant();
    const sim::Truth truth = robot.truth();
    const uint32_t tickUs = robot.nowUs();
    if (robot.pass()) {
      if (ticks % options.every == 0) writeRow(*out, tickUs, plant, truth);
      ++ticks;
    }
    for (; printedLogs < sim::logLines().size(); ++printedLogs) {
      const sim::LogLine &line = sim::logLines()[printedLogs];
      std::cerr << line.tUs << ' ' << line.tag << ' ' << line.text << "\n";
    }
  }
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  Options options;
  const std::string usageError = parseArguments(argc, argv, options);
  if (!usageError.empty()) {
    std::cerr << "navbot_sim: " << usageError << "\n" << kUsage;
    return 2;
  }

  std::vector<sim::ScenarioStep> steps;
  if (!options.scenario.empty()) {
    std::ifstream file(options.scenario);
    if (!file) {
      std::cerr << "navbot_sim: cannot read " << options.scenario << "\n";
      return 2;
    }
    std::stringstream text;
    text << file.rdbuf();
    std::string error;
    if (!sim::parseScenario(text.str(), steps, error)) {
      std::cerr << "navbot_sim: " << error << "\n";
      return 2;
    }
  }

  sim::Params params;
  for (const std::string &set : options.sets) {
    const size_t equals = set.find('=');
    if (equals == std::string::npos || !sim::setParam(params, set.substr(0, equals), set.substr(equals + 1))) {
      std::cerr << "navbot_sim: unknown parameter or bad value in --set " << set << "\n";
      return 2;
    }
  }

  sim::Robot robot(params);
  robot.boot();
  if (options.live) return sim::runLive(robot, steps, options.listen);
  return runBatch(robot, steps, options);
}
