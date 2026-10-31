#include <algorithm>
#include <cmath>

#include "sim.h"

namespace sim {
namespace {

constexpr double kRpmToRadPerSec = 0.10471975512;
// SimpleFOC's current filter on ESP32 (DEF_CURR_FILTER_Tf), as the plan states it.
constexpr double kCurrentFilterTfSec = 1.0 / (2.0 * 3.14159265358979323846 * 300.0);

// SimpleFOC LowPassFilter with a known dt: a step longer than 0.3 s passes the input through.
double lowPass(double previous, double x, double tfSec, double dtSec) {
  if (dtSec > 0.3) return x;
  const double alpha = tfSec / (tfSec + dtSec);
  return alpha * previous + (1.0 - alpha) * x;
}

// Volts of back-EMF per rad/s of shaft speed, with SimpleFOC's KV convention.
double backEmfPerRadPerSec(double kvRpmPerV) { return 1.0 / (kvRpmPerV * std::sqrt(3.0) * kRpmToRadPerSec); }

}  // namespace

void SimMotor::disable() {
  enabled = false;
  target = 0.0f;
  current_sp = 0.0f;
  currentQ = 0.0f;
  voltage.q = 0.0f;
  voltage.d = 0.0f;
}

void configureSimMotor(SimMotor &motor, const WheelMotorParams &params) {
  motor.phase_resistance = params.phaseResistanceOhm;
  motor.KV_rating = params.kvRatingRpmPerV * kWheelKvSafetyFactor;
  motor.voltage_limit = kWheelVoltageLimitV;
  motor.current_limit = kWheelVoltageLimitV / params.phaseResistanceOhm;
}

void focStep(SimMotor &motor, double sensorAngleRad, double dtSec) {
  if (motor.hasPreviousAngle) {
    const double rawVelocity = (sensorAngleRad - motor.previousAngleRad) / dtSec;
    motor.shaft_velocity = static_cast<float>(
        lowPass(motor.shaft_velocity, rawVelocity, kWheelVelocityFilterTfSec, dtSec));
  } else {
    motor.shaft_velocity = 0.0f;
  }
  motor.hasPreviousAngle = true;
  motor.previousAngleRad = sensorAngleRad;
  motor.shaft_angle = static_cast<float>(sensorAngleRad);
  if (!motor.enabled) return;

  const double limit = motor.current_limit;
  const double currentSp = std::clamp(static_cast<double>(motor.target), -limit, limit);
  motor.current_sp = static_cast<float>(currentSp);
  const double currentQ = lowPass(motor.currentQ, currentSp, kCurrentFilterTfSec, dtSec);
  motor.currentQ = static_cast<float>(currentQ);

  const double voltageBemf = motor.shaft_velocity * backEmfPerRadPerSec(motor.KV_rating);
  const double voltageLimit = motor.voltage_limit;
  const double voltageQ = std::clamp(currentQ * motor.phase_resistance + voltageBemf, -voltageLimit, voltageLimit);
  motor.voltage.q = static_cast<float>(voltageQ);
}

double appliedVoltageQ(const SimMotor &motor, double busV) {
  if (!motor.enabled) return 0.0;
  const double applied = motor.voltage.q * busV / motor.voltage_power_supply;
  const double modulationLimit = busV / std::sqrt(3.0);
  return std::clamp(applied, -modulationLimit, modulationLimit);
}

MotorOutput motorOutput(const MotorPhysics &physics, double appliedVq, double relativeSpeedRadPerSec) {
  const double ke = backEmfPerRadPerSec(physics.kvRpmPerV);
  const double currentA = (appliedVq - ke * relativeSpeedRadPerSec) / physics.phaseResistanceOhm;
  return {currentA, physics.torquePerKe * ke * currentA};
}

}  // namespace sim
