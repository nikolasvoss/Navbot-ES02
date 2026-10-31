// The sketch's balance path on the simulation clock, wired to the plant, the motors, the battery and the sensors (plan,
// Model: Battery and One pass).

#include <Arduino.h>

#include <algorithm>
#include <cmath>
#include <random>

#include "MathConstants.h"
#include "RobotConfig.h"
#include "WheelBoundary.h"
#include "filter.h"
#include "imu.h"
#include "sim.h"

// The sketch's globals that the balance path and signals.def.h read (OllieFOCdrive.ino).
ControlTick::State control;
Config balanceSettings;
sim::SimMotor motor1;  // left wheel
sim::SimMotor motor2;  // right wheel
float m1FilteredVelocityRadPerSec = 0;
float m2FilteredVelocityRadPerSec = 0;
float controlTimestepSec = 0;
uint32_t controlStartUs = 0;
uint32_t controlExecutionUs = 0;
uint32_t loopEntryIntervalUs = 0;

namespace sim {
namespace {

constexpr double kMinBusV = 0.1;                 // the bus model never goes below this, so no division by zero
constexpr float kWheelSpeedFilterTfSec = 0.01f;  // the sketch's m1VelocityFilter and m2VelocityFilter, LowPassFilter(0.01)

// Motor-local sign of one wheel: WheelBoundary's sign, reversed when flipMotorSign.
double motorSign(bool left, bool flipMotorSign) {
  const double sign = left ? WheelBoundary::kLeftMotorSign : WheelBoundary::kRightMotorSign;
  return flipMotorSign ? -sign : sign;
}

// SimpleFOC's LowPassFilter(Tf) with the time of the control tick. A step longer than 0.3 s passes the input through.
float wheelSpeedFilter(float previous, float x, float dtSec) {
  if (dtSec > 0.3f) return x;
  const float alpha = kWheelSpeedFilterTfSec / (kWheelSpeedFilterTfSec + dtSec);
  return alpha * previous + (1.0f - alpha) * x;
}

ControlTick::MotorFeedback feedbackOf(const SimMotor &motor) {
  return {motor.voltage.q, motor.voltage_limit, motor.target, motor.current_limit};
}

// The sketch's applyBalanceOutput().
void applyWheelCommand(const ControlTick::WheelCommand &command) {
  switch (command.action) {
    case ControlTick::WheelAction::Keep:
      return;
    case ControlTick::WheelAction::Disable:
      motor1.disable();
      motor2.disable();
      return;
    case ControlTick::WheelAction::Drive:
      motor1.enable();
      motor2.enable();
      motor1.target = command.motor1Target;
      motor2.target = command.motor2Target;
      return;
  }
}

// The motor-local current and the forward torque of one wheel motor. A disabled motor carries no current.
struct WheelDrive {
  double currentA;
  double forwardTorqueNm;
};

WheelDrive wheelDrive(const SimMotor &motor, const MotorPhysics &physics, double sign, double wheelRadPerSec,
                      double pitchRateRadPerSec, double busV) {
  if (!motor.enabled) return {0.0, 0.0};
  const double relativeSpeed = sign * (wheelRadPerSec - pitchRateRadPerSec);
  const MotorOutput out = motorOutput(physics, appliedVoltageQ(motor, busV), relativeSpeed);
  return {out.currentA, sign * out.torqueNm};
}

// One physics step of dtSec: the bus from the previous step's currents, the motor currents and torques, the plant.
void physicsStep(const Params &params, PlantState &plant, Truth &truth, double dtSec) {
  const double busPrev = std::max(truth.busV, kMinBusV);
  const double power = 1.5 * (appliedVoltageQ(motor1, busPrev) * truth.leftCurrentA +
                              appliedVoltageQ(motor2, busPrev) * truth.rightCurrentA);
  const double loadA = params.battery.otherLoadA + std::max(0.0, power) / busPrev;
  truth.busV = std::max(kMinBusV, params.battery.openCircuitV - params.battery.internalResistanceOhm * loadA);

  const double leftSign = motorSign(true, params.flipMotorSign);
  const double rightSign = motorSign(false, params.flipMotorSign);
  const WheelDrive left = wheelDrive(motor1, params.leftMotor, leftSign, plant.leftWheelRadPerSec,
                                     plant.pitchRateRadPerSec, truth.busV);
  const WheelDrive right = wheelDrive(motor2, params.rightMotor, rightSign, plant.rightWheelRadPerSec,
                                      plant.pitchRateRadPerSec, truth.busV);
  truth.leftCurrentA = left.currentA;
  truth.rightCurrentA = right.currentA;
  stepPlant(plant, params.plant, WheelTorques{left.forwardTorqueNm, right.forwardTorqueNm}, dtSec);
}

}  // namespace

RcInput defaultRc() {
  RcInput rc{};
  for (int16_t &channel : rc.channels) channel = 992;
  rc.channels[4] = rc.channels[5] = rc.channels[6] = 192;  // CH5 (position 0), CH6 and CH7 off
  return rc;
}

Robot::Robot(const Params &params) : params_(params), rc_(defaultRc()), rng_(params.seed) {
  truth_.busV = params.battery.openCircuitV;
}

void Robot::boot() {
  hal().nowUs = 0;
  balanceSettings = defaultBalanceConfig();
  biquadFilterInitLPF(&control.voltageFilterLpf, 50, 1000);
  control.legPoseDegrees = kLegSagDegrees;
  imu::begin(imu::Settings{}, imu::CalibrationValues{});
  configureSimMotor(motor1, kLeftWheelMotor);
  configureSimMotor(motor2, kRightWheelMotor);
  motor1.disable();
  motor2.disable();
}

bool Robot::pass() {
  const uint32_t t = hal().nowUs;
  const double sinceLastPassSec = hasPreviousPass_ ? (t - previousPassUs_) * 1e-6 : 0.0;
  if (hasPreviousPass_) loopEntryIntervalUs = t - previousPassUs_;
  previousPassUs_ = t;
  hasPreviousPass_ = true;

  // SimpleFOC move() and loopFOC() read the encoders: the motor-local angle relative to the body.
  const double leftSign = motorSign(true, params_.flipMotorSign);
  const double rightSign = motorSign(false, params_.flipMotorSign);
  focStep(motor1, encoderAngle(leftSign * (plant_.leftWheelRad - plant_.pitchRad), params_.sensors), sinceLastPassSec);
  focStep(motor2, encoderAngle(rightSign * (plant_.rightWheelRad - plant_.pitchRad), params_.sensors), sinceLastPassSec);

  imu::service();
  if (rc_.framesOn && t >= nextRcFrameUs_) {
    ControlTick::applyRcFrame(control, rc_.channels, t);
    nextRcFrameUs_ = t + rc_.framePeriodUs;
  }

  controlTimestepSec = static_cast<float>((t - previousControlTimeUs_) * 1e-6);
  const bool ticked = controlTimestepSec >= kControlTimestepMinSec;
  if (ticked) {
    controlStartUs = t;
    if (linkPassStart) linkPassStart(t, ControlTick::balanceRequested(control));
    ControlTick::readBattery(control, batteryAdc(truth_.busV, params_.sensors, rng_), t);
    if (ControlTick::updateWheels(control, motor1.shaftAngle(), motor2.shaftAngle(), t)) {
      m1FilteredVelocityRadPerSec =
          wheelSpeedFilter(m1FilteredVelocityRadPerSec, control.m1VelocityRadPerSec, controlTimestepSec);
      m2FilteredVelocityRadPerSec =
          wheelSpeedFilter(m2FilteredVelocityRadPerSec, control.m2VelocityRadPerSec, controlTimestepSec);
    }
    if (ControlTick::serviceOverspeed(control, millis())) {
      motor1.disable();
      motor2.disable();
    }
    const ControlTick::WheelCommand command =
        ControlTick::runBalance(control, balanceSettings, imu::latest(), feedbackOf(motor1), feedbackOf(motor2),
                                rc_.signalOk, false, t);
    applyWheelCommand(command);
    ControlTick::stepLegs(control, controlTimestepSec);
    controlExecutionUs = 0;
    if (linkPassEnd) linkPassEnd(t);
    previousControlTimeUs_ = t;
  }

  // The physics until the next pass. The pass takes passUs plus a uniform jitter.
  std::uniform_int_distribution<int32_t> jitter(-static_cast<int32_t>(params_.timing.passJitterUs),
                                                static_cast<int32_t>(params_.timing.passJitterUs));
  const int64_t passUs = static_cast<int64_t>(params_.timing.passUs) + jitter(rng_);
  const uint32_t end = t + static_cast<uint32_t>(std::max<int64_t>(1, passUs));
  for (uint32_t now = t; now < end;) {
    const uint32_t stepUs = std::min(params_.timing.physicsStepUs, end - now);
    now += stepUs;
    hal().nowUs = now;
    physicsStep(params_, plant_, truth_, stepUs * 1e-6);
    while (nextImuSampleUs_ <= now) {
      hal().imuSample = imuSample(plant_, params_.plant, params_.sensors, rng_);
      hal().imuReady = true;
      nextImuSampleUs_ += params_.sensors.imuPeriodUs;
    }
  }
  return ticked;
}

uint32_t Robot::nowUs() const { return hal().nowUs; }

PlantState &Robot::plant() { return plant_; }

RcInput &Robot::rc() { return rc_; }

BatteryParams &Robot::battery() { return params_.battery; }

const Truth &Robot::truth() const { return truth_; }

const Params &Robot::params() const { return params_; }

void Robot::push(double impulseNs) { applyPush(plant_, params_.plant, impulseNs); }

void Robot::setLifted(bool lifted) { plant_.lifted = lifted; }

}  // namespace sim
