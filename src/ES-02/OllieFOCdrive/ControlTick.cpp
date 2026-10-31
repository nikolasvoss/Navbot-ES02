#include "ControlTick.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "BoardConfig.h"
#include "LegPoseRamp.h"
#include "Logging.h"
#include "MathConstants.h"
#include "OverspeedGuard.h"
#include "RobotConfig.h"
#include "WheelBoundary.h"
#include "rc_mode.h"

namespace ControlTick {
namespace {

// Raw S.BUS values (kSbusChannelMin to kSbusChannelMax, centre 992). Decided and measured 2026-10-22 (plan: RC map).
constexpr int kSbusChannelMax = 1792;
constexpr int kSbusChannelMin = 192;
constexpr int kRcCenterRaw = (kSbusChannelMax + kSbusChannelMin) / 2;     // 992
constexpr int kRcHalfRangeRaw = (kSbusChannelMax - kSbusChannelMin) / 2;  // 800
constexpr int kRcSwitchOnRaw = 992;   // CH6 and CH7: 2-position latching switches
constexpr int kRcKnobMinRaw = 172;    // CH9 trim knob: outside 172 to 1811 the knob is invalid
constexpr int kRcKnobMaxRaw = 1811;

// Measurement signs, checked on the supported robot 2026-10-24: leaning forward lowers correctedRollDegrees and drives
// gyro x below 0, so both are negated to make a forward lean positive (positive gains then push the wheels forward).
// Turning left raises gyro z, which is the positive yaw rate.
constexpr float kTiltSign = -1.0f;
constexpr float kTiltRateSign = -1.0f;
constexpr float kYawRateSign = 1.0f;

// Stick position in -1..1 from a raw S.BUS value, centred on kRcCenterRaw.
float rcStickFraction(int16_t raw) {
  const float fraction = static_cast<float>(raw - kRcCenterRaw) / kRcHalfRangeRaw;
  return std::max(-1.0f, std::min(1.0f, fraction));
}

// Balance is on or being turned on. The state is the balance controller's, not the Output's name.
bool balanceActive(const State &state) { return state.balanceOutput.state != ::State::Disabled; }

// Moves linkBalance with CH5. Entering position 1 starts it only while the controller drives; leaving position 1
// clears it and the stored Command. Called only while the RC frame is fresh.
void updateLinkBalance(State &state) {
  if (state.ch5Position == state.previousCh5Position) return;
  if (state.ch5Position == 1) {
    const bool driving =
        state.balanceOutput.state == ::State::Run || state.balanceOutput.state == ::State::Commissioning;
    state.linkBalance = RcMode::LinkBalanceOnEntry(driving);
  }
  if (state.previousCh5Position == 1) {
    state.linkBalance = false;
    state.linkCommand = {};
  }
  state.previousCh5Position = state.ch5Position;
}

// Balance measurements from the acquisition times and values in the state, the IMU snapshot and the motor feedback.
Measurements buildMeasurements(const State &state, const imu::Snapshot &imuSnapshot, const MotorFeedback &motor1,
                               const MotorFeedback &motor2) {
  Measurements m{};
  m.imuSequence = imuSnapshot.sequence;
  m.imuReadTimeUs = imuSnapshot.readTimeUs;
  m.imuValid = imuSnapshot.valid;
  m.tiltRad = kTiltSign * imuSnapshot.correctedRollDegrees * kDegreesToRadiansF;
  m.tiltRateRadPerSec = kTiltRateSign * imuSnapshot.angularVelocityRadPerSec.x;
  m.yawRateRadPerSec = kYawRateSign * imuSnapshot.angularVelocityRadPerSec.z;
  m.wheelReadTimeUs = state.wheelReadTimeUs;
  m.wheelsValid = state.wheelsValid;
  m.leftWheelRadPerSec = state.m1VelocityRadPerSec;
  m.rightWheelRadPerSec = state.m2VelocityRadPerSec;
  m.busReadTimeUs = state.busReadTimeUs;
  m.busVoltageV = state.batteryVoltage;
  // Approximation: near the usable voltage ceiling, or at the current limit of the last command.
  m.voltageClipped = std::fabs(motor1.voltageQ) >= motor1.voltageLimit - 0.01f ||
                     std::fabs(motor2.voltageQ) >= motor2.voltageLimit - 0.01f;
  m.currentClipped = std::fabs(motor1.target) >= motor1.currentLimit || std::fabs(motor2.target) >= motor2.currentLimit;
  return m;
}

// The balance request of this tick. Freshness and the receiver failsafe decide sourceValid; a stale frame is zero demand.
Request buildRequest(State &state, const Config &config, bool rcSignalOk, uint32_t nowUs) {
  const bool fresh = rcFrameFresh(state, nowUs);
  if (fresh) {
    // CH5 off rearms a tripped overspeed guard (once both wheels are below the limit), so no serial command is needed.
    if (state.ch5Position == 0) resetOverspeed(state);
    updateLinkBalance(state);
  }
  Request request{};
  request.receivedTimeUs = state.rcFrameUs;
  request.sourceValid = fresh && rcSignalOk;
  state.rcValid = request.sourceValid;
  if (state.ch5Position == 1) {
    // The link drives. A missing or old Command is zero demand; the controller keeps balancing in place. An old Command
    // is latched as expired: it is cleared here and stays cleared until a new Command arrives.
    if (RcMode::CommandExpired(state.linkCommand.seen, nowUs, state.linkCommand.receivedUs)) state.linkCommand = {};
    request.speedMps = state.linkCommand.seen ? state.linkCommand.speedMps : 0.0f;
    request.yawRateRadPerSec = state.linkCommand.seen ? state.linkCommand.turnRadPerSec : 0.0f;
  } else {
    request.speedMps = config.speedCapMps * rcStickFraction(state.rcChannels[2]);                   // CH3: forward
    request.yawRateRadPerSec = -config.yawRateCapRadPerSec * rcStickFraction(state.rcChannels[3]);  // CH4: left
  }
  // A guard trip reads as enable off, so the controller goes Disabled. The operator must switch CH5 off and on to rearm.
  request.enableSwitch = RcMode::EnableSwitch(state.ch5Position, state.linkBalance) && !state.wheelOverspeed.tripped;
  request.saveSwitch = state.rcChannels[5] >= kRcSwitchOnRaw;                                    // CH6
  request.commissioningSwitch = state.rcChannels[6] >= kRcSwitchOnRaw;                           // CH7
  request.knobValid = state.rcChannels[8] >= kRcKnobMinRaw && state.rcChannels[8] <= kRcKnobMaxRaw;  // CH9
  request.knobTrimDeg = rcStickFraction(state.rcChannels[8]) * config.trimEnvelopeRad * kRadiansToDegreesF;
  return request;
}

// Logs state and fault changes, save pulses, and refusal edges. Never logs per tick.
void logBalanceChange(const State &state, const Output &out) {
  if (out.state != state.balanceOutput.state || out.fault != state.balanceOutput.fault)
    Logging::message(out.state == ::State::Fault ? Logging::Level::Warning : Logging::Level::Info, "BALANCE",
                     "%s -> %s (%s)", StateName(state.balanceOutput.state), StateName(out.state),
                     FaultReasonName(out.fault));
  if (out.saveTrimRequested)  // one-tick pulse
    Logging::message(Logging::Level::Info, "BALANCE", "save requested trim=%.3f deg; storage not wired",
                     out.trimToSaveDeg);
  if (out.diag.saveRefused && !state.balanceOutput.diag.saveRefused)
    Logging::message(Logging::Level::Info, "BALANCE", "save refused; no pulse");
  if (out.diag.commissioningRefused && !state.balanceOutput.diag.commissioningRefused)
    Logging::message(Logging::Level::Info, "BALANCE", "commissioning transfer refused");
}

}  // namespace

void applyRcFrame(State &state, const int16_t (&channels)[kRcChannelCount], uint32_t nowUs) {
  for (int i = 0; i < kRcChannelCount; ++i) state.rcChannels[i] = channels[i];
  state.rcFrameUs = nowUs;
  state.rcFrameSeen = true;
  state.ch5Position = RcMode::Ch5Position(state.rcChannels[4]);
}

bool rcFrameFresh(const State &state, uint32_t nowUs) {
  return state.rcFrameSeen && static_cast<uint32_t>(nowUs - state.rcFrameUs) <= kRcFrameTimeoutMs * 1000U;
}

void readBattery(State &state, uint16_t adc, uint32_t nowUs) {
  state.busReadTimeUs = nowUs;
  state.voltageAdc = adc;
  state.batteryInstantRawV = board::kBatteryVoltsPerAdcCount * adc;
  state.voltageAdcFiltered = biquadFilterApply(&state.voltageFilterLpf, adc);
  state.batteryVoltage = board::kBatteryVoltsPerAdcCount * state.voltageAdcFiltered;
}

bool updateWheels(State &state, double motor1AngleRad, double motor2AngleRad, uint32_t readTimeUs) {
  WheelBoundary::WheelValues rawForward{};
  if (!state.wheelSpeedEstimator.update(motor1AngleRad, motor2AngleRad, readTimeUs, rawForward)) return false;
  state.wheelReadTimeUs = readTimeUs;
  state.wheelsValid = true;
  state.m1VelocityRadPerSec = rawForward.left;
  state.m2VelocityRadPerSec = rawForward.right;
  return true;
}

bool serviceOverspeed(State &state, uint32_t nowMs) {
  const bool wasTripped = state.wheelOverspeed.tripped;
  if (!OverspeedGuard::update(state.wheelOverspeed, nowMs, state.m1VelocityRadPerSec, state.m2VelocityRadPerSec))
    return false;
  if (!wasTripped)
    Logging::message(Logging::Level::Warning, "WHEEL_OVERSPEED", "peak=%.1frad_s limit=%.1frad_s; both drivers disabled",
                     state.wheelOverspeed.peakRadPerSec, OverspeedGuard::kLimitRadPerSec);
  return true;
}

void resetOverspeed(State &state) {
  if (OverspeedGuard::WithinLimit(state.m1VelocityRadPerSec, state.m2VelocityRadPerSec))
    OverspeedGuard::reset(state.wheelOverspeed);
}

bool balanceRequested(const State &state) { return state.ch5Position != 0 || balanceActive(state); }

uint32_t linkCommandAgeMs(const State &state, uint32_t nowUs) {
  return state.linkCommand.seen ? (nowUs - state.linkCommand.receivedUs) / 1000U : UINT32_MAX;
}

const char *acceptLinkCommand(State &state, float speedMps, float turnRadPerSec, uint32_t nowUs) {
  if (state.ch5Position != 1) return "CH5 not in link position (1)";
  if (state.previousCh5Position != 1) return "CH5 position change pending";
  if (!std::isfinite(speedMps) || !std::isfinite(turnRadPerSec)) return "speed and turn must be finite numbers";
  state.linkCommand = {true, speedMps, turnRadPerSec, nowUs};
  return nullptr;
}

const char *setLinkBalance(State &state, bool on, bool motorTestBusy, bool calibrationBusy) {
  if (state.ch5Position != 1) return "CH5 not in link position (1)";
  if (state.previousCh5Position != 1) return "CH5 position change pending";
  if (on) {
    if (state.wheelOverspeed.tripped) return "wheel overspeed latched, switch CH5 to 0";
    if (motorTestBusy) return "motor test running";
    if (calibrationBusy) return "calibration running";
  }
  state.linkBalance = on;
  return nullptr;
}

WheelCommand runBalance(State &state, const Config &config, const imu::Snapshot &imuSnapshot, MotorFeedback motor1,
                        MotorFeedback motor2, bool rcSignalOk, bool motorTestOwnsOutputs, uint32_t nowUs) {
  const Measurements measurements = buildMeasurements(state, imuSnapshot, motor1, motor2);
  const Request request = buildRequest(state, config, rcSignalOk, nowUs);
  const Output out = state.balanceController.update(measurements, request, config, nowUs);
  logBalanceChange(state, out);
  state.balanceOutput = out;
  state.movementSpeed = request.sourceValid ? request.speedMps : 0.0f;  // telemetry only
  state.bodyTurn = request.sourceValid ? request.yawRateRadPerSec : 0.0f;

  if (motorTestOwnsOutputs) return {WheelAction::Keep, 0.0f, 0.0f};
  // Drive needs driveEnabled and a clear overspeed guard. Otherwise both wheels get 0 A and are disabled.
  if (out.driveEnabled && !state.wheelOverspeed.tripped) {
    const WheelBoundary::MotorValues targets = WheelBoundary::toMotorTargets(out.leftA, out.rightA);
    return {WheelAction::Drive, targets.motor1, targets.motor2};
  }
  return {WheelAction::Disable, 0.0f, 0.0f};
}

void stepLegs(State &state, float controlTimestepSec) {
  state.legPoseDegrees = stepLegPoseToward(state.legPoseDegrees, kLegPoseDegrees, kLegRampDegPerSec,
                                           fminf(controlTimestepSec, kLegRampMaxDtSec));
  for (int &angle : state.servoTraceAngle) angle = static_cast<int>(lroundf(state.legPoseDegrees));
}

}  // namespace ControlTick
