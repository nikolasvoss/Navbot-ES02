#pragma once

// The balance path of the control tick without Arduino, SimpleFOC or FreeRTOS: the RC frame and the link requests, the
// measurements, the balance controller, the overspeed guard, the battery filter and the leg pose. The sketch and the host
// simulation (src/sim/) call it the same way; the hardware stays with the caller. signals.def.h reads the fields of State.

#include <cstdint>

#include "BalanceController.h"
#include "OverspeedGuard.h"
#include "WheelBoundary.h"
#include "filter.h"
#include "imu.h"

namespace ControlTick {

constexpr int kRcChannelCount = 18;

// The last Command accepted in CH5 position 1.
struct LinkCommand {
  bool seen;
  float speedMps;
  float turnRadPerSec;
  uint32_t receivedUs;
};

// Everything the tick keeps between calls. The sketch and the simulation each have one instance named `control`.
struct State {
  // Remote control: the last S.BUS frame, raw (192 to 1792, centre 992), and the CH5 and link state.
  int16_t rcChannels[kRcChannelCount] = {};
  bool rcFrameSeen = false;
  uint32_t rcFrameUs = 0;            // nowUs of applyRcFrame()
  uint8_t ch5Position = 0;           // 0, 1 (link) or 2 (RC), from the last frame (rc_mode.h)
  uint8_t previousCh5Position = 0;   // the position the last fresh RC frame had
  bool rcValid = false;              // telemetry: the frame is fresh and the receiver reports no failsafe
  bool linkBalance = false;          // the link holds balance on, in CH5 position 1
  LinkCommand linkCommand{};

  // Balance controller
  BalanceController balanceController;
  Output balanceOutput{};            // result of the last update()
  float movementSpeed = 0.0f;        // telemetry: the speed request of the last tick, 0 without a valid source
  float bodyTurn = 0.0f;             // telemetry: the yaw-rate request of the last tick

  // Wheels: forward-positive speeds from the estimator (WheelBoundary.h)
  WheelBoundary::WheelSpeedEstimator wheelSpeedEstimator;
  uint32_t wheelReadTimeUs = 0;
  bool wheelsValid = false;
  float m1VelocityRadPerSec = 0.0f;  // left
  float m2VelocityRadPerSec = 0.0f;  // right
  OverspeedGuard::State wheelOverspeed{};

  // Battery: the caller designs voltageFilterLpf (biquadFilterInitLPF) before the first readBattery().
  biquadFilter_t voltageFilterLpf{};
  uint16_t voltageAdc = 0;
  float voltageAdcFiltered = 0.0f;
  float batteryVoltage = 0.0f;       // filtered
  float batteryInstantRawV = 0.0f;   // unfiltered
  uint32_t busReadTimeUs = 0;

  // Legs: the commanded pose, ramped toward kLegPoseDegrees (RobotConfig.h), and the four servo arguments.
  float legPoseDegrees = 0.0f;
  int servoTraceAngle[4] = {0, 0, 0, 0};
};

// The SimpleFOC fields of one wheel motor that the clip flags of the measurements read. Motor-local sign.
struct MotorFeedback {
  float voltageQ;      // motor.voltage.q
  float voltageLimit;  // motor.voltage_limit
  float target;        // motor.target
  float currentLimit;  // motor.current_limit
};

// What the caller does with both wheel drivers after runBalance().
enum class WheelAction : uint8_t {
  Keep,     // a motor test owns the outputs: change nothing
  Disable,  // targets 0 A, then disable both drivers
  Drive,    // enable both drivers, then set the targets
};

struct WheelCommand {
  WheelAction action;
  float motor1Target;  // A, motor-local sign (WheelBoundary::toMotorTargets), 0 unless action is Drive
  float motor2Target;
};

// A decoded S.BUS frame (the sketch's applyRcChannels): stores the channels, the arrival time and the CH5 position.
void applyRcFrame(State &state, const int16_t (&channels)[kRcChannelCount], uint32_t nowUs);

// A frame was seen and is at most kRcFrameTimeoutMs old (the sketch's rcFrameFresh).
bool rcFrameFresh(const State &state, uint32_t nowUs);

// The battery reading (the sketch's readBatteryVoltage) without the ADC read. adc is the raw count.
void readBattery(State &state, uint16_t adc, uint32_t nowUs);

// The wheel speeds (the sketch's updateWheelVelocities) from SimpleFOC shaftAngle() of motor1 and motor2. Returns
// true when the speeds were updated; then wheelReadTimeUs is readTimeUs and wheelsValid is true.
bool updateWheels(State &state, double motor1AngleRad, double motor2AngleRad, uint32_t readTimeUs);

// The overspeed guard (the sketch's serviceWheelOverspeed). Returns true while it is tripped; the caller then sets 0 A
// and disables both drivers. Logs the trip once, as the sketch does.
bool serviceOverspeed(State &state, uint32_t nowMs);

// Rearms the guard while both wheels are within the limit (the sketch's resetWheelOverspeed).
void resetOverspeed(State &state);

// CH5 is not 0, or the controller is not Disabled. Motor tests, calibration, reboot and SetParam wait for it.
bool balanceRequested(const State &state);

// Milliseconds since the last accepted Command; UINT32_MAX before the first one.
uint32_t linkCommandAgeMs(const State &state, uint32_t nowUs);

// A Command from the link (the sketch's handleLinkCommand without the diagnostic-build check). Returns nullptr when
// accepted, or the refusal text, in the sketch's order.
const char *acceptLinkCommand(State &state, float speedMps, float turnRadPerSec, uint32_t nowUs);

// The balance action (the sketch's setLinkBalance without the diagnostic-build check). motorTestBusy is a motor test
// active or still applied; calibrationBusy is a calibration step selected. Returns nullptr or the refusal text.
const char *setLinkBalance(State &state, bool on, bool motorTestBusy, bool calibrationBusy);

// The balance part of the control tick, in the sketch's order: measurements, request, BalanceController::update(), the
// state-change log lines, telemetry, and the wheel command. rcSignalOk is the receiver's failsafe flag
// (sBus.Failsafe() == kSignalOk). motorTestOwnsOutputs is a motor test active or still applied. nowUs is read after
// the measurements, so no measurement timestamp is later than nowUs.
WheelCommand runBalance(State &state, const Config &config, const imu::Snapshot &imuSnapshot, MotorFeedback motor1,
                        MotorFeedback motor2, bool rcSignalOk, bool motorTestOwnsOutputs, uint32_t nowUs);

// One step of the leg pose ramp toward kLegPoseDegrees; sets servoTraceAngle to the rounded pose.
// controlTimestepSec is capped at kLegRampMaxDtSec inside.
void stepLegs(State &state, float controlTimestepSec);

}  // namespace ControlTick
