// The settings-group rules behind SetParam: a candidate is applied only when it passes the group's check, and a refused
// candidate leaves the running struct alone. The balance case is the cross-field rule that would latch InvalidConfig.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "BalanceController.h"
#include "Calibration.h"
#include "settings_groups.h"

namespace {

bool failed = false;

void Check(bool condition, const char* what) {
  std::printf("%s: %s\n", what, condition ? "ok" : "FAIL");
  if (!condition) failed = true;
}

// The firmware's balance settings with the values the controller tests use: valid, and trimEnvelopeRad below
// tiltRefLimitRad.
Config ValidConfig() {
  Config config;
  config.armingTimeoutUs = 1000000;
  config.armingTiltWindowRad = 0.05f;
  config.armingTiltRateRadPerSec = 0.1f;
  config.armingRequestSpeedMps = 0.05f;
  config.armingRequestYawRadPerSec = 0.05f;
  config.armingWheelSpeedRadPerSec = 5.0f;
  config.imuMaxAgeUs = 20000;
  config.wheelMaxAgeUs = 20000;
  config.busMaxAgeUs = 100000;
  config.dtMaxUs = 50000;
  config.fallAngleRad = 0.6f;
  config.fallDurationUs = 500000;
  config.wheelOverspeedRadPerSec = 100.0f;
  config.overspeedDurationUs = 200000;
  config.trimValidated = true;
  config.storedTrimDeg = 0.0f;
  config.fixedPoseReachable = true;
  config.phaseResistanceOhm = 22.0f;
  config.kvRating = 100.0f;
  config.voltageLimitV = 8.4f;
  config.currentLimitA = 5.0f;
  config.sagMarginV = 0.4f;
  config.modulationFactor = 1.0f;
  config.leftWheelSign = 1;
  config.rightWheelSign = 1;
  config.busMinV = 6.0f;
  config.busMaxV = 30.0f;
  config.speedCapMps = 1.0f;
  config.accelCapMps2 = 2.0f;
  config.brakeCapMps2 = 4.0f;
  config.yawRateCapRadPerSec = 0.5f;
  config.requestMaxAgeUs = 50000;
  config.wheelRadiusM = 0.1f;
  config.speedLoopPeriodUs = 20000;
  config.speedKpRadPerMps = 0.5f;
  config.speedKiRadPerMpsS = 0.2f;
  config.speedIntegralEnabled = false;
  config.tiltRefLimitRad = 0.3f;
  config.tiltRefSlewRadPerSec = 1.0f;
  config.tiltClipDurationUs = 1000000;
  config.reserveLowA = 0.1f;
  config.authorityDurationUs = 100000;
  config.trimEnvelopeRad = 0.2f;
  config.trimSlewRadPerSec = 0.1f;
  config.knobPickupBandRad = 0.02f;
  config.standstillWheelSpeedTolMps = 0.01f;
  config.standstillTiltErrorTolRad = 0.01f;
  config.transferToleranceRad = 1e-4f;
  config.saveRequestSpeedTolMps = 0.02f;
  config.saveRequestYawTolRadPerSec = 0.03f;
  config.demandScaleCapStart = 0.5f;
  return config;
}

Measurements StandingStill(uint32_t nowUs) {
  Measurements m{};
  m.imuSequence = 1;
  m.imuReadTimeUs = nowUs;
  m.imuValid = true;
  m.wheelReadTimeUs = nowUs;
  m.wheelsValid = true;
  m.busReadTimeUs = nowUs;
  m.busVoltageV = 7.4f;
  return m;
}

Request LiveRequest(bool enable, uint32_t nowUs) {
  Request request{};
  request.receivedTimeUs = nowUs;
  request.sourceValid = true;
  request.enableSwitch = enable;
  request.knobValid = true;
  return request;
}

// Switch on at 2 ms and held: the controller arms at 2 ms and runs at 3 ms if the running config is valid.
State DriveTo(const Config& config, FaultReason& fault) {
  BalanceController controller;
  controller.update(StandingStill(1000), LiveRequest(false, 1000), config, 1000);
  controller.update(StandingStill(2000), LiveRequest(true, 2000), config, 2000);
  const Output out = controller.update(StandingStill(3000), LiveRequest(true, 3000), config, 3000);
  fault = out.fault;
  return out.state;
}

void BalanceGroupApplies() {
  Config running = ValidConfig();
  Config candidate = running;
  candidate.speedKpRadPerMps = 0.12f;
  const char* error = nullptr;
  Check(ApplyBalanceSettings(running, candidate, &error) && error == nullptr && running.speedKpRadPerMps == 0.12f,
        "a valid candidate replaces the running struct");
}

void CrossFieldRuleRefusedWithoutLatching() {
  Config running = ValidConfig();
  Config candidate = running;
  candidate.trimEnvelopeRad = 0.4f;  // above tiltRefLimitRad 0.3
  const char* error = nullptr;
  const bool applied = ApplyBalanceSettings(running, candidate, &error);
  Check(!applied, "trimEnvelopeRad above tiltRefLimitRad is refused");
  Check(error != nullptr && std::strcmp(error, "out of range or breaks a cross-field rule") == 0,
        "the refusal names the rule");
  Check(running.trimEnvelopeRad == 0.2f, "the running trim envelope is unchanged after the refusal");

  // The controller would latch InvalidConfig on the refused candidate, so it must never be the running struct. The
  // running struct still drives to Run, so no fault is latched.
  FaultReason fault = FaultReason::None;
  const State state = DriveTo(running, fault);
  Check(state == State::Run && fault == FaultReason::None, "the running config still arms and runs");
  FaultReason refusedFault = FaultReason::None;
  DriveTo(candidate, refusedFault);
  Check(refusedFault == FaultReason::InvalidConfig, "the refused candidate would have latched InvalidConfig");
}

void NonFiniteAndRangeRefused() {
  Config running = ValidConfig();
  const char* error = nullptr;
  Config nan = running;
  nan.tiltPAPerRad = std::nanf("");
  Check(!ApplyBalanceSettings(running, nan, &error) && std::isnan(running.tiltPAPerRad) == false,
        "a NaN gain is refused and the running gain is kept");
  Config zeroTiltClip = running;
  zeroTiltClip.tiltClipDurationUs = 0;
  Check(!ApplyBalanceSettings(running, zeroTiltClip, &error) && running.tiltClipDurationUs == 1000000u,
        "a zero clip duration is refused");
}

void ServoTrimsApply() {
  ServoTrimsDegrees running{1.0f, 2.0f, 3.0f, 4.0f};
  const char* error = nullptr;
  ServoTrimsDegrees candidate{1.0f, 2.0f, 3.0f, 10.0f};
  Check(ApplyServoTrims(running, candidate, &error) && running.servo4Degrees == 10.0f,
        "trims at the plus 10 degree limit are applied");
  candidate.servo2Degrees = -10.5f;
  Check(!ApplyServoTrims(running, candidate, &error) && running.servo2Degrees == 2.0f,
        "a trim beyond minus 10 degrees is refused and the other trims stay");
  Check(error != nullptr && std::strcmp(error, "outside plus or minus 10 degrees") == 0,
        "the refusal names the limit");
  candidate.servo2Degrees = std::nanf("");
  Check(!ApplyServoTrims(running, candidate, &error), "a NaN trim is refused");
}

}  // namespace

int main() {
  BalanceGroupApplies();
  CrossFieldRuleRefusedWithoutLatching();
  NonFiniteAndRangeRefused();
  ServoTrimsApply();
  return failed ? 1 : 0;
}
