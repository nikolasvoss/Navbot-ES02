#include "BalanceController.h"

#include <algorithm>
#include <cmath>

#include "MathConstants.h"

namespace {
// Equals SimpleFOC's _RPM_TO_RADS: radians per second per rpm.
constexpr float kRpmToRadPerSec = 0.10471975512f;

struct WheelInterval {
  float minA;
  float maxA;
  bool finite;
};

bool isUnitSign(int sign) { return sign == 1 || sign == -1; }

// Forward-positive current interval of one wheel. The motor sees sign * forward speed, and forward
// current is sign * motor current, so a -1 wheel mirrors the motor-frame interval.
WheelInterval wheelInterval(float forwardRadPerSec, int sign, float vMaxV, const Config &config) {
  const float bemfV = (sign * forwardRadPerSec) / (config.kvRating * std::sqrt(3.0f)) / kRpmToRadPerSec;
  const float maxMotorA = std::min(config.currentLimitA, (vMaxV - bemfV) / config.phaseResistanceOhm);
  const float minMotorA = std::max(-config.currentLimitA, (-vMaxV - bemfV) / config.phaseResistanceOhm);
  WheelInterval out = sign > 0 ? WheelInterval{minMotorA, maxMotorA, false}
                               : WheelInterval{-maxMotorA, -minMotorA, false};
  // std::min and std::max would hide a NaN back-EMF behind the current limit, so check it here.
  out.finite = std::isfinite(bemfV) && std::isfinite(out.minA) && std::isfinite(out.maxA);
  return out;
}

bool elapsedLongerThan(uint32_t nowUs, uint32_t startUs, uint32_t durationUs) {
  return static_cast<uint32_t>(nowUs - startUs) > durationUs;
}

// A timestamp ahead of nowUs wraps to a huge unsigned age and counts as stale.
bool isStale(uint32_t nowUs, uint32_t readTimeUs, uint32_t maxAgeUs) {
  return static_cast<uint32_t>(nowUs - readTimeUs) > maxAgeUs;
}

bool armingConditionsMet(const Measurements &measurements, const Request &request, const Config &config,
                         uint32_t nowUs) {
  const float trimRad = config.storedTrimDeg * kDegreesToRadiansF;
  return measurements.imuValid && measurements.wheelsValid &&
         !isStale(nowUs, measurements.imuReadTimeUs, config.imuMaxAgeUs) &&
         !isStale(nowUs, measurements.wheelReadTimeUs, config.wheelMaxAgeUs) &&
         !isStale(nowUs, measurements.busReadTimeUs, config.busMaxAgeUs) &&
         std::fabs(measurements.tiltRad - trimRad) <= config.armingTiltWindowRad &&
         std::fabs(measurements.tiltRateRadPerSec) <= config.armingTiltRateRadPerSec &&
         std::fabs(request.speedMps) <= config.armingRequestSpeedMps &&
         std::fabs(request.yawRateRadPerSec) <= config.armingRequestYawRadPerSec;
}

// Demand caps must be usable: a zero brake cap would hold the last speed when the request times out,
// and a negative or NaN rate would move speed the wrong way.
bool demandConfigValid(const Config &config) {
  return std::isfinite(config.speedCapMps) && config.speedCapMps >= 0.0f &&
         std::isfinite(config.accelCapMps2) && config.accelCapMps2 > 0.0f &&
         std::isfinite(config.brakeCapMps2) && config.brakeCapMps2 > 0.0f &&
         std::isfinite(config.yawRateCapRadPerSec) && config.yawRateCapRadPerSec >= 0.0f;
}

float nanToZero(float value) { return std::isnan(value) ? 0.0f : value; }

// Moves speed toward target within one step. Growing the magnitude uses accelRate; shrinking it uses
// brakeRate. A reversal brakes to zero and spends the rest of the step accelerating the other way.
float shapeSpeed(float previous, float target, float accelRate, float brakeRate, float dtSec) {
  float speed = previous;
  float timeLeftSec = dtSec;
  const bool shrinking = (speed > 0.0f && target < speed) || (speed < 0.0f && target > speed);
  if (shrinking) {
    // Brake to the target when it lies on this side of zero, otherwise to zero.
    const float goal = speed > 0.0f ? std::max(target, 0.0f) : std::min(target, 0.0f);
    if (brakeRate <= 0.0f) {
      return speed;
    }
    const float brakeTimeSec = std::fabs(speed - goal) / brakeRate;
    if (brakeTimeSec > timeLeftSec) {
      return speed + (goal > speed ? brakeRate : -brakeRate) * timeLeftSec;
    }
    speed = goal;
    timeLeftSec -= brakeTimeSec;
  }
  const float stepSpeed = std::min(std::fabs(target - speed), accelRate * timeLeftSec);
  return speed + (target > speed ? stepSpeed : -stepSpeed);
}
}  // namespace

Demand BalanceController::shapeDemand(Demand previous, Demand requested, float demandScale, float dtSec,
                                      const Config &config) {
  const float scale = std::isnan(demandScale) ? 0.0f : std::clamp(demandScale, 0.0f, 1.0f);
  const float speedCapMps = config.speedCapMps * scale;
  const float yawCapRadPerSec = config.yawRateCapRadPerSec * scale;

  const float speedTarget = std::max(-speedCapMps, std::min(speedCapMps, nanToZero(requested.speedMps)));
  const float yawRate =
      std::max(-yawCapRadPerSec, std::min(yawCapRadPerSec, nanToZero(requested.yawRateRadPerSec)));

  Demand shaped;
  shaped.speedMps = shapeSpeed(previous.speedMps, speedTarget, config.accelCapMps2 * scale,
                               config.brakeCapMps2, dtSec);
  shaped.yawRateRadPerSec = yawRate;
  return shaped;
}

Output BalanceController::update(const Measurements &measurements, const Request &request, const Config &config,
                                 uint32_t nowUs) {
  // The first call only records the switch level, so a switch that is already on at boot is not an edge.
  const bool enableEdge = enableLevelKnown_ && request.enableSwitch && !previousEnableSwitch_;
  enableLevelKnown_ = true;
  previousEnableSwitch_ = request.enableSwitch;

  if (!request.enableSwitch) {
    state_ = State::Disabled;
    fault_ = FaultReason::None;
  } else {
    switch (state_) {
      case State::Disabled:
        if (enableEdge && config.trimValidated && config.fixedPoseReachable && fault_ == FaultReason::None) {
          state_ = State::Arming;
          armingStartUs_ = nowUs;
        }
        break;
      case State::Arming:
        if (armingConditionsMet(measurements, request, config, nowUs)) {
          state_ = State::Run;
          resetRunTracking();
          checkEnvelopes(measurements, config);  // drivers must not be enabled on an invalid model
        } else if (elapsedLongerThan(nowUs, armingStartUs_, config.armingTimeoutUs)) {
          state_ = State::Disabled;
        }
        break;
      case State::Run:
      case State::Commissioning:  // Run <-> Commissioning transitions arrive in package 4 step 7.
        checkRunning(measurements, request, config, nowUs);
        break;
      case State::Fault:  // Only the switch leaves Fault, and it leads to Disabled, never to Run.
        break;
    }
  }

  if (state_ != State::Run && state_ != State::Commissioning) {
    leftMinA_ = leftMaxA_ = rightMinA_ = rightMaxA_ = 0.0f;
    shapedDemand_ = Demand{};
  }

  Output output;
  output.driveEnabled = state_ == State::Run;
  output.state = state_;
  output.fault = fault_;
  output.diag.leftMinA = leftMinA_;
  output.diag.leftMaxA = leftMaxA_;
  output.diag.rightMinA = rightMinA_;
  output.diag.rightMaxA = rightMaxA_;
  output.diag.demandScale = demandScale_;
  output.diag.shapedSpeedMps = shapedDemand_.speedMps;
  output.diag.shapedYawRateRadPerSec = shapedDemand_.yawRateRadPerSec;
  return output;
}

void BalanceController::resetRunTracking() {
  imuTimestampValid_ = false;
  fallBeyond_ = false;
  overspeedActive_ = false;
  shapedDemand_ = Demand{};
}

void BalanceController::latchFault(FaultReason reason) {
  state_ = State::Fault;
  fault_ = reason;
  leftMinA_ = leftMaxA_ = rightMinA_ = rightMaxA_ = 0.0f;
}

void BalanceController::checkRunning(const Measurements &measurements, const Request &request,
                                    const Config &config, uint32_t nowUs) {
  // Freshness is checked on every tick, so a stalled IMU goes stale even though its sequence does not change.
  if (isStale(nowUs, measurements.imuReadTimeUs, config.imuMaxAgeUs)) {
    latchFault(FaultReason::StaleImu);
    return;
  }
  if (isStale(nowUs, measurements.wheelReadTimeUs, config.wheelMaxAgeUs)) {
    latchFault(FaultReason::StaleWheels);
    return;
  }
  if (isStale(nowUs, measurements.busReadTimeUs, config.busMaxAgeUs)) {
    latchFault(FaultReason::StaleBus);
    return;
  }
  if (!measurements.imuValid || !measurements.wheelsValid) {
    latchFault(FaultReason::InvalidInput);
    return;
  }

  // Only a new IMU sample feeds dt and the fall timer. An unchanged sequence is not a fresh sample.
  const bool newImuSample = !imuSequenceSeen_ || measurements.imuSequence != lastImuSequence_;
  if (newImuSample) {
    imuSequenceSeen_ = true;
    lastImuSequence_ = measurements.imuSequence;

    if (imuTimestampValid_) {
      const uint32_t dtUs = measurements.imuReadTimeUs - lastImuReadTimeUs_;
      if (dtUs == 0 || dtUs > config.dtMaxUs) {
        latchFault(FaultReason::BadDt);
        return;
      }
      dtSec_ = static_cast<float>(dtUs) * 1e-6f;

      // A stale or invalid motion request is treated as a zero request and ramps down at the brake cap.
      Demand requested;
      if (request.sourceValid && !isStale(nowUs, request.receivedTimeUs, config.requestMaxAgeUs)) {
        requested.speedMps = request.speedMps;
        requested.yawRateRadPerSec = request.yawRateRadPerSec;
      }
      shapedDemand_ = shapeDemand(shapedDemand_, requested, demandScale_, dtSec_, config);
    }
    imuTimestampValid_ = true;
    lastImuReadTimeUs_ = measurements.imuReadTimeUs;

    if (std::fabs(measurements.tiltRad) <= config.fallAngleRad) {
      fallBeyond_ = false;
    } else if (!fallBeyond_) {
      fallBeyond_ = true;
      fallStartUs_ = nowUs;
    } else if (elapsedLongerThan(nowUs, fallStartUs_, config.fallDurationUs)) {
      latchFault(FaultReason::Fall);
      return;
    }
  }

  // Written so that a NaN speed counts as over the limit.
  const bool withinLimit = std::fabs(measurements.leftWheelRadPerSec) <= config.wheelOverspeedRadPerSec &&
                           std::fabs(measurements.rightWheelRadPerSec) <= config.wheelOverspeedRadPerSec;
  if (withinLimit) {
    overspeedActive_ = false;
  } else if (!overspeedActive_) {
    overspeedActive_ = true;
    overspeedStartUs_ = nowUs;
  } else if (elapsedLongerThan(nowUs, overspeedStartUs_, config.overspeedDurationUs)) {
    latchFault(FaultReason::Overspeed);
    return;
  }

  checkEnvelopes(measurements, config);
}

void BalanceController::checkEnvelopes(const Measurements &measurements, const Config &config) {
  if (!demandConfigValid(config)) {
    latchFault(FaultReason::InvalidConfig);
    return;
  }
  const float usableV = (measurements.busVoltageV - config.sagMarginV) * config.modulationFactor;
  const float vMaxV = std::min(config.voltageLimitV, usableV);
  // Written so that a NaN input fails the check.
  const bool modelValid = std::isfinite(usableV) && std::isfinite(vMaxV) && vMaxV > 0.0f &&
                          measurements.busVoltageV >= config.busMinV &&
                          measurements.busVoltageV <= config.busMaxV && config.kvRating > 0.0f &&
                          config.phaseResistanceOhm > 0.0f && isUnitSign(config.leftWheelSign) &&
                          isUnitSign(config.rightWheelSign);
  if (!modelValid) {
    latchFault(FaultReason::InvalidEnvelope);
    return;
  }

  const WheelInterval left =
      wheelInterval(measurements.leftWheelRadPerSec, config.leftWheelSign, vMaxV, config);
  const WheelInterval right =
      wheelInterval(measurements.rightWheelRadPerSec, config.rightWheelSign, vMaxV, config);
  leftMinA_ = left.minA;
  leftMaxA_ = left.maxA;
  rightMinA_ = right.minA;
  rightMaxA_ = right.maxA;

  const bool valid = left.finite && right.finite && left.minA <= left.maxA && right.minA <= right.maxA;
  if (!valid) {
    latchFault(FaultReason::InvalidEnvelope);
  }
}
