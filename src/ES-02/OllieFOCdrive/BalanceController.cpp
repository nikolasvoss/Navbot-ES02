#include "BalanceController.h"

#include <cmath>

#include "MathConstants.h"

namespace {
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
}  // namespace

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
        } else if (elapsedLongerThan(nowUs, armingStartUs_, config.armingTimeoutUs)) {
          state_ = State::Disabled;
        }
        break;
      case State::Run:
      case State::Commissioning:  // Run <-> Commissioning transitions arrive in package 4 step 7.
        checkRunning(measurements, config, nowUs);
        break;
      case State::Fault:  // Only the switch leaves Fault, and it leads to Disabled, never to Run.
        break;
    }
  }

  Output output;
  output.driveEnabled = state_ == State::Run;
  output.state = state_;
  output.fault = fault_;
  return output;
}

void BalanceController::resetRunTracking() {
  imuTimestampValid_ = false;
  fallBeyond_ = false;
  overspeedActive_ = false;
}

void BalanceController::latchFault(FaultReason reason) {
  state_ = State::Fault;
  fault_ = reason;
}

void BalanceController::checkRunning(const Measurements &measurements, const Config &config, uint32_t nowUs) {
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
  }
}
