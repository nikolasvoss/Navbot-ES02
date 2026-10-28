#include "MotorDiagnostic.h"

#include <cmath>

namespace MotorDiagnostic {
namespace {
bool safe(const Inputs &inputs) {
  return !inputs.balanceEnabled && !inputs.calibrationActive && std::isfinite(inputs.leftVelocityRadPerSec) &&
         std::isfinite(inputs.rightVelocityRadPerSec) &&
         std::fabs(inputs.leftVelocityRadPerSec) <= kMaximumSpeedRadPerSec &&
         std::fabs(inputs.rightVelocityRadPerSec) <= kMaximumSpeedRadPerSec;
}

bool valid(const Request &request) {
  if (request.leaseMs == 0 || request.leaseMs > kMaximumLeaseMs || request.mode == Mode::Idle) return false;
  if (!std::isfinite(request.leftCurrentA) || !std::isfinite(request.rightCurrentA) ||
      !std::isfinite(request.voltageV)) return false;
  if (request.mode == Mode::CurrentTest)
    return request.selectedMotor == kBothMotors && std::fabs(request.leftCurrentA) <= kMaximumCurrentA &&
           std::fabs(request.rightCurrentA) <= kMaximumCurrentA && request.voltageV == 0.0f;
  return request.mode == Mode::VoltageIdentification && (request.selectedMotor == 1 || request.selectedMotor == 2) &&
         request.leftCurrentA == 0.0f && request.rightCurrentA == 0.0f &&
         std::fabs(request.voltageV) <= kMaximumVoltageV;
}

// The lease of an action arrives as a float; a fraction of a millisecond is not a lease.
bool wholeLease(float leaseMs) {
  return std::isfinite(leaseMs) && std::floor(leaseMs) == leaseMs && leaseMs >= 1.0f && leaseMs <= kMaximumLeaseMs;
}
}  // namespace

bool elapsed(uint32_t nowMs, uint32_t startMs, uint32_t durationMs) {
  return static_cast<uint32_t>(nowMs - startMs) >= durationMs;
}

bool MakeCurrentRequest(float leftA, float rightA, float leaseMs, Request &request) {
  if (!wholeLease(leaseMs)) return false;
  Request parsed{};
  parsed.mode = Mode::CurrentTest;
  parsed.leftCurrentA = leftA;
  parsed.rightCurrentA = rightA;
  parsed.leaseMs = static_cast<uint32_t>(leaseMs);
  if (!valid(parsed)) return false;
  request = parsed;
  return true;
}

bool MakeVoltageRequest(float motor, float voltageV, float leaseMs, Request &request) {
  if (!wholeLease(leaseMs) || (motor != 1.0f && motor != 2.0f)) return false;
  Request parsed{};
  parsed.mode = Mode::VoltageIdentification;
  parsed.selectedMotor = static_cast<uint8_t>(motor);
  parsed.voltageV = voltageV;
  parsed.leaseMs = static_cast<uint32_t>(leaseMs);
  if (!valid(parsed)) return false;
  request = parsed;
  return true;
}

const char *RequestResultText(RequestResult result) {
  switch (result) {
    case RequestResult::Accepted:
      return "accepted";
    case RequestResult::Invalid:
      return "invalid request";
    case RequestResult::Unsafe:
      return "refused: balance or calibration active, or a wheel is too fast";
    case RequestResult::FaultLatched:
      return "refused: fault latched, send motor_reset";
  }
  return "refused";
}

RequestResult start(State &state, const Request &request, const Inputs &inputs) {
  if (!valid(request)) return RequestResult::Invalid;
  if (state.fault != Fault::None) return RequestResult::FaultLatched;
  if (!safe(inputs)) return RequestResult::Unsafe;
  state.request = request;
  state.startedAtMs = inputs.nowMs;
  state.overspeedPending = false;
  state.active = true;
  return RequestResult::Accepted;
}

void stop(State &state) {
  state.active = false;
  state.overspeedPending = false;
}

bool resetFault(State &state, const Inputs &inputs) {
  if (inputs.balanceEnabled || inputs.calibrationActive || !std::isfinite(inputs.leftVelocityRadPerSec) ||
      !std::isfinite(inputs.rightVelocityRadPerSec) ||
      std::fabs(inputs.leftVelocityRadPerSec) > kMaximumSpeedRadPerSec ||
      std::fabs(inputs.rightVelocityRadPerSec) > kMaximumSpeedRadPerSec)
    return false;
  state.fault = Fault::None;
  stop(state);
  return true;
}

Fault update(State &state, const Inputs &inputs) {
  if (state.fault != Fault::None) return state.fault;
  if (!state.active) return Fault::None;
  if (!std::isfinite(inputs.leftVelocityRadPerSec) || !std::isfinite(inputs.rightVelocityRadPerSec)) {
    state.fault = Fault::InvalidVelocity;
  } else {
    // A single over-limit sample can be measurement noise; the fault needs a continuous run of them.
    const bool overspeed = std::fabs(inputs.leftVelocityRadPerSec) > kMaximumSpeedRadPerSec ||
                           std::fabs(inputs.rightVelocityRadPerSec) > kMaximumSpeedRadPerSec;
    if (!overspeed) {
      state.overspeedPending = false;
    } else {
      if (!state.overspeedPending) {
        state.overspeedPending = true;
        state.overspeedSinceMs = inputs.nowMs;
      }
      if (elapsed(inputs.nowMs, state.overspeedSinceMs, kOverspeedDebounceMs)) state.fault = Fault::Overspeed;
    }
    if (state.fault == Fault::None && state.active && inputs.balanceEnabled) {
      state.fault = Fault::BalanceEnabled;
    } else if (state.fault == Fault::None && state.active && inputs.calibrationActive) {
      state.fault = Fault::CalibrationActive;
    } else if (state.fault == Fault::None && state.active &&
               elapsed(inputs.nowMs, state.startedAtMs, state.request.leaseMs)) {
      state.fault = Fault::LeaseExpired;
    }
  }
  if (state.fault != Fault::None) stop(state);
  return state.fault;
}

}  // namespace MotorDiagnostic
