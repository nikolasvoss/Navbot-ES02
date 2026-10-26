#include "MotorDiagnostic.h"

#include <cmath>
#include <cstdlib>

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

bool parseNumbers(const char *text, float *values, size_t count) {
  if (text == nullptr || values == nullptr) return false;
  const char *cursor = text;
  for (size_t i = 0; i < count; ++i) {
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r' || *cursor == '\n') ++cursor;
    if (*cursor == '\0') return false;
    char *end = nullptr;
    values[i] = std::strtof(cursor, &end);
    if (end == cursor || !std::isfinite(values[i])) return false;
    cursor = end;
    if (i + 1 < count && *cursor != ' ' && *cursor != '\t' && *cursor != '\r' && *cursor != '\n') return false;
  }
  while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r' || *cursor == '\n') ++cursor;
  return *cursor == '\0';
}
}  // namespace

bool elapsed(uint32_t nowMs, uint32_t startMs, uint32_t durationMs) {
  return static_cast<uint32_t>(nowMs - startMs) >= durationMs;
}

bool parseCurrentRequest(const char *text, Request &request) {
  float values[3]{};
  if (!parseNumbers(text, values, 3) || values[2] < 1.0f || values[2] > kMaximumLeaseMs ||
      std::floor(values[2]) != values[2]) return false;
  Request parsed{};
  parsed.mode = Mode::CurrentTest;
  parsed.leftCurrentA = values[0];
  parsed.rightCurrentA = values[1];
  parsed.leaseMs = static_cast<uint32_t>(values[2]);
  if (!valid(parsed)) return false;
  request = parsed;
  return true;
}

bool parseVoltageRequest(const char *text, Request &request) {
  float values[3]{};
  if (!parseNumbers(text, values, 3) || (values[0] != 1.0f && values[0] != 2.0f) ||
      values[2] < 1.0f || values[2] > kMaximumLeaseMs || std::floor(values[2]) != values[2]) return false;
  Request parsed{};
  parsed.mode = Mode::VoltageIdentification;
  parsed.selectedMotor = static_cast<uint8_t>(values[0]);
  parsed.voltageV = values[1];
  parsed.leaseMs = static_cast<uint32_t>(values[2]);
  if (!valid(parsed)) return false;
  request = parsed;
  return true;
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
