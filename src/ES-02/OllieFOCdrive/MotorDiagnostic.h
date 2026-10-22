#pragma once

#include <cstdint>

namespace MotorDiagnostic {

enum class Mode : uint8_t { Idle, CurrentTest, VoltageIdentification };
enum class Fault : uint8_t { None, LeaseExpired, BalanceEnabled, CalibrationActive, InvalidVelocity, Overspeed };
enum class RequestResult : uint8_t { Accepted, Invalid, Unsafe, FaultLatched };

constexpr uint32_t kMaximumLeaseMs = 5000;
// Estimate-based bound on the T current target, not derived from kMaximumVoltageV. It rests on the 22 ohm
// kWheelPhaseResistanceOhm estimate, which is not measured (the measured phase-to-phase value is 11.5 ohm).
constexpr float kMaximumCurrentA = 0.02f;
// Wheel-clear runs only. Raised from 2.0 V by operator approval on 2026-10-22. The request reaches
// voltage.q unclamped up to this value; the SpaceVectorPWM modulator saturates near supply/sqrt(3).
constexpr float kMaximumVoltageV = 6.0f;
// Wheel-clear runs only. Raised from 60 rad/s by operator approval on 2026-10-22 (earlier 15 and 3 rad/s
// caps ended identification before any speed plateau).
constexpr float kMaximumSpeedRadPerSec = 120.0f;
// Overspeed latches only after a wheel has stayed above the limit for this long (measured with Inputs::nowMs).
constexpr uint32_t kOverspeedDebounceMs = 50;

constexpr uint8_t kBothMotors = 0;

struct Request {
  Mode mode = Mode::Idle;
  float leftCurrentA = 0.0f;
  float rightCurrentA = 0.0f;
  uint8_t selectedMotor = kBothMotors;
  float voltageV = 0.0f;
  uint32_t leaseMs = 0;
};

struct State {
  Request request{};
  uint32_t startedAtMs = 0;
  bool active = false;
  Fault fault = Fault::None;
  uint32_t overspeedSinceMs = 0;
  bool overspeedPending = false;
};

struct Inputs {
  uint32_t nowMs = 0;
  bool balanceEnabled = false;
  bool calibrationActive = false;
  float leftVelocityRadPerSec = 0.0f;
  float rightVelocityRadPerSec = 0.0f;
};

RequestResult start(State &state, const Request &request, const Inputs &inputs);
void stop(State &state);
bool resetFault(State &state, const Inputs &inputs);
Fault update(State &state, const Inputs &inputs);
bool elapsed(uint32_t nowMs, uint32_t startMs, uint32_t durationMs);
bool parseCurrentRequest(const char *text, Request &request);
bool parseVoltageRequest(const char *text, Request &request);

}  // namespace MotorDiagnostic
