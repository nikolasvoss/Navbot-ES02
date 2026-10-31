#pragma once

#include <cstdint>

namespace MotorDiagnostic {

enum class Mode : uint8_t { Idle, CurrentTest, VoltageIdentification };
enum class Fault : uint8_t { None, LeaseExpired, BalanceEnabled, CalibrationActive, InvalidVelocity, Overspeed };
enum class RequestResult : uint8_t { Accepted, Invalid, Unsafe, FaultLatched };

constexpr uint32_t kMaximumLeaseMs = 5000;
// Bound on the T current target. Raised from 0.02 A to 0.35 A on 2026-10-22 by operator request: at the
// commissioned 5.75 ohm per phase, 0.35 A needs 2.0 V, so it matches kMaximumVoltageV at standstill.
constexpr float kMaximumCurrentA = 0.35f;
// Wheel-clear runs only. The request reaches voltage.q unclamped up to this value; the SpaceVectorPWM
// modulator saturates near supply/sqrt(3). Lowered back to 2.0 V on 2026-10-22 by operator request.
constexpr float kMaximumVoltageV = 2.0f;
// Wheel-clear runs only. Lowered back to 60 rad/s on 2026-10-22 by operator request (earlier 15 and 3 rad/s
// caps ended identification before any speed plateau).
constexpr float kMaximumSpeedRadPerSec = 60.0f;
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
// The motor_current action's arguments as a request. The lease must be a whole number of milliseconds.
bool MakeCurrentRequest(float leftA, float rightA, float leaseMs, Request &request);
// The motor_voltage action's arguments as a request: motor 1 or 2, a voltage and a whole-number lease.
bool MakeVoltageRequest(float motor, float voltageV, float leaseMs, Request &request);
// Why start() refused a request, as the Ack of the action shows it.
const char *RequestResultText(RequestResult result);

}  // namespace MotorDiagnostic
