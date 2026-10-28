#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

#include "../../src/ES-02/OllieFOCdrive/MotorDiagnostic.h"

namespace {
MotorDiagnostic::Inputs safeInputs(uint32_t now = 10) { return {now, false, false, 0.0f, 0.0f}; }

MotorDiagnostic::Inputs velocityInputs(uint32_t now, float left, float right) {
  return {now, false, false, left, right};
}

MotorDiagnostic::Request currentRequest(float left = 0.02f, float right = -0.02f, uint32_t lease = 2000) {
  MotorDiagnostic::Request request{};
  request.mode = MotorDiagnostic::Mode::CurrentTest;
  request.leftCurrentA = left;
  request.rightCurrentA = right;
  request.leaseMs = lease;
  return request;
}
}  // namespace

int main() {
  using namespace MotorDiagnostic;
  // The motor_current action: the endpoint has checked the argument bounds; these checks are the module's own.
  Request parsed{};
  assert(MakeCurrentRequest(0.02f, -0.02f, 2000.0f, parsed));
  assert(parsed.mode == Mode::CurrentTest && parsed.leftCurrentA == 0.02f && parsed.rightCurrentA == -0.02f);
  assert(parsed.leaseMs == 2000 && parsed.selectedMotor == kBothMotors && parsed.voltageV == 0.0f);
  assert(!MakeCurrentRequest(0.1f, 0.1f, 500.5f, parsed));  // the lease is a whole number of ms
  assert(!MakeCurrentRequest(0.1f, 0.1f, 0.0f, parsed));
  assert(!MakeCurrentRequest(0.1f, 0.1f, 5001.0f, parsed));
  assert(!MakeCurrentRequest(0.36f, 0.0f, 500.0f, parsed));  // above kMaximumCurrentA
  assert(!MakeCurrentRequest(std::nanf(""), 0.0f, 500.0f, parsed));
  assert(MakeCurrentRequest(0.0f, 0.0f, 5000.0f, parsed) && parsed.leaseMs == 5000);
  // The motor_voltage action: motor 1 or 2, voltage up to kMaximumVoltageV, whole-number lease.
  assert(MakeVoltageRequest(2.0f, -0.5f, 1250.0f, parsed));
  assert(parsed.mode == Mode::VoltageIdentification && parsed.selectedMotor == 2 && parsed.voltageV == -0.5f);
  assert(parsed.leaseMs == 1250 && parsed.leftCurrentA == 0.0f && parsed.rightCurrentA == 0.0f);
  assert(MakeVoltageRequest(1.0f, 2.0f, 5000.0f, parsed) && parsed.voltageV == 2.0f && parsed.leaseMs == 5000);
  assert(MakeVoltageRequest(2.0f, -2.0f, 5000.0f, parsed) && parsed.voltageV == -2.0f);
  assert(!MakeVoltageRequest(0.0f, 0.1f, 500.0f, parsed));  // motor 1 or 2
  assert(!MakeVoltageRequest(1.5f, 0.1f, 500.0f, parsed));
  assert(!MakeVoltageRequest(1.0f, 2.01f, 500.0f, parsed));
  assert(!MakeVoltageRequest(1.0f, 0.1f, 5001.0f, parsed));
  assert(!MakeVoltageRequest(1.0f, std::nanf(""), 500.0f, parsed));
  assert(!MakeVoltageRequest(1.0f, 0.1f, 1000.9f, parsed));
  assert(std::strcmp(RequestResultText(RequestResult::Accepted), "accepted") == 0);
  assert(std::strcmp(RequestResultText(RequestResult::Invalid), "invalid request") == 0);
  assert(std::strcmp(RequestResultText(RequestResult::FaultLatched), "refused: fault latched, send motor_reset") == 0);

  State state{};
  Request request = currentRequest();
  assert(start(state, request, safeInputs()) == RequestResult::Accepted);
  assert(state.active);
  Inputs conflict = safeInputs(11);
  conflict.balanceEnabled = true;
  assert(update(state, conflict) == Fault::BalanceEnabled);
  assert(!state.active);
  assert(start(state, request, safeInputs(12)) == RequestResult::FaultLatched);
  assert(!resetFault(state, conflict));
  assert(resetFault(state, safeInputs(13)));
  assert(!state.active && state.fault == Fault::None);

  // A motor action is refused while balance or calibration runs, and the Ack says so; nothing latches.
  Inputs calibration = safeInputs();
  calibration.calibrationActive = true;
  assert(start(state, request, calibration) == RequestResult::Unsafe);
  assert(!state.active && state.fault == Fault::None);
  Inputs balancing = safeInputs(20);
  balancing.balanceEnabled = true;
  assert(start(state, request, balancing) == RequestResult::Unsafe);
  assert(std::strcmp(RequestResultText(RequestResult::Unsafe),
                     "refused: balance or calibration active, or a wheel is too fast") == 0);
  assert(!state.active && state.fault == Fault::None);
  assert(start(state, request, safeInputs()) == RequestResult::Accepted);
  assert(update(state, calibration) == Fault::CalibrationActive);
  assert(resetFault(state, safeInputs()));

  assert(start(state, request, safeInputs(100)) == RequestResult::Accepted);
  assert(update(state, safeInputs(2099)) == Fault::None);
  assert(update(state, safeInputs(2100)) == Fault::LeaseExpired);
  assert(!state.active);
  assert(resetFault(state, safeInputs(2101)));

  Request voltage{};
  assert(MakeVoltageRequest(1.0f, 0.4f, 1000.0f, voltage));
  const float overLimit = kMaximumSpeedRadPerSec + 0.01f;

  // Overspeed latches once an over-limit condition has lasted kOverspeedDebounceMs (50 ms).
  assert(start(state, voltage, safeInputs(300)) == RequestResult::Accepted);
  assert(update(state, velocityInputs(301, 0.0f, overLimit)) == Fault::None);
  assert(update(state, velocityInputs(350, 0.0f, overLimit)) == Fault::None);  // 49 ms
  Inputs overspeed = velocityInputs(351, 0.0f, overLimit);                    // 50 ms
  assert(update(state, overspeed) == Fault::Overspeed);
  assert(!state.active && !state.overspeedPending);
  assert(!resetFault(state, overspeed));
  assert(resetFault(state, safeInputs(352)));

  // The left wheel alone counts, and the fault latches on the 50 ms sample.
  assert(start(state, voltage, safeInputs(7000)) == RequestResult::Accepted);
  assert(update(state, velocityInputs(7001, -overLimit, 0.0f)) == Fault::None);
  assert(update(state, velocityInputs(7050, -overLimit, 0.0f)) == Fault::None);  // 49 ms
  assert(state.active && state.fault == Fault::None);
  assert(update(state, velocityInputs(7051, -overLimit, 0.0f)) == Fault::Overspeed);
  assert(!state.active && state.fault == Fault::Overspeed);
  assert(resetFault(state, safeInputs(7052)));

  // An in-limit sample clears the timer; the run must restart from the next over-limit sample.
  assert(start(state, voltage, safeInputs(8000)) == RequestResult::Accepted);
  assert(update(state, velocityInputs(8001, 0.0f, overLimit)) == Fault::None);
  assert(update(state, velocityInputs(8030, 0.0f, overLimit)) == Fault::None);
  assert(update(state, velocityInputs(8031, 0.0f, 0.5f)) == Fault::None);
  assert(!state.overspeedPending);
  assert(update(state, velocityInputs(8032, 0.0f, overLimit)) == Fault::None);
  assert(update(state, velocityInputs(8081, 0.0f, overLimit)) == Fault::None);  // 49 ms since 8032
  assert(update(state, velocityInputs(8082, 0.0f, overLimit)) == Fault::Overspeed);  // 50 ms
  assert(resetFault(state, safeInputs(8083)));

  // A single -65 rad/s sample (above the 60 rad/s cap), then an in-limit sample, does not fault.
  assert(start(state, voltage, safeInputs(9000)) == RequestResult::Accepted);
  assert(update(state, velocityInputs(9001, 0.8f, -65.0f)) == Fault::None);
  assert(update(state, velocityInputs(9002, 0.8f, -0.8f)) == Fault::None);
  assert(update(state, velocityInputs(9003, -40.0f, 0.8f)) == Fault::None);
  assert(update(state, velocityInputs(9004, 0.8f, 0.8f)) == Fault::None);
  assert(state.active && state.fault == Fault::None && !state.overspeedPending);
  stop(state);
  assert(resetFault(state, safeInputs(9005)));

  // start() on an active lease clears a pending over-limit run.
  assert(start(state, voltage, safeInputs(10000)) == RequestResult::Accepted);
  assert(update(state, velocityInputs(10001, 0.0f, overLimit)) == Fault::None);
  assert(start(state, voltage, safeInputs(10002)) == RequestResult::Accepted);
  assert(!state.overspeedPending);
  assert(update(state, velocityInputs(10003, 0.0f, overLimit)) == Fault::None);
  assert(update(state, velocityInputs(10052, 0.0f, overLimit)) == Fault::None);  // 49 ms since 10003
  assert(update(state, velocityInputs(10053, 0.0f, overLimit)) == Fault::Overspeed);
  assert(resetFault(state, safeInputs(10054)));

  // stop() clears a pending over-limit run.
  assert(start(state, voltage, safeInputs(11000)) == RequestResult::Accepted);
  assert(update(state, velocityInputs(11001, 0.0f, overLimit)) == Fault::None);
  stop(state);
  assert(!state.overspeedPending);
  assert(start(state, voltage, safeInputs(11010)) == RequestResult::Accepted);
  assert(update(state, velocityInputs(11011, 0.0f, overLimit)) == Fault::None);
  assert(update(state, velocityInputs(11060, 0.0f, overLimit)) == Fault::None);  // 49 ms
  assert(update(state, velocityInputs(11061, 0.0f, overLimit)) == Fault::Overspeed);
  assert(resetFault(state, safeInputs(11062)));

  // resetFault() clears a pending over-limit run.
  assert(start(state, voltage, safeInputs(12000)) == RequestResult::Accepted);
  assert(update(state, velocityInputs(12001, 0.0f, overLimit)) == Fault::None);
  assert(resetFault(state, safeInputs(12010)));
  assert(!state.overspeedPending);
  assert(start(state, voltage, safeInputs(12011)) == RequestResult::Accepted);
  assert(update(state, velocityInputs(12012, 0.0f, overLimit)) == Fault::None);
  assert(update(state, velocityInputs(12061, 0.0f, overLimit)) == Fault::None);  // 49 ms
  assert(update(state, velocityInputs(12062, 0.0f, overLimit)) == Fault::Overspeed);
  assert(resetFault(state, safeInputs(12063)));

  // The timer is wrap-safe: the run starts at 0xfffffff8 and the fault lands at 0x2a, 50 ms later.
  assert(start(state, voltage, safeInputs(0xfffffff0u)) == RequestResult::Accepted);
  assert(update(state, velocityInputs(0xfffffff8u, 0.0f, overLimit)) == Fault::None);
  assert(update(state, velocityInputs(0xfffffff9u, 0.0f, overLimit)) == Fault::None);
  assert(update(state, velocityInputs(0x00000029u, 0.0f, overLimit)) == Fault::None);  // 49 ms
  assert(update(state, velocityInputs(0x0000002au, 0.0f, overLimit)) == Fault::Overspeed);  // 50 ms
  assert(resetFault(state, safeInputs(0x0000002bu)));

  assert(start(state, voltage, safeInputs(400)) == RequestResult::Accepted);
  Inputs nonfinite = safeInputs(401);
  nonfinite.leftVelocityRadPerSec = std::numeric_limits<float>::quiet_NaN();
  assert(update(state, nonfinite) == Fault::InvalidVelocity);
  assert(resetFault(state, safeInputs(402)));

  assert(start(state, voltage, safeInputs(500)) == RequestResult::Accepted);
  assert(!elapsed(501, 500, 2));
  assert(elapsed(502, 500, 2));
  const uint32_t startNearWrap = 0xfffffff0u;
  assert(!elapsed(0xfffffff5u, startNearWrap, 10));
  assert(elapsed(0x00000004u, startNearWrap, 20));
  stop(state);
  assert(!state.active && state.fault == Fault::None);

  Inputs normalMotion = safeInputs(600);
  normalMotion.leftVelocityRadPerSec = 50.0f;
  assert(update(state, normalMotion) == Fault::None);

  // Renewing an active lease restarts its timer from the renewal time.
  assert(start(state, currentRequest(0.01f, 0.01f, 1000), safeInputs(1000)) == RequestResult::Accepted);
  assert(start(state, currentRequest(0.01f, 0.01f, 1000), safeInputs(1900)) == RequestResult::Accepted);
  assert(state.active && state.startedAtMs == 1900);
  assert(update(state, safeInputs(2899)) == Fault::None);
  assert(update(state, safeInputs(2900)) == Fault::LeaseExpired);
  assert(!state.active);
  assert(resetFault(state, safeInputs(2901)));

  // An invalid request is refused without touching the running lease.
  assert(start(state, currentRequest(0.01f, 0.01f, 1000), safeInputs(3000)) == RequestResult::Accepted);
  assert(start(state, currentRequest(0.36f, 0.0f, 1000), safeInputs(3500)) == RequestResult::Invalid);
  assert(state.active && state.startedAtMs == 3000);
  assert(state.request.leftCurrentA == 0.01f && state.request.rightCurrentA == 0.01f);
  assert(state.request.leaseMs == 1000);
  assert(update(state, safeInputs(3999)) == Fault::None);
  assert(update(state, safeInputs(4000)) == Fault::LeaseExpired);
  assert(resetFault(state, safeInputs(4001)));

  // A mode switch while active replaces the request and restarts the lease.
  assert(start(state, currentRequest(0.01f, 0.01f, 1000), safeInputs(5000)) == RequestResult::Accepted);
  Request voltageSwitch{};
  assert(MakeVoltageRequest(1.0f, 0.3f, 800.0f, voltageSwitch));
  assert(start(state, voltageSwitch, safeInputs(5200)) == RequestResult::Accepted);
  assert(state.active && state.startedAtMs == 5200);
  assert(state.request.mode == Mode::VoltageIdentification && state.request.selectedMotor == 1);
  assert(state.request.voltageV == 0.3f && state.request.leaseMs == 800);
  assert(update(state, safeInputs(5999)) == Fault::None);
  assert(update(state, safeInputs(6000)) == Fault::LeaseExpired);
  assert(resetFault(state, safeInputs(6001)));

  // start() on a lease that has expired but not yet seen by update() begins a fresh lease.
  // The new request is fully validated and gated, so no stale lease survives.
  assert(start(state, currentRequest(0.01f, 0.01f, 100), safeInputs(7000)) == RequestResult::Accepted);
  assert(start(state, currentRequest(-0.01f, 0.01f, 300), safeInputs(7500)) == RequestResult::Accepted);
  assert(state.active && state.startedAtMs == 7500);
  assert(state.request.leftCurrentA == -0.01f && state.request.leaseMs == 300);
  assert(update(state, safeInputs(7799)) == Fault::None);
  assert(update(state, safeInputs(7800)) == Fault::LeaseExpired);
  assert(resetFault(state, safeInputs(7801)));

  std::puts("motor diagnostic state machine passed");
}
