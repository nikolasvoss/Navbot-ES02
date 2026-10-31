#include <cmath>
#include <cstdio>

#include "sim.h"

namespace {

int g_failures = 0;

// Prints one line per check and counts failures; the process exit code is 1 if any check failed.
void expectNear(const char *name, double got, double want, double tol) {
  const bool ok = std::fabs(got - want) <= tol;
  std::printf("%s %s: got %.9g, want %.9g, tol %.3g\n", ok ? "PASS" : "FAIL", name, got, want, tol);
  if (!ok) ++g_failures;
}

void expectTrue(const char *name, bool ok) {
  std::printf("%s %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) ++g_failures;
}

// Total mechanical energy of the plant with no yaw: axle and wheels, body in pitch, potential of the centre of mass.
double plantEnergy(const sim::PlantState &s, const sim::PlantParams &p) {
  const double wheelRadius = p.wheelRadiusM;
  const double m = p.bodyMassKg + 2.0 * p.wheelMassKg + 2.0 * p.wheelInertiaKgM2 / (wheelRadius * wheelRadius);
  const double j = p.bodyPitchInertiaKgM2 + p.bodyMassKg * p.comHeightM * p.comHeightM;
  const double mh = p.bodyMassKg * p.comHeightM;
  const double kinetic = 0.5 * m * s.vMps * s.vMps + mh * std::cos(s.pitchRad) * s.vMps * s.pitchRateRadPerSec +
                         0.5 * j * s.pitchRateRadPerSec * s.pitchRateRadPerSec;
  const double potential = p.bodyMassKg * p.gravityMps2 * p.comHeightM * std::cos(s.pitchRad);
  return kinetic + potential;
}

void testLinearization() {
  const sim::PlantParams p;
  // Default parameters: A = mb g h / D = 252.48 per s^2 and B = (1 + mb h / (r M)) / D = 2505.0 per (N m) (plan T2).
  sim::PlantState s;
  s.pitchRad = 0.001;
  sim::stepPlant(s, p, sim::WheelTorques{}, 1e-6);
  expectNear("pitch accel at 0.001 rad, no torque (A * theta = 0.25248)", s.pitchAccelRadPerSec2, 0.25248, 0.0025);

  sim::PlantState t;
  sim::stepPlant(t, p, sim::WheelTorques{0.01, 0.01}, 1e-6);
  expectNear("pitch accel for torque sum 0.02 N m (-B u = -50.10)", t.pitchAccelRadPerSec2, -50.10, 0.501);
}

void testEnergy() {
  const sim::PlantParams p = [] {
    sim::PlantParams params;
    params.bearingFrictionNmPerRadPerSec = 0.0;
    return params;
  }();
  sim::PlantState s;
  s.pitchRad = 0.3;
  const double e0 = plantEnergy(s, p);
  for (int i = 0; i < 2000; ++i) sim::stepPlant(s, p, sim::WheelTorques{}, 50e-6);
  expectTrue("energy test: plant has not reached ground contact within 0.1 s", !s.fallen);
  expectNear("energy change from theta 0.3 over 0.1 s, relative (< 0.001)", (plantEnergy(s, p) - e0) / e0, 0.0,
             0.001);
}

void testFall() {
  const sim::PlantParams p;
  sim::PlantState s;
  s.pitchRad = 0.1;
  int steps = 0;
  while (!s.fallen && steps < 20000) {
    sim::stepPlant(s, p, sim::WheelTorques{}, 50e-6);
    ++steps;
  }
  expectTrue("fall: reaches fallen within 1 s", s.fallen && steps * 50e-6 <= 1.0);
  expectNear("fall: pitch clamped to ground contact", s.pitchRad, 1.2, 0.0);

  const double pitchAtFall = s.pitchRad;
  for (int i = 0; i < 2000; ++i) sim::stepPlant(s, p, sim::WheelTorques{0.01, 0.01}, 50e-6);
  expectTrue("fall: stays fallen with no motion", s.fallen && s.pitchRad == pitchAtFall && s.vMps == 0.0 &&
                                                     s.leftWheelRadPerSec == 0.0);
}

void testYaw() {
  const sim::PlantParams p;
  sim::PlantState s;
  // Equal and opposite torques: psi'' = (d / (2 r)) * 0.02 / Jpsi = 26.446 per s^2.
  sim::stepPlant(s, p, sim::WheelTorques{-0.01, 0.01}, 1e-3);
  expectNear("yaw rate after 1 ms of +-0.01 N m (psi'' * 1e-3)", s.yawRateRadPerSec, 0.0264459, 0.0002645);
  expectNear("yaw test: axle acceleration zero", s.vMps, 0.0, 1e-9);
  expectNear("yaw test: pitch stays zero", s.pitchRad, 0.0, 1e-12);
}

void testLifted() {
  const sim::PlantParams p;
  sim::PlantState s;
  s.lifted = true;
  // Wheel inertia 2e-5 kg m^2: 0.001 N m gives 50 rad/s^2; the friction term is negligible over 50 us.
  sim::stepPlant(s, p, sim::WheelTorques{0.001, 0.0}, 50e-6);
  expectNear("lifted: left wheel rate after 50 us (0.001 / Iw * dt = 0.0025)", s.leftWheelRadPerSec, 0.0025, 1e-6);
  expectTrue("lifted: body stays at zero", s.xM == 0.0 && s.vMps == 0.0 && s.pitchRad == 0.0 && s.yawRad == 0.0);
}

void testPush() {
  const sim::PlantParams p;
  sim::PlantState s;
  // Impulse 0.01 N s at rest, theta 0: solution of [[M, c], [c, J]] [dv; dw] = [P; P h] with c = mb h.
  sim::applyPush(s, p, 0.01);
  expectNear("push: axle speed change", s.vMps, 0.0070076, 1e-7);
  expectNear("push: pitch rate change", s.pitchRateRadPerSec, 0.137593, 1e-6);
  expectNear("push: left wheel speed follows the axle speed (dv / r)", s.leftWheelRadPerSec, 0.36593, 1e-4);
}

void testMotorConfig() {
  sim::SimMotor motor;
  sim::configureSimMotor(motor, kLeftWheelMotor);
  expectNear("config: KV_rating = 79.5 * 1.8", motor.KV_rating, 143.1, 1e-3);
  expectNear("config: current_limit = 4.2 / 5.75", motor.current_limit, 0.7304, 1e-4);
}

void testMotorFoc() {
  sim::SimMotor motor;
  sim::configureSimMotor(motor, kLeftWheelMotor);
  motor.enable();
  motor.target = 0.5f;
  for (int i = 0; i < 200; ++i) sim::focStep(motor, 0.0, 1e-3);
  expectNear("focStep: 0.5 A at rest gives voltage.q 2.875", motor.voltage.q, 2.875, 1e-3);

  motor.target = 2.0f;
  for (int i = 0; i < 200; ++i) sim::focStep(motor, 0.0, 1e-3);
  expectNear("focStep: 2 A is clipped to voltage.q 4.2", motor.voltage.q, 4.2, 1e-3);

  motor.disable();
  sim::focStep(motor, 0.0, 1e-3);
  expectNear("focStep: disabled motor keeps voltage.q 0", motor.voltage.q, 0.0, 0.0);
}

void testApplied() {
  sim::SimMotor motor;
  sim::configureSimMotor(motor, kLeftWheelMotor);
  motor.enable();
  motor.voltage.q = 4.2f;
  expectNear("applied: 4.2 V at bus 8.4", sim::appliedVoltageQ(motor, 8.4), 4.2, 1e-6);
  expectNear("applied: 4.2 V at bus 6.0 gives 3.0", sim::appliedVoltageQ(motor, 6.0), 3.0, 1e-6);
  motor.voltage.q = 6.0f;
  expectNear("applied: 6.0 V at bus 8.4 clamps to 8.4 / sqrt(3) = 4.850", sim::appliedVoltageQ(motor, 8.4), 4.850,
             1e-3);
}

void testMotorOutput() {
  const sim::MotorPhysics physics;  // KV 79.5, R 5.75
  // Ke = 1 / (79.5 * sqrt(3) * 0.10472) = 0.069350 V per rad/s; i = -100 Ke / 5.75 = -1.20608 A.
  const sim::MotorOutput out = sim::motorOutput(physics, 0.0, 100.0);
  expectNear("motor output: current at 0 V, 100 rad/s", out.currentA, -1.20608, 1e-5);
}

}  // namespace

int main() {
  testLinearization();
  testEnergy();
  testFall();
  testYaw();
  testLifted();
  testPush();
  testMotorConfig();
  testMotorFoc();
  testApplied();
  testMotorOutput();
  std::printf("%s: %d failures\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
  return g_failures == 0 ? 0 : 1;
}
