#include <algorithm>
#include <array>
#include <cmath>

#include "sim.h"

namespace sim {
namespace {

// RK4 state: body and yaw, then the wheel angles. The wheel rates are kinematic (rolling without slip) unless the
// robot is lifted; then the two wheel rates are the free state.
enum Index { kX, kV, kPitch, kPitchRate, kYaw, kYawRate, kPhiL, kPhiR, kOmegaL, kOmegaR, kCount };
using Vec = std::array<double, kCount>;

// Mass matrix [[m11, m12], [m12, m22]] of (axle, pitch), with the wheel inertia reflected to the axle.
struct MassMatrix {
  double m11;
  double m12;
  double m22;
  double det;
};

MassMatrix massMatrix(const PlantParams &p, double pitchRad) {
  const double m = p.bodyMassKg + 2.0 * p.wheelMassKg + 2.0 * p.wheelInertiaKgM2 / (p.wheelRadiusM * p.wheelRadiusM);
  const double c = p.bodyMassKg * p.comHeightM * std::cos(pitchRad);
  const double j = p.bodyPitchInertiaKgM2 + p.bodyMassKg * p.comHeightM * p.comHeightM;
  return {m, c, j, m * j - c * c};
}

double yawInertia(const PlantParams &p) {
  const double halfTrack = p.trackWidthM / 2.0;
  return p.bodyYawInertiaKgM2 + 2.0 * p.wheelMassKg * halfTrack * halfTrack +
         2.0 * p.wheelInertiaKgM2 * halfTrack * halfTrack / (p.wheelRadiusM * p.wheelRadiusM);
}

// Wheel rates from the rolling constraint.
double leftRolling(const PlantParams &p, double v, double yawRate) {
  return (v - yawRate * p.trackWidthM / 2.0) / p.wheelRadiusM;
}

double rightRolling(const PlantParams &p, double v, double yawRate) {
  return (v + yawRate * p.trackWidthM / 2.0) / p.wheelRadiusM;
}

// Time derivative of the RK4 state for the torques applied.
Vec derivative(const Vec &s, const PlantParams &p, WheelTorques t, bool lifted) {
  Vec k{};
  if (lifted) {
    k[kPhiL] = s[kOmegaL];
    k[kPhiR] = s[kOmegaR];
    k[kOmegaL] = (t.leftNm - p.bearingFrictionNmPerRadPerSec * s[kOmegaL]) / p.wheelInertiaKgM2;
    k[kOmegaR] = (t.rightNm - p.bearingFrictionNmPerRadPerSec * s[kOmegaR]) / p.wheelInertiaKgM2;
    return k;
  }

  const double v = s[kV];
  const double pitch = s[kPitch];
  const double pitchRate = s[kPitchRate];
  const double yawRate = s[kYawRate];
  const double wl = leftRolling(p, v, yawRate);
  const double wr = rightRolling(p, v, yawRate);
  const double nL = t.leftNm - p.bearingFrictionNmPerRadPerSec * (wl - pitchRate);
  const double nR = t.rightNm - p.bearingFrictionNmPerRadPerSec * (wr - pitchRate);
  const double nSum = nL + nR;

  const MassMatrix mm = massMatrix(p, pitch);
  const double mh = p.bodyMassKg * p.comHeightM;
  const double r1 = nSum / p.wheelRadiusM + mh * std::sin(pitch) * pitchRate * pitchRate;
  const double r2 = p.bodyMassKg * p.gravityMps2 * p.comHeightM * std::sin(pitch) - nSum;
  const double axleAccel = (mm.m22 * r1 - mm.m12 * r2) / mm.det;
  const double pitchAccel = (mm.m11 * r2 - mm.m12 * r1) / mm.det;
  const double yawAccel =
      ((p.trackWidthM / (2.0 * p.wheelRadiusM)) * (nR - nL) - p.yawFrictionNmPerRadPerSec * yawRate) / yawInertia(p);

  k[kX] = v;
  k[kV] = axleAccel;
  k[kPitch] = pitchRate;
  k[kPitchRate] = pitchAccel;
  k[kYaw] = yawRate;
  k[kYawRate] = yawAccel;
  k[kPhiL] = wl;
  k[kPhiR] = wr;
  return k;
}

Vec pack(const PlantState &state) {
  return {state.xM,         state.vMps,
          state.pitchRad,   state.pitchRateRadPerSec,
          state.yawRad,     state.yawRateRadPerSec,
          state.leftWheelRad, state.rightWheelRad,
          state.leftWheelRadPerSec, state.rightWheelRadPerSec};
}

void unpack(const Vec &s, PlantState &state) {
  state.xM = s[kX];
  state.vMps = s[kV];
  state.pitchRad = s[kPitch];
  state.pitchRateRadPerSec = s[kPitchRate];
  state.yawRad = s[kYaw];
  state.yawRateRadPerSec = s[kYawRate];
  state.leftWheelRad = s[kPhiL];
  state.rightWheelRad = s[kPhiR];
  state.leftWheelRadPerSec = s[kOmegaL];
  state.rightWheelRadPerSec = s[kOmegaR];
}

// Rolling wheels take their rates from the body state, so they are set from it after each change of the body.
void setRollingWheelRates(PlantState &state, const PlantParams &p) {
  state.leftWheelRadPerSec = leftRolling(p, state.vMps, state.yawRateRadPerSec);
  state.rightWheelRadPerSec = rightRolling(p, state.vMps, state.yawRateRadPerSec);
}

void stopPlant(PlantState &state) {
  state.vMps = 0.0;
  state.pitchRateRadPerSec = 0.0;
  state.yawRateRadPerSec = 0.0;
  state.leftWheelRadPerSec = 0.0;
  state.rightWheelRadPerSec = 0.0;
  state.accelMps2 = 0.0;
  state.pitchAccelRadPerSec2 = 0.0;
}

}  // namespace

void stepPlant(PlantState &state, const PlantParams &params, WheelTorques torques, double dtSec) {
  if (state.fallen) {
    stopPlant(state);
    return;
  }

  Vec s = pack(state);
  if (state.lifted) {
    // The body is held upright at x = 0 and yaw 0; only the wheels turn.
    s[kX] = s[kV] = s[kPitch] = s[kPitchRate] = s[kYaw] = s[kYawRate] = 0.0;
  }

  const Vec k1 = derivative(s, params, torques, state.lifted);
  Vec tmp{};
  for (int i = 0; i < kCount; ++i) tmp[i] = s[i] + 0.5 * dtSec * k1[i];
  const Vec k2 = derivative(tmp, params, torques, state.lifted);
  for (int i = 0; i < kCount; ++i) tmp[i] = s[i] + 0.5 * dtSec * k2[i];
  const Vec k3 = derivative(tmp, params, torques, state.lifted);
  for (int i = 0; i < kCount; ++i) tmp[i] = s[i] + dtSec * k3[i];
  const Vec k4 = derivative(tmp, params, torques, state.lifted);

  Vec next{};
  for (int i = 0; i < kCount; ++i) {
    next[i] = s[i] + dtSec / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
  }
  // The average over the step: the velocity change divided by dt.
  const double accel = (k1[kV] + 2.0 * k2[kV] + 2.0 * k3[kV] + k4[kV]) / 6.0;
  const double pitchAccel = (k1[kPitchRate] + 2.0 * k2[kPitchRate] + 2.0 * k3[kPitchRate] + k4[kPitchRate]) / 6.0;

  unpack(next, state);
  if (state.lifted) {
    state.accelMps2 = 0.0;
    state.pitchAccelRadPerSec2 = 0.0;
    return;
  }

  state.accelMps2 = accel;
  state.pitchAccelRadPerSec2 = pitchAccel;
  setRollingWheelRates(state, params);
  if (std::fabs(state.pitchRad) >= params.groundContactPitchRad) {
    state.fallen = true;
    state.pitchRad = std::copysign(params.groundContactPitchRad, state.pitchRad);
    stopPlant(state);
  }
}

void applyPush(PlantState &state, const PlantParams &params, double impulseNs) {
  if (state.lifted || state.fallen) return;

  // The impulse acts at the centre of mass: generalized impulses (P, P h cos theta) on (axle, pitch).
  const MassMatrix mm = massMatrix(params, state.pitchRad);
  const double r1 = impulseNs;
  const double r2 = impulseNs * params.comHeightM * std::cos(state.pitchRad);
  const double dv = (mm.m22 * r1 - mm.m12 * r2) / mm.det;
  const double dPitchRate = (mm.m11 * r2 - mm.m12 * r1) / mm.det;

  state.vMps += dv;
  state.pitchRateRadPerSec += dPitchRate;
  setRollingWheelRates(state, params);
}

}  // namespace sim
