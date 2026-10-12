#include "OllieFOCdrive.h"

#include <cmath>
#include <cstdio>

static int failures = 0;

static void expectNear(const char *what, float actual, float expected, float tolerance = 0.0f) {
  if (std::fabs(actual - expected) > tolerance) {
    std::printf("FAIL %s: got %.6f, expected %.6f\n", what, actual, expected);
    ++failures;
  }
}

static void constructorStoresLimitsAndStartsAtZero() {
  MyPIDController pid(1.0f, 2.0f, 3.0f, 5.0f, 7.0f, 0.5f, 0.0f, 0.0f);

  expectNear("ctor Kp", pid.Kp, 1.0f);
  expectNear("ctor Ki", pid.Ki, 2.0f);
  expectNear("ctor Kd", pid.Kd, 3.0f);
  expectNear("ctor iLimit", pid.iLimit, 5.0f);
  expectNear("ctor outputLimit", pid.outputLimit, 7.0f);
  expectNear("ctor integral", pid.integral, 0.0f);
  expectNear("ctor previousError", pid.previousError, 0.0f);
  expectNear("ctor deriv", pid.deriv, 0.0f);
  expectNear("ctor output", pid.output, 0.0f);
}

static void firstComputeHasNoDerivativeKick() {
  MyPIDController pid(1.0f, 2.0f, 3.0f, 0.0f, 10.0f, 0.5f, 0.0f, 0.0f);

  const float output = pid.compute(2.0f, 0.5f);

  expectNear("first deriv", pid.deriv, 4.0f);
  expectNear("first integral", pid.integral, 1.0f);
  expectNear("first outP", pid.outP, 2.0f);
  expectNear("first outI", pid.outI, 2.0f);
  expectNear("first outD", pid.outD, 12.0f);
  expectNear("first output clamped to outputLimit", output, 10.0f);
}

static void outputLimitClampsBothSides() {
  MyPIDController pid(1.0f, 0.0f, 0.0f, 0.0f, 7.0f, 1.0f, 0.0f, 0.0f);

  expectNear("positive output clamp", pid.compute(100.0f, 1.0f), 7.0f);
  expectNear("negative output clamp", pid.compute(-100.0f, 1.0f), -7.0f);
  expectNear("output within limit", pid.compute(3.0f, 1.0f), 3.0f);
}

static void zeroOutputLimitMeansUnlimited() {
  MyPIDController pid(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);

  expectNear("unlimited output", pid.compute(100.0f, 1.0f), 100.0f);
}

static void integralLimitClampsBothSides() {
  MyPIDController pid(0.0f, 1.0f, 0.0f, 3.0f, 0.0f, 1.0f, 0.0f, 0.0f);

  pid.compute(10.0f, 1.0f);
  expectNear("positive integral clamp", pid.integral, 3.0f);
  expectNear("positive outI", pid.outI, 3.0f);

  pid.compute(-100.0f, 1.0f);
  expectNear("negative integral clamp", pid.integral, -3.0f);
  expectNear("negative outI", pid.outI, -3.0f);
}

static void setPidStoresLimitsAndResetsState() {
  MyPIDController pid(1.0f, 1.0f, 1.0f, 50.0f, 0.0f, 1.0f, 0.0f, 0.0f);
  pid.compute(4.0f, 1.0f);
  pid.compute(6.0f, 1.0f);

  pid.setPID(1.0f, 0.0f, 0.0f, 2.0f, 4.0f, 1.0f, 0.0f, 0.0f);

  expectNear("setPID iLimit", pid.iLimit, 2.0f);
  expectNear("setPID outputLimit", pid.outputLimit, 4.0f);
  expectNear("setPID integral", pid.integral, 0.0f);
  expectNear("setPID previousError", pid.previousError, 0.0f);
  expectNear("setPID output clamp", pid.compute(100.0f, 1.0f), 4.0f);
}

static void derivativeFilterAttenuatesFirstStep() {
  // 16 Hz Butterworth LPF at 128 Hz: b0 = 0.0976311, so the first filtered
  // derivative of a unit step over dt = 1/128 s is 0.0976311 * 128.
  MyPIDController pid(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0078125f, 1.0f, 16.0f);

  pid.compute(1.0f, 0.0078125f);

  expectNear("filtered first deriv", pid.deriv, 12.49678f, 1e-3f);
}

static void setPidEnablesDerivativeFilter() {
  MyPIDController pid(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0078125f, 0.0f, 0.0f);
  pid.setPID(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0078125f, 1.0f, 16.0f);

  pid.compute(1.0f, 0.0078125f);

  expectNear("setPID filtered first deriv", pid.deriv, 12.49678f, 1e-3f);
}

int main() {
  constructorStoresLimitsAndStartsAtZero();
  firstComputeHasNoDerivativeKick();
  outputLimitClampsBothSides();
  zeroOutputLimitMeansUnlimited();
  integralLimitClampsBothSides();
  setPidStoresLimitsAndResetsState();
  derivativeFilterAttenuatesFirstStep();
  setPidEnablesDerivativeFilter();

  if (failures != 0) {
    std::printf("%d PID checks failed\n", failures);
    return 1;
  }
  std::printf("PID checks passed\n");
  return 0;
}
