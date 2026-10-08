#include "LegKinematics.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace baseline {
int RightInverseKinematics(float x, float y, float p, float *angles);
int LeftInverseKinematics(float x, float y, float p, float *angles);
}

namespace {

enum class FloatClass { Finite, PositiveInfinity, NegativeInfinity, NaN };

FloatClass classify(float value) {
  if (std::isnan(value)) return FloatClass::NaN;
  if (std::isinf(value))
    return std::signbit(value) ? FloatClass::NegativeInfinity : FloatClass::PositiveInfinity;
  return FloatClass::Finite;
}

void require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "%s\n", message);
    std::exit(1);
  }
}

LegSolveResult solve(bool left, FootPoseMeters pose) {
  return left ? LegKinematics::solveLeft(pose) : LegKinematics::solveRight(pose);
}

int solveBaseline(bool left, FootPoseMeters pose, float *angles) {
  return left ? baseline::LeftInverseKinematics(pose.xMeters, pose.yMeters, pose.bodyPitchDegrees, angles)
              : baseline::RightInverseKinematics(pose.xMeters, pose.yMeters, pose.bodyPitchDegrees, angles);
}

void compareWithBaseline(bool left, FootPoseMeters pose, int statusCounts[3]) {
  float expectedAngles[2];
  int expectedStatus = solveBaseline(left, pose, expectedAngles);
  LegSolveResult actual = solve(left, pose);
  require(static_cast<int>(actual.status) == expectedStatus, "solve status differs from baseline");
  require(expectedStatus >= 0 && expectedStatus < 3, "baseline status outside frozen status set");
  ++statusCounts[expectedStatus];
  const float actualAngles[] = {actual.angles.joint1Degrees, actual.angles.joint2Degrees};
  for (int index = 0; index < 2; ++index) {
    FloatClass expectedClass = classify(expectedAngles[index]);
    require(classify(actualAngles[index]) == expectedClass, "angle finite classification differs from baseline");
    if (expectedClass == FloatClass::Finite) {
      require(std::fabs(actualAngles[index] - expectedAngles[index]) <= 0.00001f,
              "finite angle differs from baseline by more than 1e-5 degrees");
    }
  }
}

}

int main() {
  const float xValues[] = {-0.12f, -0.05f, -0.049f, -0.02f, 0.0f, 0.02f, 0.049f, 0.05f, 0.12f};
  const float yValues[] = {0.0f, 0.049f, 0.05f, 0.06f, 0.075f, 0.09f, 0.1f, 0.12f};
  const float pitchValues[] = {-45.0f, -20.0f, -12.0f, -1.0f, 0.0f, 1.0f, 12.0f, 20.0f, 45.0f};
  int rightStatusCounts[3] = {0, 0, 0};
  int leftStatusCounts[3] = {0, 0, 0};

  for (float x : xValues) {
    for (float y : yValues) {
      for (float pitch : pitchValues) {
        FootPoseMeters pose{x, y, pitch};
        compareWithBaseline(false, pose, rightStatusCounts);
        compareWithBaseline(true, pose, leftStatusCounts);
      }
    }
  }

  const float infinity = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const FootPoseMeters nonfiniteCases[] = {
      {nan, 0.07f, 0.0f}, {infinity, 0.07f, 0.0f}, {-infinity, 0.07f, 0.0f},
      {0.01f, nan, 0.0f}, {0.01f, infinity, 0.0f}, {0.01f, -infinity, 0.0f},
      {0.01f, 0.07f, nan}, {0.01f, 0.07f, infinity}, {0.01f, 0.07f, -infinity}};
  for (FootPoseMeters pose : nonfiniteCases) {
    compareWithBaseline(false, pose, rightStatusCounts);
    compareWithBaseline(true, pose, leftStatusCounts);
  }

  const float clampInputs[][2] = {
      {-0.12f, 0.07f}, {-0.05f, 0.07f}, {0.05f, 0.07f}, {0.12f, 0.07f},
      {0.01f, 0.0f}, {0.01f, 0.05f}, {0.01f, 0.1f}, {0.01f, 0.12f}};
  for (const auto &input : clampInputs) {
    float clampedX = input[0] < -0.05f ? -0.05f : (input[0] > 0.05f ? 0.05f : input[0]);
    float clampedY = input[1] < 0.05f ? 0.05f : (input[1] > 0.1f ? 0.1f : input[1]);
    for (int side = 0; side < 2; ++side) {
      bool left = side == 1;
      LegSolveResult actual = solve(left, {input[0], input[1], 7.0f});
      LegSolveResult boundary = solve(left, {clampedX, clampedY, 7.0f});
      require(actual.status == boundary.status, "out-of-range pose did not match clamped status");
      require(classify(actual.angles.joint1Degrees) == classify(boundary.angles.joint1Degrees) &&
                  classify(actual.angles.joint2Degrees) == classify(boundary.angles.joint2Degrees),
              "out-of-range pose did not match clamped angle classifications");
      if (std::isfinite(actual.angles.joint1Degrees) && std::isfinite(boundary.angles.joint1Degrees)) {
        require(actual.angles.joint1Degrees == boundary.angles.joint1Degrees &&
                    actual.angles.joint2Degrees == boundary.angles.joint2Degrees,
                "out-of-range pose did not match exact clamp boundary result");
      }
    }
  }

  require(rightStatusCounts[0] > 0 && leftStatusCounts[0] > 0, "success status was not exercised");
  require(rightStatusCounts[1] > 0 && leftStatusCounts[1] > 0, "first-joint reach status was not exercised");
  require(rightStatusCounts[2] > 0 && leftStatusCounts[2] > 0, "second-joint reach status was not exercised");

  std::printf("PASS: baseline equivalence across %d finite poses per side, 9 nonfinite cases per side, and clamp boundaries\n",
              static_cast<int>(sizeof(xValues) / sizeof(xValues[0]) * sizeof(yValues) / sizeof(yValues[0]) *
                               sizeof(pitchValues) / sizeof(pitchValues[0])));
  return 0;
}
