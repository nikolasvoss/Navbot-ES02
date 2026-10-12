#include "LegKinematics.h"

#include <math.h>

#include "MathConstants.h"

namespace {

constexpr float BODY_THIGH_LENGTH_M = 0.035f;
constexpr float BODY_SHANK_LENGTH_M = 0.072f;
constexpr float FOOT_OFFSET_M = 0.017f;
constexpr float FOOT_X_LIMIT_M = 0.05f;
constexpr float FOOT_Y_MIN_M = 0.05f;
constexpr float FOOT_Y_MAX_M = 0.1f;

float arcToAngle(float arc) {
  return arc * (180 / kPi);
}

float angleToArc(float angle) {
  return angle * (kPi / 180);
}

float clampX(float x) {
  return x < -FOOT_X_LIMIT_M ? -FOOT_X_LIMIT_M : (x > FOOT_X_LIMIT_M ? FOOT_X_LIMIT_M : x);
}

float clampY(float y) {
  return y < FOOT_Y_MIN_M ? FOOT_Y_MIN_M : (y > FOOT_Y_MAX_M ? FOOT_Y_MAX_M : y);
}

double square(double value) {
  return value * value;
}

// Law-of-cosines ratio clamped to [-1, 1] so acos and asin stay finite.
double unitRatio(double numerator, double denominator) {
  if (denominator <= 0)
    return 0;
  const double ratio = numerator / denominator;
  return ratio < -1 ? -1 : (ratio > 1 ? 1 : ratio);
}

// Left and right legs share this solver: mirroring the left leg negates x and
// pitch and then OF, which cancels exactly. The double intermediates, the float
// storage points and the acos precision of each joint match the previous
// implementation because acos near full extension amplifies rounding
// differences far beyond float epsilon.
LegSolveResult solveLeg(FootPoseMeters pose) {
  const float clampedX = clampX(pose.xMeters);
  const float clampedY = clampY(pose.yMeters);
  const bool poseValid = isfinite(clampedX) && isfinite(clampedY) && isfinite(pose.bodyPitchDegrees);
  const float x = poseValid ? clampedX : 0;
  const float y = poseValid ? clampedY : FOOT_Y_MIN_M;
  const float pitch = angleToArc(poseValid ? pose.bodyPitchDegrees : 0);

  const float AB = BODY_THIGH_LENGTH_M;
  const float BC = BODY_SHANK_LENGTH_M;
  const float OA = FOOT_OFFSET_M;
  const float OE = OA;

  const float OF = x * cosf(pitch) - y * sinf(pitch);
  const float FC = x * sinf(pitch) + y * cosf(pitch);

  const float OC = sqrtf(square(OF) + square(FC));
  const float aOCF = asinf(unitRatio(OF, OC));
  const float aAOC = angleToArc(90) + aOCF;
  const float AC = sqrtf(square(OA) + square(OC) - 2 * OA * OC * cosf(aAOC));

  const float aEOC = angleToArc(90) - aOCF;
  const float EC = sqrtf(square(OE) + square(OC) - 2 * OE * OC * cosf(aEOC));

  LegSolveStatus status = LegSolveStatus::Success;
  if (!poseValid || AC >= (AB + BC))
    status = LegSolveStatus::FirstJointOutOfReach;
  else if (EC >= (AB + BC))
    status = LegSolveStatus::SecondJointOutOfReach;

  const float aOCA = acosf(unitRatio(square(OC) + square(AC) - square(OA), 2 * OC * AC));
  const float aOAC = kPi - aOCA - aAOC;
  const float aBAC = acos(unitRatio(square(AB) + square(AC) - square(BC), 2 * AB * AC));
  const float aBAG = kPi - aBAC - aOAC;

  const float aOCE = acosf(unitRatio(square(OC) + square(EC) - square(OE), 2 * OC * EC));
  const float aOEC = kPi - aOCE - aEOC;
  const float aDEC = acos(unitRatio(square(AB) + square(EC) - square(BC), 2 * AB * EC));
  const float aDEH = kPi - aDEC - aOEC;

  return {{arcToAngle(aBAG), arcToAngle(aDEH)}, status};
}

}

LegSolveResult LegKinematics::solveRight(FootPoseMeters pose) {
  return solveLeg(pose);
}

LegSolveResult LegKinematics::solveLeft(FootPoseMeters pose) {
  return solveLeg(pose);
}
