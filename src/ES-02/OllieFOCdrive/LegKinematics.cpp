#include "LegKinematics.h"

#include <math.h>

namespace {

constexpr float BODY_THIGH_LENGTH_M = 0.035f;
constexpr float BODY_SHANK_LENGTH_M = 0.072f;
constexpr float FOOT_OFFSET_M = 0.017f;
constexpr double PI = 3.1415926535897932384626433832795;

float arcToAngle(float arc) {
  return arc * (180 / PI);
}

float angleToArc(float angle) {
  return angle * (PI / 180);
}

float clampX(float x) {
  return x < -0.05 ? -0.05 : (x > 0.05 ? 0.05 : x);
}

float clampY(float y) {
  return y < 0.05 ? 0.05 : (y > 0.1 ? 0.1 : y);
}

}

LegSolveResult LegKinematics::solveRight(FootPoseMeters pose) {
  float x = clampX(pose.xMeters);
  float y = clampY(pose.yMeters);
  float p = pose.bodyPitchDegrees;

  int error = 0;
  float AB = BODY_THIGH_LENGTH_M;
  float BC = BODY_SHANK_LENGTH_M;
  float OA = FOOT_OFFSET_M;
  float OE = OA;

  float aOCF = 0;
  float aOCF2 = 0;
  float aAOC = 0;
  float OC = 0;
  float OF = 0;
  float FC = 0;
  float AC = 0;
  float aOAC = 0;
  float aOCA = 0;
  float aBAC = 0;
  float aBAG = 0;
  float aEOC = 0;
  float EC = 0;
  float aOCE = 0;
  float aOEC = 0;
  float aDEC = 0;
  float aDEH = 0;

  float pitch, x1, y1;

  pitch = angleToArc(p);

  x1 = x * cosf(pitch) - y * sinf(pitch);
  y1 = x * sinf(pitch) + y * cosf(pitch);

  OF = x1;
  FC = y1;

  OC = sqrtf(pow(OF, 2) + pow(FC, 2));
  aOCF = asinf(OF / OC);
  aAOC = angleToArc(90) + aOCF;

  AC = sqrtf(pow(OA, 2) + pow(OC, 2) - 2 * OA * OC * cos(aAOC));
  aOCA = acosf((pow(OC, 2) + pow(AC, 2) - pow(OA, 2)) / (2 * OC * AC));
  aOAC = PI - aOCA - aAOC;
  aBAC = acos((pow(AB, 2) + pow(AC, 2) - pow(BC, 2)) / (2 * AB * AC));
  aBAG = PI - aBAC - aOAC;
  JointAnglesDegrees angles;
  angles.joint1Degrees = arcToAngle(aBAG);

  aOCF2 = -aOCF;
  aEOC = angleToArc(90) + aOCF2;

  EC = sqrtf(pow(OE, 2) + pow(OC, 2) - 2 * OE * OC * cos(aEOC));
  aOCE = acosf((pow(OC, 2) + pow(EC, 2) - pow(OE, 2)) / (2 * OC * EC));
  aOEC = PI - aOCE - aEOC;
  aDEC = acos((pow(AB, 2) + pow(EC, 2) - pow(BC, 2)) / (2 * AB * EC));
  aDEH = PI - aDEC - aOEC;
  angles.joint2Degrees = arcToAngle(aDEH);

  if (AC >= (AB + BC))
    error = 1;
  else if (EC >= (AB + BC))
    error = 2;

  return {angles, static_cast<LegSolveStatus>(error)};
}

LegSolveResult LegKinematics::solveLeft(FootPoseMeters pose) {
  float x = clampX(pose.xMeters);
  float y = clampY(pose.yMeters);
  float p = pose.bodyPitchDegrees;

  x = -x;
  p = -p;
  int error = 0;
  float AB = BODY_THIGH_LENGTH_M;
  float BC = BODY_SHANK_LENGTH_M;
  float OA = FOOT_OFFSET_M;
  float OE = OA;

  float aOCF = 0;
  float aOCF2 = 0;
  float aAOC = 0;
  float OC = 0;
  float OF = 0;
  float FC = 0;
  float AC = 0;
  float aOAC = 0;
  float aOCA = 0;
  float aBAC = 0;
  float aBAG = 0;
  float aEOC = 0;
  float EC = 0;
  float aOCE = 0;
  float aOEC = 0;
  float aDEC = 0;
  float aDEH = 0;

  float pitch, x1, y1;

  pitch = angleToArc(p);

  x1 = x * cosf(pitch) - y * sinf(pitch);
  y1 = x * sinf(pitch) + y * cosf(pitch);

  OF = -x1;
  FC = y1;

  OC = sqrtf(pow(OF, 2) + pow(FC, 2));
  aOCF = asinf(OF / OC);
  aAOC = angleToArc(90) + aOCF;

  AC = sqrtf(pow(OA, 2) + pow(OC, 2) - 2 * OA * OC * cos(aAOC));
  aOCA = acosf((pow(OC, 2) + pow(AC, 2) - pow(OA, 2)) / (2 * OC * AC));
  aOAC = PI - aOCA - aAOC;
  aBAC = acos((pow(AB, 2) + pow(AC, 2) - pow(BC, 2)) / (2 * AB * AC));
  aBAG = PI - aBAC - aOAC;
  JointAnglesDegrees angles;
  angles.joint1Degrees = arcToAngle(aBAG);

  aOCF2 = -aOCF;
  aEOC = angleToArc(90) + aOCF2;

  EC = sqrtf(pow(OE, 2) + pow(OC, 2) - 2 * OE * OC * cos(aEOC));
  aOCE = acosf((pow(OC, 2) + pow(EC, 2) - pow(OE, 2)) / (2 * OC * EC));
  aOEC = PI - aOCE - aEOC;
  aDEC = acos((pow(AB, 2) + pow(EC, 2) - pow(BC, 2)) / (2 * AB * EC));
  aDEH = PI - aDEC - aOEC;
  angles.joint2Degrees = arcToAngle(aDEH);

  if (AC >= (AB + BC))
    error = 1;
  else if (EC >= (AB + BC))
    error = 2;

  return {angles, static_cast<LegSolveStatus>(error)};
}
