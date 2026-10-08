#include <math.h>

namespace baseline {

constexpr float BODY_THIGH_LENGTH_M = 0.035f;
constexpr float BODY_SHANK_LENGTH_M = 0.072f;
constexpr double PI = 3.1415926535897932384626433832795;

float AngleToArc(float angle) { return angle * (PI / 180); }
float ArcToAngle(float arc) { return arc * (180 / PI); }

int RightInverseKinematics(float x, float y, float p, float *ax) {
  x = x < -0.05 ? -0.05 : (x > 0.05 ? 0.05 : x);
  y = y < 0.05 ? 0.05 : (y > 0.1 ? 0.1 : y);
  int error = 0;
  float AB = BODY_THIGH_LENGTH_M;
  float BC = BODY_SHANK_LENGTH_M;
  float OA = 0.017f;
  float aOCF = 0, aOCF2 = 0, aAOC = 0, OC = 0, OF = 0, FC = 0, AC = 0;
  float aOAC = 0, aOCA = 0, aBAC = 0, aBAG = 0;
  float OE = OA, aEOC = 0, EC = 0, aOCE = 0, aOEC = 0, aDEC = 0, aDEH = 0;
  float pitch, x1, y1;
  pitch = AngleToArc(p);
  x1 = x * cosf(pitch) - y * sinf(pitch);
  y1 = x * sinf(pitch) + y * cosf(pitch);
  OF = x1;
  FC = y1;
  OC = sqrtf(pow(OF, 2) + pow(FC, 2));
  aOCF = asinf(OF / OC);
  aAOC = AngleToArc(90) + aOCF;
  AC = sqrtf(pow(OA, 2) + pow(OC, 2) - 2 * OA * OC * cos(aAOC));
  aOCA = acosf((pow(OC, 2) + pow(AC, 2) - pow(OA, 2)) / (2 * OC * AC));
  aOAC = PI - aOCA - aAOC;
  aBAC = acos((pow(AB, 2) + pow(AC, 2) - pow(BC, 2)) / (2 * AB * AC));
  aBAG = PI - aBAC - aOAC;
  ax[0] = ArcToAngle(aBAG);
  aOCF2 = -aOCF;
  aEOC = AngleToArc(90) + aOCF2;
  EC = sqrtf(pow(OE, 2) + pow(OC, 2) - 2 * OE * OC * cos(aEOC));
  aOCE = acosf((pow(OC, 2) + pow(EC, 2) - pow(OE, 2)) / (2 * OC * EC));
  aOEC = PI - aOCE - aEOC;
  aDEC = acos((pow(AB, 2) + pow(EC, 2) - pow(BC, 2)) / (2 * AB * EC));
  aDEH = PI - aDEC - aOEC;
  ax[1] = ArcToAngle(aDEH);
  if (AC >= (AB + BC)) return error = 1;
  else if (EC >= (AB + BC)) return error = 2;
  return error;
}

int LeftInverseKinematics(float x, float y, float p, float *ax) {
  x = x < -0.05 ? -0.05 : (x > 0.05 ? 0.05 : x);
  y = y < 0.05 ? 0.05 : (y > 0.1 ? 0.1 : y);
  x = -x;
  p = -p;
  int error = 0;
  float AB = BODY_THIGH_LENGTH_M;
  float BC = BODY_SHANK_LENGTH_M;
  float OA = 0.017f;
  float aOCF = 0, aOCF2 = 0, aAOC = 0, OC = 0, OF = 0, FC = 0, AC = 0;
  float aOAC = 0, aOCA = 0, aBAC = 0, aBAG = 0;
  float OE = OA, aEOC = 0, EC = 0, aOCE = 0, aOEC = 0, aDEC = 0, aDEH = 0;
  float pitch, x1, y1;
  pitch = AngleToArc(p);
  x1 = x * cosf(pitch) - y * sinf(pitch);
  y1 = x * sinf(pitch) + y * cosf(pitch);
  OF = -x1;
  FC = y1;
  OC = sqrtf(pow(OF, 2) + pow(FC, 2));
  aOCF = asinf(OF / OC);
  aAOC = AngleToArc(90) + aOCF;
  AC = sqrtf(pow(OA, 2) + pow(OC, 2) - 2 * OA * OC * cos(aAOC));
  aOCA = acosf((pow(OC, 2) + pow(AC, 2) - pow(OA, 2)) / (2 * OC * AC));
  aOAC = PI - aOCA - aAOC;
  aBAC = acos((pow(AB, 2) + pow(AC, 2) - pow(BC, 2)) / (2 * AB * AC));
  aBAG = PI - aBAC - aOAC;
  ax[0] = ArcToAngle(aBAG);
  aOCF2 = -aOCF;
  aEOC = AngleToArc(90) + aOCF2;
  EC = sqrtf(pow(OE, 2) + pow(OC, 2) - 2 * OE * OC * cos(aEOC));
  aOCE = acosf((pow(OC, 2) + pow(EC, 2) - pow(OE, 2)) / (2 * OC * EC));
  aOEC = PI - aOCE - aEOC;
  aDEC = acos((pow(AB, 2) + pow(EC, 2) - pow(BC, 2)) / (2 * AB * EC));
  aDEH = PI - aDEC - aOEC;
  ax[1] = ArcToAngle(aDEH);
  if (AC >= (AB + BC)) return error = 1;
  else if (EC >= (AB + BC)) return error = 2;
  return error;
}

}
