#pragma once

#include <stdint.h>

struct FootPoseMeters {
  float xMeters;
  float yMeters;
  float bodyPitchDegrees;
};

struct JointAnglesDegrees {
  float joint1Degrees;
  float joint2Degrees;
};

enum class LegSolveStatus : uint8_t {
  Success = 0,
  FirstJointOutOfReach = 1,
  SecondJointOutOfReach = 2
};

struct LegSolveResult {
  // Always finite, but only meaningful when status == Success.
  JointAnglesDegrees angles;
  LegSolveStatus status;
};

namespace LegKinematics {

LegSolveResult solveRight(FootPoseMeters pose);
LegSolveResult solveLeft(FootPoseMeters pose);

}
