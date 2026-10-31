#pragma once

// The angle limit of a leg servo, in degrees either side of centre. ServoControl clamps the commanded angle to it, and a
// servo trim is refused beyond it. Kept apart from ServoControl.h, which needs Arduino, so the host tests can use it.
constexpr int kServoAngleLimitDeg = 90;
