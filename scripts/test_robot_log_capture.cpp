#include <assert.h>
#include <stdint.h>

#include "Logging.h"
#include "RobotLogCapture.h"

namespace Logging {
Profile gProfile = Profile::Idle;

void setProfile(Profile profile) { gProfile = profile; }
}  // namespace Logging

int main() {
  using Logging::Profile;
  using RobotLogCapture::profileForSelector;

  assert(profileForSelector(0) == Profile::Idle);
  assert(profileForSelector(1) == Profile::SelectedDebug);
  assert(profileForSelector(45) == Profile::SelectedDebug);
  assert(profileForSelector(46) == Profile::Idle);
  assert(profileForSelector(55) == Profile::Trace);
  assert(profileForSelector(56) == Profile::Control);
  assert(profileForSelector(57) == Profile::Balance);
  assert(profileForSelector(58) == Profile::Drive);
  assert(profileForSelector(59) == Profile::Idle);
  assert(profileForSelector(60) == Profile::SelectedDebug);
  assert(profileForSelector(75) == Profile::SelectedDebug);
  assert(profileForSelector(76) == Profile::Idle);

  assert(!RobotLogCapture::isMeasurementProfile(Profile::Idle));
  assert(!RobotLogCapture::isMeasurementProfile(Profile::SelectedDebug));
  assert(RobotLogCapture::isMeasurementProfile(Profile::Diagnostic));
  assert(RobotLogCapture::isMeasurementProfile(Profile::Trace));
  assert(RobotLogCapture::isMeasurementProfile(Profile::Control));
  assert(RobotLogCapture::isMeasurementProfile(Profile::Balance));
  assert(RobotLogCapture::isMeasurementProfile(Profile::Drive));

  RobotLogCapture::setProfileForSelector(56);
  assert(Logging::gProfile == Profile::Control);
  RobotLogCapture::setProfileForSelector(77);
  assert(Logging::gProfile == Profile::Idle);

  using Logging::DebugSelector;
  assert(!RobotLogCapture::selectedDebugDue(DebugSelector::K8, 49));
  assert(RobotLogCapture::selectedDebugDue(DebugSelector::K8, 50));
  assert(!RobotLogCapture::selectedDebugDue(DebugSelector::K8, 99));
  assert(RobotLogCapture::selectedDebugDue(DebugSelector::K8, 100));
  assert(!RobotLogCapture::selectedDebugDue(static_cast<DebugSelector>(46), 100));

  assert(!RobotLogCapture::selectedDebugDue(DebugSelector::K60, 49));
  assert(RobotLogCapture::selectedDebugDue(DebugSelector::K60, 50));
  assert(!RobotLogCapture::selectedDebugDue(DebugSelector::K75, 99));
  assert(RobotLogCapture::selectedDebugDue(DebugSelector::K75, 100));

  RobotLogCapture::resetControlWindow();
  int32_t ranges[4] = {-1, -1, -1, -1};
  RobotLogCapture::copyServoRanges(ranges);
  for (int32_t range : ranges) assert(range == 0);

  const int32_t firstAngles[4] = {10, -10, 4, -4};
  const int32_t secondAngles[4] = {12, -12, 2, -2};
  const int32_t thirdAngles[4] = {11, -11, 3, -3};
  RobotLogCapture::observeServoAngles(firstAngles);
  RobotLogCapture::observeServoAngles(secondAngles);
  RobotLogCapture::copyServoRanges(ranges);
  for (int32_t range : ranges) assert(range == 2);

  RobotLogCapture::observeServoAngles(thirdAngles);
  RobotLogCapture::copyServoRanges(ranges);
  for (int32_t range : ranges) assert(range == 1);

  RobotLogCapture::resetControlWindow();
  RobotLogCapture::observeServoAngles(firstAngles);
  RobotLogCapture::copyServoRanges(ranges);
  for (int32_t range : ranges) assert(range == 0);
  return 0;
}
