#pragma once

#include "Calibration.h"

class CalibrationStore {
public:
  PersistedCalibrationValues load();
  AttitudeBiasDegrees saveAttitude(AttitudeBiasDegrees requested);
  GyroBiasRawCounts saveGyro(GyroBiasRawCounts requested);
  ServoTrimsDegrees saveChangedServos(
      ServoTrimsDegrees requested,
      ServoTrimChangeMask changed);
};
