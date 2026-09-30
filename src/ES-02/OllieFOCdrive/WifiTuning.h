#pragma once

#include "TuningParameters.h"
#include "WifiTuningConfig.h"
#include "TelemetryWire.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

struct WifiTuningState {
  bool supportedMode;
  bool rcValid;
  uint32_t rcAgeMs;
  bool ch5Off;
  float tuningMode;
  uint32_t bootId;
  bool calibrationInactive;
  uint32_t imuOdrHz;
  uint32_t filterCutoffHz;
  float rollZeroBiasDeg;
  float pitchZeroBiasDeg;
  float gyroBiasXCounts;
  float gyroBiasYCounts;
  float gyroBiasZCounts;
};

void WifiTuningBegin();
void WifiTuningReprovision(char *cmd);
void WifiTuningProcessOne(const WifiTuningState &state);
bool WifiTuningRecordingActive();
bool WifiTuningMutationLocked();
void WifiTuningRecordingTick(const telemetry::Sample &sample, const WifiTuningState &state);
