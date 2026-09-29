#pragma once

#include "TuningParameters.h"
#include "WifiTuningConfig.h"
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
};

void WifiTuningBegin();
void WifiTuningReprovision(char *cmd);
void WifiTuningProcessOne(const WifiTuningState &state);
