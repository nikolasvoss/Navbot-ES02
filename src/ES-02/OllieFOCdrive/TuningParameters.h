#pragma once

#include <stddef.h>

struct TuningParameter {
  const char *name;
  float minimum;
  float maximum;
  const char *description;
  float *value;
  bool isMode;
};

struct TuningValue {
  const char *name;
  float value;
};

constexpr size_t TUNING_PARAMETER_COUNT = 18;

void bindTuningParameters(float *angleP, float *angleI, float *angleD, float *angleL,
                          float *speedP, float *speedI, float *speedD, float *speedL,
                          float *yawP, float *yawI, float *yawD, float *yawL,
                          float *rollP, float *rollI, float *rollD, float *rollL,
                          float *tuningMode, float *wheelSpeedFeedbackGain);
const TuningParameter *findTuningParameter(const char *name);
const TuningParameter *tuningParameterAt(size_t index);
bool isSbusFresh(bool hasFrame, unsigned long ageMs, bool failsafeOk);
const char *validateTuningWritePolicy(bool supportedMode, bool rcValid, bool ch5Off, const TuningValue *values, size_t count, float currentMode);
bool hasDuplicateJsonObjectKeys(const char *body, size_t length);
bool parseTuningLine(const char *line, char *nameOut, size_t nameCapacity, bool *hasValue, float *valueOut);
bool validateTuningBatch(const TuningValue *values, size_t count, float currentMode, const char **errorCode);
