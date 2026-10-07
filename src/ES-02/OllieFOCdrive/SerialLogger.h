#pragma once

#include "SerialLogFormat.h"

void SerialLoggerBegin();
bool SerialLoggerSubmit(SerialLogRecord row);
void SerialLoggerSetTraceMode(int mode);
bool SerialLoggerTraceModeSelected();
