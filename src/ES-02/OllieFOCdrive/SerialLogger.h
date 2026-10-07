#pragma once

#include "SerialLogFormat.h"

enum class SerialLoggerIncompleteReason : uint8_t {
  None,
  QueueFull,
  SenderStartFailed,
  SenderWriteFailed
};

constexpr uint32_t kSerialLoggerQueueDepth = 32;

void SerialLoggerBegin();
bool SerialLoggerSubmit(SerialLogRecord row);
SerialLoggerIncompleteReason SerialLoggerIncomplete();
uint8_t SerialLoggerRejectedCount();
void SerialLoggerSetSelectedMode(int mode);
bool SerialLoggerSelectedMode();
