#pragma once

#include "LogSamples.h"

#include <stddef.h>

namespace LoggingInternal {

enum class RecordKind : uint8_t {
  Message,
  Diagnostic,
  Trace,
  Control,
  Balance,
  Drive,
  Debug
};

struct MessageRecord {
  Logging::Level level;
  uint8_t tagLength;
  uint8_t textLength;
  char tag[Logging::kMessageTagSize];
  char text[Logging::kMessageTextSize];
};

union Payload {
  MessageRecord message;
  Logging::DiagnosticSample diagnostic;
  Logging::TraceSample trace;
  Logging::ControlSample control;
  Logging::BalanceSample balance;
  Logging::DriveSample drive;
  Logging::DebugSample debug;
};

struct Record {
  uint32_t epoch;
  Logging::Profile profile;
  RecordKind kind;
  Payload payload;
};

static_assert(sizeof(Record) <= 168, "logging queue slot exceeds the 168-byte budget");

int formatRecord(const Record &record, char *buffer, size_t capacity);

}  // namespace LoggingInternal
