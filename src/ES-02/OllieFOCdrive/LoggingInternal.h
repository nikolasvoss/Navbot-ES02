#pragma once

#include <stddef.h>

#include "LogSamples.h"

namespace LoggingInternal {

enum class RecordKind : uint8_t { Message, Diagnostic, Trace, Control, Balance, Drive, Debug };

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

static_assert(sizeof(Record) <= 224, "logging queue slot exceeds the 224-byte budget");

// Sender line buffer. The widest DRIVE row (49 columns, floats at -9999.0, uint32 at 0xffffffff) is about 525
// bytes; tests/serial_logger/test_serial_logger.cpp checks that row against this bound.
constexpr size_t kLineBufferSize = 576;

int formatRecord(const Record &record, char *buffer, size_t capacity, uint32_t droppedRecords, uint32_t writeFailures);

}  // namespace LoggingInternal
