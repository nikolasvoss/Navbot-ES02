#include "Logging.h"

#include <esp_attr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <atomic>

#include "Arduino.h"
#include "LoggingInternal.h"

namespace {
constexpr UBaseType_t kQueueDepth = Logging::kQueueDepth;
constexpr uint32_t kSenderTaskStackDepth = 4096;
constexpr UBaseType_t kSenderTaskPriority = 1;
constexpr uint32_t kSelectedDebugMinIntervalUs = 2500;

StaticQueue_t queueControl DRAM_ATTR;
uint8_t queueStorage[kQueueDepth * sizeof(LoggingInternal::Record)] DRAM_ATTR;
QueueHandle_t logQueue = nullptr;

struct ProfileSnapshot {
  Logging::Profile profile;
  uint32_t epoch;
};

portMUX_TYPE profileMux = portMUX_INITIALIZER_UNLOCKED;
Logging::Profile currentProfile = Logging::Profile::Idle;
uint32_t currentEpoch = 0;

std::atomic<uint8_t> incompleteReason{static_cast<uint8_t>(Logging::IncompleteReason::None)};
std::atomic<uint32_t> droppedRecords{0};
std::atomic<uint32_t> writeFailures{0};
std::atomic<bool> beginAttempted{false};
std::atomic<bool> senderStarted{false};

uint32_t lastSelectedDebugMicros = 0;
bool selectedDebugSubmitted = false;

ProfileSnapshot snapshotProfile() {
  portENTER_CRITICAL(&profileMux);
  const ProfileSnapshot result = {currentProfile, currentEpoch};
  portEXIT_CRITICAL(&profileMux);
  return result;
}

void latchIncomplete(Logging::IncompleteReason reason) {
  uint8_t expected = static_cast<uint8_t>(Logging::IncompleteReason::None);
  incompleteReason.compare_exchange_strong(expected, static_cast<uint8_t>(reason), std::memory_order_relaxed);
}

bool isIncomplete() {
  return incompleteReason.load(std::memory_order_relaxed) != static_cast<uint8_t>(Logging::IncompleteReason::None);
}

bool isValidDebugSelector(Logging::DebugSelector selector) {
  const int value = static_cast<int>(selector);
  return Logging::isSelectedDebugSelector(value) || Logging::isChannelDebugSelector(value);
}

Logging::Result enqueue(LoggingInternal::Record &record) {
  if (isIncomplete()) return Logging::Result::Incomplete;
  if (!logQueue || !senderStarted.load(std::memory_order_relaxed)) {
    latchIncomplete(Logging::IncompleteReason::SenderStartFailed);
    return Logging::Result::NotStarted;
  }

  if (xQueueSend(logQueue, &record, 0) == pdTRUE) return Logging::Result::Accepted;

  droppedRecords.fetch_add(1, std::memory_order_relaxed);
  latchIncomplete(Logging::IncompleteReason::QueueFull);
  return Logging::Result::QueueFull;
}

void senderTask(void *) {
  LoggingInternal::Record record;
  char line[LoggingInternal::kLineBufferSize];
  for (;;) {
    if (xQueueReceive(logQueue, &record, portMAX_DELAY) != pdTRUE) continue;

    const ProfileSnapshot profile = snapshotProfile();
    if (record.epoch != profile.epoch || record.profile != profile.profile) continue;

    const int length =
        LoggingInternal::formatRecord(record, line, sizeof(line), droppedRecords.load(std::memory_order_relaxed),
                                      writeFailures.load(std::memory_order_relaxed));
    if (length <= 0 || static_cast<size_t>(length) >= sizeof(line)) {
      latchIncomplete(Logging::IncompleteReason::FormatFailed);
      continue;
    }

    if (Serial.write(reinterpret_cast<const uint8_t *>(line), static_cast<size_t>(length)) !=
        static_cast<size_t>(length)) {
      writeFailures.fetch_add(1, std::memory_order_relaxed);
      latchIncomplete(Logging::IncompleteReason::SenderWriteFailed);
    }
  }
}

template <typename Sample>
Logging::Result submitMeasurement(const Sample &sample, Logging::Profile expectedProfile,
                                  LoggingInternal::RecordKind kind,
                                  void (*copyPayload)(LoggingInternal::Payload &, const Sample &)) {
  const ProfileSnapshot profile = snapshotProfile();
  if (profile.profile != expectedProfile) return Logging::Result::Suppressed;
  if (isIncomplete()) return Logging::Result::Incomplete;

  LoggingInternal::Record record{};
  record.epoch = profile.epoch;
  record.profile = profile.profile;
  record.kind = kind;
  copyPayload(record.payload, sample);
  return enqueue(record);
}

void copyPayload(LoggingInternal::Payload &payload, const Logging::DiagnosticSample &sample) {
  payload.diagnostic = sample;
}
void copyPayload(LoggingInternal::Payload &payload, const Logging::TraceSample &sample) { payload.trace = sample; }
void copyPayload(LoggingInternal::Payload &payload, const Logging::ControlSample &sample) { payload.control = sample; }
void copyPayload(LoggingInternal::Payload &payload, const Logging::BalanceSample &sample) { payload.balance = sample; }
void copyPayload(LoggingInternal::Payload &payload, const Logging::DriveSample &sample) { payload.drive = sample; }
}  // namespace

namespace Logging {

bool begin() {
  bool expected = false;
  if (!beginAttempted.compare_exchange_strong(expected, true, std::memory_order_relaxed))
    return senderStarted.load(std::memory_order_relaxed);

  logQueue = xQueueCreateStatic(kQueueDepth, sizeof(LoggingInternal::Record), queueStorage, &queueControl);
  if (!logQueue) {
    latchIncomplete(IncompleteReason::SenderStartFailed);
    return false;
  }

  if (xTaskCreatePinnedToCore(senderTask, "logging", kSenderTaskStackDepth, nullptr, kSenderTaskPriority, nullptr, 0) !=
      pdTRUE) {
    latchIncomplete(IncompleteReason::SenderStartFailed);
    return false;
  }
  senderStarted.store(true, std::memory_order_relaxed);
  return true;
}

void setProfile(Profile nextProfile) {
  portENTER_CRITICAL(&profileMux);
  if (currentProfile != nextProfile) {
    currentProfile = nextProfile;
    ++currentEpoch;
  }
  portEXIT_CRITICAL(&profileMux);
}

Profile profile() { return snapshotProfile().profile; }

Result message(Level level, const char *tag, const char *format, ...) {
  if (format == nullptr || tag == nullptr) return Result::Invalid;

  const ProfileSnapshot profile = snapshotProfile();
  if (profile.profile != Profile::Idle) return Result::Suppressed;

  const size_t tagLength = strlen(tag);
  if (tagLength > kMessageTagSize) return Result::Truncated;

  LoggingInternal::MessageRecord messageRecord{};
  messageRecord.level = level;
  messageRecord.tagLength = static_cast<uint8_t>(tagLength);
  memcpy(messageRecord.tag, tag, tagLength);

  va_list args;
  va_start(args, format);
  const int length = vsnprintf(messageRecord.text, sizeof(messageRecord.text), format, args);
  va_end(args);
  if (length < 0) {
    latchIncomplete(IncompleteReason::FormatFailed);
    return Result::FormattingFailed;
  }
  if (static_cast<size_t>(length) >= sizeof(messageRecord.text)) return Result::Truncated;
  messageRecord.textLength = static_cast<uint8_t>(length);

  LoggingInternal::Record record{};
  record.epoch = profile.epoch;
  record.profile = profile.profile;
  record.kind = LoggingInternal::RecordKind::Message;
  record.payload.message = messageRecord;
  return enqueue(record);
}

Result submit(const DiagnosticSample &sample) {
  return submitMeasurement(sample, Profile::Diagnostic, LoggingInternal::RecordKind::Diagnostic, copyPayload);
}

Result submit(const TraceSample &sample) {
  return submitMeasurement(sample, Profile::Trace, LoggingInternal::RecordKind::Trace, copyPayload);
}

Result submit(const ControlSample &sample) {
  return submitMeasurement(sample, Profile::Control, LoggingInternal::RecordKind::Control, copyPayload);
}

Result submit(const BalanceSample &sample) {
  return submitMeasurement(sample, Profile::Balance, LoggingInternal::RecordKind::Balance, copyPayload);
}

Result submit(const DriveSample &sample) {
  return submitMeasurement(sample, Profile::Drive, LoggingInternal::RecordKind::Drive, copyPayload);
}

Result submit(const DebugSample &sample) {
  if (!isValidDebugSelector(sample.selector)) return Result::Invalid;
  if ((sample.selector == DebugSelector::K26 || sample.selector == DebugSelector::K40) &&
      sample.payload.touchPoint.state != 0 && sample.payload.touchPoint.state != 1)
    return Result::Invalid;

  const ProfileSnapshot profile = snapshotProfile();
  if (profile.profile != Profile::SelectedDebug) return Result::Suppressed;
  if (isIncomplete()) return Result::Incomplete;

  const uint32_t now = micros();
  if (selectedDebugSubmitted && static_cast<uint32_t>(now - lastSelectedDebugMicros) < kSelectedDebugMinIntervalUs)
    return Result::Paced;

  LoggingInternal::Record record{};
  record.epoch = profile.epoch;
  record.profile = profile.profile;
  record.kind = LoggingInternal::RecordKind::Debug;
  record.payload.debug = sample;
  const Result result = enqueue(record);
  if (result == Result::Accepted) {
    lastSelectedDebugMicros = now;
    selectedDebugSubmitted = true;
  }
  return result;
}

}  // namespace Logging
