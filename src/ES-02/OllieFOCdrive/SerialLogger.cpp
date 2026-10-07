#include "Arduino.h"
#include <esp_attr.h>
#include "SerialLogger.h"

#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

namespace {
constexpr UBaseType_t kQueueDepth = kSerialLoggerQueueDepth;
constexpr size_t kRowBufferSize = 384;
StaticQueue_t queueControl DRAM_ATTR;
uint8_t queueStorage[kQueueDepth * sizeof(SerialLogRecord)] DRAM_ATTR;
QueueHandle_t logQueue;
std::atomic<uint32_t> droppedRecords{0};
std::atomic<uint32_t> uartWriteFailures{0};
std::atomic<bool> selectedModeActive{false};
std::atomic<uint8_t> incompleteReason{static_cast<uint8_t>(SerialLoggerIncompleteReason::None)};
std::atomic<uint8_t> rejectedRecords{0};

void latchIncomplete(SerialLoggerIncompleteReason reason) {
  uint8_t expected = static_cast<uint8_t>(SerialLoggerIncompleteReason::None);
  incompleteReason.compare_exchange_strong(expected, static_cast<uint8_t>(reason),
                                          std::memory_order_relaxed);
}

void countRejected() {
  const uint8_t count = rejectedRecords.load(std::memory_order_relaxed);
  if (count < UINT8_MAX) rejectedRecords.store(static_cast<uint8_t>(count + 1),
                                               std::memory_order_relaxed);
}

void senderTask(void *) {
  SerialLogRecord row;
  char buffer[kRowBufferSize];
  for (;;) {
    if (xQueueReceive(logQueue, &row, portMAX_DELAY) != pdTRUE) continue;
    const int length = formatSerialLogRecord(row, buffer, sizeof(buffer));
    if (length <= 0 || (size_t)length >= sizeof(buffer) ||
        Serial.write((const uint8_t *)buffer, (size_t)length) != (size_t)length) {
      uartWriteFailures.fetch_add(1, std::memory_order_relaxed);
      latchIncomplete(SerialLoggerIncompleteReason::SenderWriteFailed);
    }
  }
}
}

void SerialLoggerSetSelectedMode(int mode) {
  selectedModeActive.store((mode >= 1 && mode <= 45) || (mode >= 55 && mode <= 58), std::memory_order_relaxed);
}

bool SerialLoggerSelectedMode() {
  return selectedModeActive.load(std::memory_order_relaxed);
}

void SerialLoggerBegin() {
  if (logQueue) return;
  logQueue = xQueueCreateStatic(kQueueDepth, sizeof(SerialLogRecord), queueStorage, &queueControl);
  if (!logQueue) {
    latchIncomplete(SerialLoggerIncompleteReason::SenderStartFailed);
    return;
  }
  if (xTaskCreatePinnedToCore(senderTask, "serial_log", 4096, nullptr, 1, nullptr, 0) != pdTRUE)
    latchIncomplete(SerialLoggerIncompleteReason::SenderStartFailed);
}

bool SerialLoggerSubmit(SerialLogRecord row) {
  if (incompleteReason.load(std::memory_order_relaxed) !=
      static_cast<uint8_t>(SerialLoggerIncompleteReason::None)) {
    countRejected();
    return false;
  }
  if (!logQueue) {
    latchIncomplete(SerialLoggerIncompleteReason::SenderStartFailed);
    countRejected();
    return false;
  }
  row.dropped = droppedRecords.load(std::memory_order_relaxed);
  row.writeFailures = uartWriteFailures.load(std::memory_order_relaxed);
  if (xQueueSend(logQueue, &row, 0) == pdTRUE) return true;
  droppedRecords.fetch_add(1, std::memory_order_relaxed);
  latchIncomplete(SerialLoggerIncompleteReason::QueueFull);
  countRejected();
  return false;
}

SerialLoggerIncompleteReason SerialLoggerIncomplete() {
  return static_cast<SerialLoggerIncompleteReason>(incompleteReason.load(std::memory_order_relaxed));
}

uint8_t SerialLoggerRejectedCount() {
  return rejectedRecords.load(std::memory_order_relaxed);
}
