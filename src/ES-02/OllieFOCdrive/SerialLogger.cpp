#include "Arduino.h"
#include "SerialLogger.h"

#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

namespace {
constexpr UBaseType_t kQueueDepth = 16;
constexpr size_t kRowBufferSize = 384;
StaticQueue_t queueControl;
uint8_t queueStorage[kQueueDepth * sizeof(SerialLogRecord)];
QueueHandle_t logQueue;
std::atomic<uint32_t> droppedRecords{0};
std::atomic<uint32_t> uartWriteFailures{0};
std::atomic<bool> traceModeSelected{false};

void senderTask(void *) {
  SerialLogRecord row;
  char buffer[kRowBufferSize];
  for (;;) {
    if (xQueueReceive(logQueue, &row, portMAX_DELAY) != pdTRUE) continue;
    const int length = formatSerialLogRecord(row, buffer, sizeof(buffer));
    if (length <= 0 || (size_t)length >= sizeof(buffer) ||
        Serial.write((const uint8_t *)buffer, (size_t)length) != (size_t)length) {
      uartWriteFailures.fetch_add(1, std::memory_order_relaxed);
    }
  }
}
}

void SerialLoggerSetTraceMode(int mode) {
  traceModeSelected.store(mode >= 55 && mode <= 58, std::memory_order_relaxed);
}

bool SerialLoggerTraceModeSelected() {
  return traceModeSelected.load(std::memory_order_relaxed);
}

void SerialLoggerBegin() {
  if (logQueue) return;
  logQueue = xQueueCreateStatic(kQueueDepth, sizeof(SerialLogRecord), queueStorage, &queueControl);
  if (logQueue) xTaskCreatePinnedToCore(senderTask, "serial_log", 4096, nullptr, 1, nullptr, 0);
}

bool SerialLoggerSubmit(SerialLogRecord row) {
  if (!logQueue) return false;
  row.dropped = droppedRecords.load(std::memory_order_relaxed);
  row.writeFailures = uartWriteFailures.load(std::memory_order_relaxed);
  if (xQueueSend(logQueue, &row, 0) == pdTRUE) return true;
  droppedRecords.fetch_add(1, std::memory_order_relaxed);
  return false;
}
