#include "Arduino.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <new>
#include <string>
#include <thread>

namespace {
std::mutex serialMutex;
std::condition_variable serialChanged;
bool writesBlocked = false;
bool writeStarted = false;
bool shortWrite = false;
bool taskCreationFails = false;
std::string serialBytes;
size_t serialLineCount = 0;
uint32_t serialBaud = 0;
std::chrono::steady_clock::time_point nextWriteAvailable{};
const auto clockOrigin = std::chrono::steady_clock::now();
}

HostSerial Serial;

uint32_t micros() {
  return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - clockOrigin).count());
}

void limitSerialBaud(uint32_t baud) {
  std::lock_guard<std::mutex> lock(serialMutex);
  serialBaud = baud;
  nextWriteAvailable = std::chrono::steady_clock::now();
}

size_t HostSerial::write(const uint8_t *data, size_t length) {
  std::unique_lock<std::mutex> lock(serialMutex);
  writeStarted = true;
  serialChanged.notify_all();
  serialChanged.wait(lock, [] { return !writesBlocked; });
  if (serialBaud) {
    const auto now = std::chrono::steady_clock::now();
    const auto start = std::max(now, nextWriteAvailable);
    const auto duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(static_cast<double>(length) * 10.0 / serialBaud));
    nextWriteAvailable = start + duration;
    const auto available = nextWriteAvailable;
    lock.unlock();
    std::this_thread::sleep_until(available);
    lock.lock();
  }
  const size_t written = shortWrite && length > 0 ? length - 1 : length;
  shortWrite = false;
  serialBytes.append(reinterpret_cast<const char *>(data), written);
  for (size_t i = 0; i < written; ++i) if (data[i] == '\n') ++serialLineCount;
  serialChanged.notify_all();
  return written;
}

void blockSerialWrites() {
  std::lock_guard<std::mutex> lock(serialMutex);
  writesBlocked = true;
  writeStarted = false;
}

bool waitForSerialWriteBlocked() {
  std::unique_lock<std::mutex> lock(serialMutex);
  return serialChanged.wait_for(lock, std::chrono::seconds(1), [] { return writeStarted; });
}

void releaseSerialWrites() {
  std::lock_guard<std::mutex> lock(serialMutex);
  writesBlocked = false;
  serialChanged.notify_all();
}


bool waitForSerialLines(size_t count) {
  std::unique_lock<std::mutex> lock(serialMutex);
  return serialChanged.wait_for(lock, std::chrono::seconds(2), [count] { return serialLineCount >= count; });
}

void shortNextSerialWrite() {
  std::lock_guard<std::mutex> lock(serialMutex);
  shortWrite = true;
}

std::string serialOutput() {
  std::lock_guard<std::mutex> lock(serialMutex);
  return serialBytes;
}

QueueHandle_t xQueueCreateStatic(UBaseType_t depth, UBaseType_t itemSize, uint8_t *, StaticQueue_t *control) {
  new (control) StaticQueue_t{};
  control->depth = depth;
  control->itemSize = itemSize;
  return control;
}

BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t waitTicks) {
  std::unique_lock<std::mutex> lock(queue->mutex);
  (void)waitTicks;
  if (queue->items.size() == queue->depth) return pdFALSE;
  std::vector<uint8_t> bytes(queue->itemSize);
  std::memcpy(bytes.data(), item, queue->itemSize);
  queue->items.push_back(std::move(bytes));
  queue->ready.notify_one();
  return pdTRUE;
}

BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t waitTicks) {
  std::unique_lock<std::mutex> lock(queue->mutex);
  if (waitTicks == portMAX_DELAY) queue->ready.wait(lock, [&] { return !queue->items.empty(); });
  else if (queue->items.empty()) return pdFALSE;
  std::memcpy(item, queue->items.front().data(), queue->itemSize);
  queue->items.pop_front();
  return pdTRUE;
}

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t task, const char *, uint32_t, void *argument,
                                  UBaseType_t, void *, BaseType_t) {
  if (taskCreationFails) {
    taskCreationFails = false;
    return pdFALSE;
  }
  std::thread(task, argument).detach();
  return pdTRUE;
}

void failNextTaskCreation() { taskCreationFails = true; }
