#include "Arduino.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <new>
#include <thread>

namespace {
std::mutex serialMutex;
std::condition_variable serialChanged;
bool writesBlocked = false;
bool writeStarted = false;
}

HostSerial Serial;

size_t HostSerial::write(const uint8_t *, size_t length) {
  std::unique_lock<std::mutex> lock(serialMutex);
  writeStarted = true;
  serialChanged.notify_all();
  serialChanged.wait(lock, [] { return !writesBlocked; });
  return length;
}

void blockSerialWrites() {
  std::lock_guard<std::mutex> lock(serialMutex);
  writesBlocked = true;
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
  std::thread(task, argument).detach();
  return pdTRUE;
}
