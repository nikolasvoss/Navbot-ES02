#pragma once

#include "FreeRTOS.h"
#include <condition_variable>
#include <deque>
#include <mutex>
#include <vector>

struct StaticQueue_t {
  size_t depth;
  size_t itemSize;
  std::deque<std::vector<uint8_t>> items;
  std::mutex mutex;
  std::condition_variable ready;
};

QueueHandle_t xQueueCreateStatic(UBaseType_t depth, UBaseType_t itemSize, uint8_t *storage, StaticQueue_t *control);
BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t waitTicks);
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t waitTicks);
