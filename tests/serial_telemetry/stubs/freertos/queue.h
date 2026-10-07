#pragma once
#include <stddef.h>
#include <stdint.h>
#include <mutex>
#include "FreeRTOS.h"

struct StaticQueue_t {
  std::mutex mutex;
  uint8_t *storage = nullptr;
  size_t itemSize = 0;
  size_t capacity = 0;
  size_t head = 0;
  size_t tail = 0;
  size_t count = 0;
};
using QueueHandle_t = StaticQueue_t *;

QueueHandle_t xQueueCreateStatic(size_t length, size_t itemSize, uint8_t *storage,
                                 StaticQueue_t *control);
BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t wait);
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t wait);
