#pragma once

#include <stddef.h>
#include <stdint.h>
#include <mutex>

using UBaseType_t = unsigned int;
using BaseType_t = int;
using TickType_t = unsigned int;
using StackType_t = uint32_t;
struct StaticQueue_t;
using QueueHandle_t = StaticQueue_t *;
using portMUX_TYPE = std::mutex;
#define portMUX_INITIALIZER_UNLOCKED {}
#define portENTER_CRITICAL(mux) (mux)->lock()
#define portEXIT_CRITICAL(mux) (mux)->unlock()
using TaskFunction_t = void (*)(void *);
#define pdTRUE 1
#define pdFALSE 0
#define portMAX_DELAY UINT32_MAX
