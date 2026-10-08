#pragma once

#include <stddef.h>
#include <stdint.h>

using UBaseType_t = unsigned int;
using BaseType_t = int;
using TickType_t = unsigned int;
using StackType_t = uint32_t;
struct StaticQueue_t;
using QueueHandle_t = StaticQueue_t *;
using TaskFunction_t = void (*)(void *);
#define pdTRUE 1
#define pdFALSE 0
#define portMAX_DELAY UINT32_MAX
