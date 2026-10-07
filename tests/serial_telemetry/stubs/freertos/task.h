#pragma once
#include "FreeRTOS.h"
using TaskFunction_t = void (*)(void *);
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t task, const char *name, uint32_t stackSize,
                                   void *argument, uint32_t priority, void *taskHandle,
                                   int32_t core);
void vTaskDelay(TickType_t ticks);
