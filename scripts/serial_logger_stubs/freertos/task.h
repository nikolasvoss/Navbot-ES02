#pragma once

#include "FreeRTOS.h"

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t task, const char *name, uint32_t stackDepth,
                                  void *argument, UBaseType_t priority, void *handle, BaseType_t core);
void failNextTaskCreation();
