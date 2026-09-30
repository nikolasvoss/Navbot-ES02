#pragma once
// WLAN services are opt-in. Copy wifi_tuning_build.example.h to the ignored
// local file wifi_tuning_build.h and enable tuning plus recording there.
#if __has_include("wifi_tuning_build.h")
#include "wifi_tuning_build.h"
#endif
#ifndef WIFI_TUNING_ENABLE
#define WIFI_TUNING_ENABLE 0
#endif
#ifndef WIFI_RECORDING_ENABLE
#define WIFI_RECORDING_ENABLE 0
#endif
#if WIFI_RECORDING_ENABLE && !WIFI_TUNING_ENABLE
#error "WIFI_RECORDING_ENABLE requires WIFI_TUNING_ENABLE and its station service"
#endif
