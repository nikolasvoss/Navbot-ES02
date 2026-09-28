#pragma once
// WLAN tuning is opt-in. Copy wifi_tuning_build.example.h to the ignored
// local file wifi_tuning_build.h and set WIFI_TUNING_ENABLE to 1 there.
#if __has_include("wifi_tuning_build.h")
#include "wifi_tuning_build.h"
#endif
#ifndef WIFI_TUNING_ENABLE
#define WIFI_TUNING_ENABLE 0
#endif
