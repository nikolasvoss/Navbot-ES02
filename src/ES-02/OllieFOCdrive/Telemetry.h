#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace Telemetry {

enum class Channel : uint8_t {
  GyroX,
  GyroY,
  GyroZ,
  GyroFilteredX,
  GyroFilteredY,
  GyroFilteredZ,
  AccelX,
  AccelY,
  AccelZ,
  AccelFilteredX,
  AccelFilteredY,
  AccelFilteredZ,
  MahonyRoll,
  MahonyPitch,
  MahonyYaw,
  ComplementaryRoll,
  ComplementaryPitch,
  ComplementaryYaw,
  Temperature,
  BatteryRaw,
  BatteryFiltered,
  RcFrameAge,
  RcFailsafe,
  ImuReady,
  Count,
};

struct ChannelInfo {
  const char *name;
  const char *units;
};

constexpr size_t kChannelCount = static_cast<size_t>(Channel::Count);

struct Frame {
  uint64_t timestampUs = 0;
  float values[kChannelCount] = {};

  void set(Channel channel, float value) {
    values[static_cast<size_t>(channel)] = value;
  }
};

const ChannelInfo &channelInfo(Channel channel);
bool writeCsv(Print &sink, const Frame &frame);

}  // namespace Telemetry
