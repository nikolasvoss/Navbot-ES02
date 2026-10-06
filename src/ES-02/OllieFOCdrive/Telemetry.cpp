#include "Telemetry.h"

#include <stdio.h>

namespace Telemetry {
namespace {

constexpr ChannelInfo kChannels[] = {
    {"Unfiltered gyro X", "rad/s"},
    {"Unfiltered gyro Y", "rad/s"},
    {"Unfiltered gyro Z", "rad/s"},
    {"Filtered gyro X", "rad/s"},
    {"Filtered gyro Y", "rad/s"},
    {"Filtered gyro Z", "rad/s"},
    {"Unfiltered accel X", "g"},
    {"Unfiltered accel Y", "g"},
    {"Unfiltered accel Z", "g"},
    {"Filtered accel X", "g"},
    {"Filtered accel Y", "g"},
    {"Filtered accel Z", "g"},
    {"Mahony roll", "deg"},
    {"Mahony pitch", "deg"},
    {"Mahony yaw", "deg"},
    {"Complementary roll", "deg"},
    {"Complementary pitch", "deg"},
    {"Complementary yaw", "deg"},
    {"IMU temperature", "°C"},
    {"Battery raw", "V"},
    {"Battery filtered", "V"},
    {"Receiver frame age", "ms"},
    {"Receiver failsafe", ""},
    {"IMU ready", ""},
};

constexpr size_t kFrameBufferSize = 512;
static_assert(sizeof(kChannels) / sizeof(kChannels[0]) == kChannelCount,
              "Telemetry metadata must cover every channel");

bool append(char *buffer, size_t capacity, size_t &length, const char *format, double value) {
  if (length >= capacity) return false;
  const int written = snprintf(buffer + length, capacity - length, format, value);
  if (written < 0 || static_cast<size_t>(written) >= capacity - length) return false;
  length += static_cast<size_t>(written);
  return true;
}

}  // namespace

const ChannelInfo &channelInfo(Channel channel) {
  return kChannels[static_cast<size_t>(channel)];
}

bool writeCsv(Print &sink, const Frame &frame) {
  char buffer[kFrameBufferSize];
  size_t length = 0;

  for (size_t i = 0; i < kChannelCount; ++i) {
    if (i != 0) buffer[length++] = ',';
    if (!append(buffer, sizeof(buffer), length, "%.6f", frame.values[i])) return false;
  }

  buffer[length++] = ',';
  if (!append(buffer, sizeof(buffer), length, "%.6f",
              static_cast<double>(frame.timestampUs) / 1000000.0)) return false;
  buffer[length++] = '\n';
  return sink.write(reinterpret_cast<const uint8_t *>(buffer), length) == length;
}

}  // namespace Telemetry
