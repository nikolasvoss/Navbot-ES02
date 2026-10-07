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

constexpr size_t kQueueCapacity = 32;
constexpr size_t kSelectedFloatCount = 26;
constexpr size_t kSelectedIntegerCount = 10;

struct SelectedDebug {
  int32_t selector = 0;
  uint32_t timestampMs = 0;
  float values[kSelectedFloatCount] = {};
  int32_t integers[kSelectedIntegerCount] = {};
};

enum class RecordKind : uint8_t {
  Diagnostic,
  SelectedDebug,
};

struct Record {
  RecordKind kind = RecordKind::Diagnostic;
  Frame diagnostic;
  SelectedDebug selected;
};

enum class IncompleteReason : uint8_t {
  None,
  BufferFull,
  SenderStartFailed,
  SenderWriteFailed,
};

const ChannelInfo &channelInfo(Channel channel);
bool enqueueDiagnostic(const Frame &frame);
bool enqueueSelected(const SelectedDebug &selected);
bool sendOne(Print &sink);
bool startSender();
IncompleteReason incompleteReason();
uint8_t rejectedCount();

}  // namespace Telemetry
