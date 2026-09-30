#pragma once

#include <stddef.h>
#include <stdint.h>

namespace telemetry {

constexpr size_t kRecordSize = 128;
constexpr size_t kRecordHeaderSize = 24;
constexpr size_t kRecordFloatCount = 26;
constexpr size_t kFrameHeaderSize = 32;
constexpr size_t kBatchRecordLimit = 32;
constexpr size_t kBatchPayloadLimit = kRecordSize * kBatchRecordLimit;
constexpr uint16_t kProtocolVersion = 1;

enum class FrameType : uint16_t { Meta = 1, Data = 2, End = 3 };

struct FrameHeader {
  FrameType type;
  uint64_t recordingId;
  uint32_t sequence;
  uint32_t payloadLength;
  uint32_t recordCount;
  uint32_t crc32;
};

struct Sample {
  uint64_t timestampUs;
  uint32_t sequence;
  uint32_t flags;
  uint32_t imuAgeUs;
  uint32_t controlDtUs;
  float values[kRecordFloatCount];
};

uint32_t crc32(const uint8_t *data, size_t length, uint32_t prior = 0);
bool encodeSample(const Sample &sample, uint8_t output[kRecordSize]);
bool decodeSample(const uint8_t input[kRecordSize], Sample &sample);
bool encodeFrameHeader(const FrameHeader &header, uint8_t output[kFrameHeaderSize]);
bool decodeFrameHeader(const uint8_t input[kFrameHeaderSize], FrameHeader &header);
bool validPayloadShape(const FrameHeader &header);

}
