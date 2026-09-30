#include "TelemetryWire.h"

#include <math.h>
#include <string.h>

namespace telemetry {
namespace {

void put16(uint8_t *out, uint16_t value) {
  out[0] = (uint8_t)value;
  out[1] = (uint8_t)(value >> 8);
}

void put32(uint8_t *out, uint32_t value) {
  for (unsigned i = 0; i < 4; ++i) out[i] = (uint8_t)(value >> (8 * i));
}

void put64(uint8_t *out, uint64_t value) {
  for (unsigned i = 0; i < 8; ++i) out[i] = (uint8_t)(value >> (8 * i));
}

uint16_t get16(const uint8_t *in) {
  return (uint16_t)in[0] | (uint16_t)((uint16_t)in[1] << 8);
}

uint32_t get32(const uint8_t *in) {
  uint32_t value = 0;
  for (unsigned i = 0; i < 4; ++i) value |= (uint32_t)in[i] << (8 * i);
  return value;
}

uint64_t get64(const uint8_t *in) {
  uint64_t value = 0;
  for (unsigned i = 0; i < 8; ++i) value |= (uint64_t)in[i] << (8 * i);
  return value;
}

uint32_t floatBits(float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  return bits;
}

float bitsFloat(uint32_t bits) {
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

}

uint32_t crc32(const uint8_t *data, size_t length, uint32_t prior) {
  uint32_t crc = prior ^ 0xffffffffu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (unsigned bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xedb88320u & (uint32_t)-(int32_t)(crc & 1u));
  }
  return crc ^ 0xffffffffu;
}

bool encodeSample(const Sample &sample, uint8_t output[kRecordSize]) {
  if (sizeof(float) != 4) return false;
  put64(output, sample.timestampUs);
  put32(output + 8, sample.sequence);
  put32(output + 12, sample.flags);
  put32(output + 16, sample.imuAgeUs);
  put32(output + 20, sample.controlDtUs);
  for (size_t i = 0; i < kRecordFloatCount; ++i)
    put32(output + kRecordHeaderSize + 4 * i, floatBits(sample.values[i]));
  return true;
}

bool decodeSample(const uint8_t input[kRecordSize], Sample &sample) {
  if (sizeof(float) != 4) return false;
  sample.timestampUs = get64(input);
  sample.sequence = get32(input + 8);
  sample.flags = get32(input + 12);
  sample.imuAgeUs = get32(input + 16);
  sample.controlDtUs = get32(input + 20);
  for (size_t i = 0; i < kRecordFloatCount; ++i)
    sample.values[i] = bitsFloat(get32(input + kRecordHeaderSize + 4 * i));
  return true;
}

bool encodeFrameHeader(const FrameHeader &header, uint8_t output[kFrameHeaderSize]) {
  const uint16_t type = (uint16_t)header.type;
  if (type < 1 || type > 3) return false;
  memcpy(output, "NBL1", 4);
  put16(output + 4, kProtocolVersion);
  put16(output + 6, type);
  put64(output + 8, header.recordingId);
  put32(output + 16, header.sequence);
  put32(output + 20, header.payloadLength);
  put32(output + 24, header.recordCount);
  put32(output + 28, header.crc32);
  return validPayloadShape(header);
}

bool decodeFrameHeader(const uint8_t input[kFrameHeaderSize], FrameHeader &header) {
  if (memcmp(input, "NBL1", 4) != 0 || get16(input + 4) != kProtocolVersion) return false;
  const uint16_t type = get16(input + 6);
  if (type < 1 || type > 3) return false;
  header.type = (FrameType)type;
  header.recordingId = get64(input + 8);
  header.sequence = get32(input + 16);
  header.payloadLength = get32(input + 20);
  header.recordCount = get32(input + 24);
  header.crc32 = get32(input + 28);
  return validPayloadShape(header);
}

bool validPayloadShape(const FrameHeader &header) {
  switch (header.type) {
    case FrameType::Meta: return header.payloadLength <= 8192 && header.recordCount == 0;
    case FrameType::Data:
      return header.recordCount >= 1 && header.recordCount <= kBatchRecordLimit &&
             header.payloadLength == header.recordCount * kRecordSize;
    case FrameType::End: return header.payloadLength <= 4096 && header.recordCount == 0;
  }
  return false;
}

}
