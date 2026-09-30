#include "../src/ES-02/OllieFOCdrive/TelemetryRecorder.h"
#include "../src/ES-02/OllieFOCdrive/TelemetryWire.h"

#include <assert.h>
#include <fstream>
#include <math.h>
#include <string>
#include <string.h>

static std::string fixture(const char *key) {
  std::ifstream input("tests/fixtures/telemetry-v1.hex");
  std::string line;
  while (std::getline(input, line)) {
    const std::string prefix = std::string(key) + "=";
    if (line.compare(0, prefix.size(), prefix) == 0) return line.substr(prefix.size());
  }
  return {};
}

static std::string hex(const uint8_t *bytes, size_t length) {
  static const char digits[] = "0123456789abcdef";
  std::string out;
  out.reserve(length * 2);
  for (size_t i = 0; i < length; ++i) { out.push_back(digits[bytes[i] >> 4]); out.push_back(digits[bytes[i] & 15]); }
  return out;
}

int main() {
  const uint8_t vector[] = "123456789";
  assert(telemetry::crc32(vector, 9) == 0xcbf43926u);

  telemetry::Sample sample{};
  sample.timestampUs = 0x0102030405060708ULL;
  sample.sequence = 0x11223344u;
  sample.flags = 0x55667788u;
  sample.imuAgeUs = 0x99aabbccu;
  sample.controlDtUs = 0xddeeff00u;
  sample.values[0] = 1.0f;
  sample.values[1] = NAN;
  uint8_t encoded[telemetry::kRecordSize];
  assert(telemetry::encodeSample(sample, encoded));
  assert(hex(encoded, sizeof(encoded)) == fixture("record"));
  const uint8_t prefix[] = {8,7,6,5,4,3,2,1, 0x44,0x33,0x22,0x11,
    0x88,0x77,0x66,0x55, 0xcc,0xbb,0xaa,0x99, 0,0xff,0xee,0xdd,
    0,0,0x80,0x3f};
  assert(memcmp(encoded, prefix, sizeof(prefix)) == 0);
  assert(encoded[28] == 0 && encoded[29] == 0 && encoded[30] == 0xc0 && encoded[31] == 0x7f);
  telemetry::Sample decoded{};
  assert(telemetry::decodeSample(encoded, decoded));
  assert(decoded.timestampUs == sample.timestampUs && decoded.sequence == sample.sequence);
  assert(isnan(decoded.values[1]));

  telemetry::FrameHeader header{telemetry::FrameType::Data, 7, 3,
                                telemetry::kRecordSize * 2, 2, 0x12345678};
  uint8_t frameHeader[telemetry::kFrameHeaderSize];
  assert(telemetry::encodeFrameHeader(header, frameHeader));
  telemetry::FrameHeader parsed{};
  assert(telemetry::decodeFrameHeader(frameHeader, parsed));
  assert(parsed.recordingId == 7 && parsed.sequence == 3 && parsed.recordCount == 2);
  frameHeader[20] = 1;
  assert(!telemetry::decodeFrameHeader(frameHeader, parsed));
  header.payloadLength = 9000;
  assert(!telemetry::encodeFrameHeader(header, frameHeader));

  telemetry::FrameHeader metaHeader{telemetry::FrameType::Meta, 7, 0, 2, 0, 0};
  assert(telemetry::encodeFrameHeader(metaHeader, frameHeader));
  const uint8_t metaPayload[] = {'{','}'};
  metaHeader.crc32 = telemetry::crc32(metaPayload, sizeof(metaPayload), telemetry::crc32(frameHeader, 28));
  assert(telemetry::encodeFrameHeader(metaHeader, frameHeader));
  std::string metaFrame = hex(frameHeader, sizeof(frameHeader)) + "7b7d";
  assert(metaFrame == fixture("meta_frame"));

  uint8_t storage[512 * telemetry::kRecordSize];
  telemetry::RecordRing ring(storage);
  assert(ring.size() == 0);
  for (unsigned i = 0; i < 512; ++i) {
    sample.sequence = i;
    assert(telemetry::encodeSample(sample, encoded));
    assert(ring.push(encoded));
  }
  assert(ring.size() == 512 && ring.highWater() == 512);
  assert(!ring.push(encoded));
  for (unsigned i = 0; i < 512; ++i) {
    assert(ring.pop(encoded));
    assert(telemetry::decodeSample(encoded, decoded));
    assert(decoded.sequence == i);
  }
  assert(!ring.pop(encoded) && ring.size() == 0);
  for (unsigned i = 0; i < 1024; ++i) {
    if (ring.size() == 512) {
      assert(ring.pop(encoded));
      assert(telemetry::decodeSample(encoded, decoded));
      assert(decoded.sequence == i - 512);
    }
    sample.sequence = i;
    assert(telemetry::encodeSample(sample, encoded));
    assert(ring.push(encoded));
  }
  for (unsigned i = 512; i < 1024; ++i) {
    assert(ring.pop(encoded));
    assert(telemetry::decodeSample(encoded, decoded));
    assert(decoded.sequence == i);
  }
  assert(!ring.pop(encoded));

  telemetry::Session session;
  assert(telemetry::acceptPrepare(session, 9, 30));
  assert(!telemetry::acceptReady(session, 8));
  assert(!telemetry::acceptStart(session, 9, 10));
  assert(telemetry::acceptReady(session, 9));
  assert(!telemetry::acceptStart(session, 8, 100));
  assert(telemetry::acceptStart(session, 9, 100));
  assert(!telemetry::acceptStart(session, 9, 101));
  assert(!telemetry::durationElapsed(session, 30'000'099));
  assert(telemetry::durationElapsed(session, 30'000'100));
  assert(telemetry::acceptStop(session, 9, 30'000'100));
  assert(telemetry::acceptStop(session, 9, 30'000'101));
  assert(!telemetry::acceptStop(session, 8, 30'000'102));
  assert(telemetry::acceptDrainComplete(session, 9));
  assert(!telemetry::acceptAck(session, 9, 30, 4, 30, 5));
  assert(telemetry::acceptAck(session, 9, 30, 5, 30, 5));
  assert(session.state == telemetry::SessionState::Complete);
  assert(telemetry::acceptAck(session, 9, 30, 5, 30, 5));
  assert(!telemetry::acceptAck(session, 9, 30, 4, 30, 5));
  assert(!telemetry::acceptPrepare(session, 10, 19));
  assert(telemetry::acceptPrepare(session, 10, 20));
  assert(telemetry::acceptStop(session, 10, 200));
  assert(session.state == telemetry::SessionState::Complete);
  telemetry::Session failed;
  assert(telemetry::acceptPrepare(failed, 11, 40));
  telemetry::fail(failed, telemetry::Failure::ConfigChanged);
  assert(failed.state == telemetry::SessionState::Failed && telemetry::canPrepare(failed));
  assert(telemetry::acceptPrepare(failed, 12, 20));
  assert(failed.failure == telemetry::Failure::None);
}
