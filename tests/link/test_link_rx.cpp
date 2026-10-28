#include "link_frame.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

// The frames were produced by navbot_link.framing.encode_frame in Python from the payloads named in each comment.
namespace {

// Payload 0x07.
const uint8_t kShortFrame[] = {0x04, 0x07, 0x17, 0x91, 0x00};

// Payload 00 01 00 00 02: COBS must restore the zeros.
const uint8_t kZerosFrame[] = {0x01, 0x02, 0x01, 0x01, 0x04, 0x02, 0xFA, 0x47, 0x00};
const uint8_t kZerosPayload[] = {0x00, 0x01, 0x00, 0x00, 0x02};

// Payload byte i = ((7 * i + 1) % 255) + 1 for i < 252. With its CRC the body is 254 bytes without a zero, the one
// case where the last COBS block has no trailing code byte.
const uint8_t kFullBlockFrame[] = {
    0xFF, 0x02, 0x09, 0x10, 0x17, 0x1E, 0x25, 0x2C, 0x33, 0x3A, 0x41, 0x48, 0x4F, 0x56, 0x5D, 0x64,
    0x6B, 0x72, 0x79, 0x80, 0x87, 0x8E, 0x95, 0x9C, 0xA3, 0xAA, 0xB1, 0xB8, 0xBF, 0xC6, 0xCD, 0xD4,
    0xDB, 0xE2, 0xE9, 0xF0, 0xF7, 0xFE, 0x06, 0x0D, 0x14, 0x1B, 0x22, 0x29, 0x30, 0x37, 0x3E, 0x45,
    0x4C, 0x53, 0x5A, 0x61, 0x68, 0x6F, 0x76, 0x7D, 0x84, 0x8B, 0x92, 0x99, 0xA0, 0xA7, 0xAE, 0xB5,
    0xBC, 0xC3, 0xCA, 0xD1, 0xD8, 0xDF, 0xE6, 0xED, 0xF4, 0xFB, 0x03, 0x0A, 0x11, 0x18, 0x1F, 0x26,
    0x2D, 0x34, 0x3B, 0x42, 0x49, 0x50, 0x57, 0x5E, 0x65, 0x6C, 0x73, 0x7A, 0x81, 0x88, 0x8F, 0x96,
    0x9D, 0xA4, 0xAB, 0xB2, 0xB9, 0xC0, 0xC7, 0xCE, 0xD5, 0xDC, 0xE3, 0xEA, 0xF1, 0xF8, 0xFF, 0x07,
    0x0E, 0x15, 0x1C, 0x23, 0x2A, 0x31, 0x38, 0x3F, 0x46, 0x4D, 0x54, 0x5B, 0x62, 0x69, 0x70, 0x77,
    0x7E, 0x85, 0x8C, 0x93, 0x9A, 0xA1, 0xA8, 0xAF, 0xB6, 0xBD, 0xC4, 0xCB, 0xD2, 0xD9, 0xE0, 0xE7,
    0xEE, 0xF5, 0xFC, 0x04, 0x0B, 0x12, 0x19, 0x20, 0x27, 0x2E, 0x35, 0x3C, 0x43, 0x4A, 0x51, 0x58,
    0x5F, 0x66, 0x6D, 0x74, 0x7B, 0x82, 0x89, 0x90, 0x97, 0x9E, 0xA5, 0xAC, 0xB3, 0xBA, 0xC1, 0xC8,
    0xCF, 0xD6, 0xDD, 0xE4, 0xEB, 0xF2, 0xF9, 0x01, 0x08, 0x0F, 0x16, 0x1D, 0x24, 0x2B, 0x32, 0x39,
    0x40, 0x47, 0x4E, 0x55, 0x5C, 0x63, 0x6A, 0x71, 0x78, 0x7F, 0x86, 0x8D, 0x94, 0x9B, 0xA2, 0xA9,
    0xB0, 0xB7, 0xBE, 0xC5, 0xCC, 0xD3, 0xDA, 0xE1, 0xE8, 0xEF, 0xF6, 0xFD, 0x05, 0x0C, 0x13, 0x1A,
    0x21, 0x28, 0x2F, 0x36, 0x3D, 0x44, 0x4B, 0x52, 0x59, 0x60, 0x67, 0x6E, 0x75, 0x7C, 0x83, 0x8A,
    0x91, 0x98, 0x9F, 0xA6, 0xAD, 0xB4, 0xBB, 0xC2, 0xC9, 0xD0, 0xD7, 0xDE, 0xE5, 0x29, 0x2E, 0x00,
};

// GetCatalog{index: 3} with seq 5 and t_ms 6.
const uint8_t kGetCatalogFrame[] = {0x0C, 0x08, 0x05, 0x10, 0x06, 0xB2, 0x01, 0x02, 0x08, 0x03, 0x0A, 0x08, 0x00};

struct Collector : LinkRxSink {
  std::vector<std::vector<uint8_t>> payloads;
  void OnPayload(const uint8_t* payload, size_t len) override { payloads.emplace_back(payload, payload + len); }
};

bool failed = false;

void Check(bool condition, const char* what) {
  std::printf("%s: %s\n", what, condition ? "ok" : "FAIL");
  if (!condition) failed = true;
}

bool PayloadIs(const Collector& c, size_t index, const uint8_t* expected, size_t len) {
  return index < c.payloads.size() && c.payloads[index].size() == len &&
         std::memcmp(c.payloads[index].data(), expected, len) == 0;
}

void TestFrames() {
  LinkRxDecoder decoder;
  Collector c;
  decoder.Feed(kShortFrame, sizeof(kShortFrame), c);
  const uint8_t shortPayload[] = {0x07};
  Check(c.payloads.size() == 1 && PayloadIs(c, 0, shortPayload, 1), "one-byte payload");

  decoder.Feed(kZerosFrame, sizeof(kZerosFrame), c);
  Check(PayloadIs(c, 1, kZerosPayload, sizeof(kZerosPayload)), "COBS restores zeros");

  uint8_t fullPayload[252];
  for (size_t i = 0; i < sizeof(fullPayload); ++i) fullPayload[i] = static_cast<uint8_t>(((7 * i + 1) % 255) + 1);
  decoder.Feed(kFullBlockFrame, sizeof(kFullBlockFrame), c);
  Check(PayloadIs(c, 2, fullPayload, sizeof(fullPayload)), "254-byte body without a trailing code byte");
  Check(decoder.badFrames() == 0, "no bad frames so far");
}

void TestSplitAndBackToBack() {
  LinkRxDecoder decoder;
  Collector c;
  for (uint8_t byte : kGetCatalogFrame) decoder.Feed(&byte, 1, c);
  Check(c.payloads.size() == 1 && c.payloads[0].size() == 9, "frame fed byte by byte");

  std::vector<uint8_t> stream;
  stream.insert(stream.end(), kShortFrame, kShortFrame + sizeof(kShortFrame));
  stream.push_back(0x00);  // the USB writer's extra delimiter and a repeated one are empty chunks
  stream.insert(stream.end(), kGetCatalogFrame, kGetCatalogFrame + sizeof(kGetCatalogFrame));
  decoder.Feed(stream.data(), stream.size(), c);
  Check(c.payloads.size() == 3 && decoder.badFrames() == 0, "two frames and an empty chunk in one read");

  navbot_link_v1_Message msg;
  Check(LinkDecodeMessage(c.payloads[2].data(), c.payloads[2].size(), msg) &&
            msg.which_body == navbot_link_v1_Message_get_catalog_tag && msg.body.get_catalog.index == 3 &&
            msg.seq == 5 && msg.t_ms == 6,
        "GetCatalog payload parses");
}

void TestBadFrames() {
  LinkRxDecoder decoder;
  Collector c;
  uint8_t corrupted[sizeof(kGetCatalogFrame)];
  std::memcpy(corrupted, kGetCatalogFrame, sizeof(corrupted));
  corrupted[3] ^= 0x01;
  decoder.Feed(corrupted, sizeof(corrupted), c);
  Check(c.payloads.empty() && decoder.badFrames() == 1, "wrong CRC is dropped and counted");

  const uint8_t garbage[] = {'h', 'e', 'l', 'l', 'o', 0x00};
  decoder.Feed(garbage, sizeof(garbage), c);
  Check(c.payloads.empty() && decoder.badFrames() == 2, "text without a valid frame is counted");

  decoder.Feed(kShortFrame, sizeof(kShortFrame), c);
  Check(c.payloads.size() == 1, "the next frame decodes after the garbage");

  std::vector<uint8_t> tooLong(300, 0x41);
  tooLong.push_back(0x00);
  decoder.Feed(tooLong.data(), tooLong.size(), c);
  Check(c.payloads.size() == 1 && decoder.badFrames() == 3, "an overlong chunk is dropped");

  const uint8_t truncatedBlock[] = {0x09, 0x01, 0x02, 0x00};  // the code byte promises 8 bytes
  decoder.Feed(truncatedBlock, sizeof(truncatedBlock), c);
  Check(decoder.badFrames() == 4, "a block that runs past the end is dropped");

  const uint8_t tooShort[] = {0x03, 0x01, 0x02, 0x00};  // 2 body bytes: no payload
  decoder.Feed(tooShort, sizeof(tooShort), c);
  Check(decoder.badFrames() == 5 && c.payloads.size() == 1, "a frame without a payload is dropped");
}

// Reference encoder for the property test, written from the framing rules rather than from link_frame.cpp:
// COBS of payload || CRC-16/CCITT-FALSE (little-endian), then 0x00.
size_t ReferenceFrame(const uint8_t* payload, size_t len, uint8_t* out) {
  uint8_t body[LINK_MAX_PAYLOAD + 2];
  std::memcpy(body, payload, len);
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(payload[i] << 8);
    for (int bit = 0; bit < 8; ++bit) crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
  }
  body[len] = static_cast<uint8_t>(crc & 0xFF);
  body[len + 1] = static_cast<uint8_t>(crc >> 8);
  const size_t total = len + 2;

  size_t read = 0;
  size_t write = 0;
  while (true) {
    const size_t codeAt = write++;
    uint8_t code = 1;
    while (read < total && body[read] != 0 && code < 0xFF) {
      out[write++] = body[read++];
      ++code;
    }
    out[codeAt] = code;
    if (read >= total) break;
    if (code < 0xFF) ++read;  // the zero that ends the block
  }
  out[write++] = 0x00;
  return write;
}

// 20000 frames from a fixed seed, each after the PC's 0x00. Every one must decode to its payload with no bad frame.
void TestRandomFrames() {
  uint32_t state = 0x2545F491u;
  auto next = [&state]() {
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
  };

  LinkRxDecoder decoder;
  size_t mismatches = 0;
  for (int i = 0; i < 20000; ++i) {
    uint8_t payload[LINK_MAX_PAYLOAD];
    const size_t len = 1 + next() % LINK_MAX_PAYLOAD;
    for (size_t b = 0; b < len; ++b) payload[b] = static_cast<uint8_t>(next());

    uint8_t frame[LINK_MAX_FRAME_LEN];
    const size_t frameLen = ReferenceFrame(payload, len, frame);
    std::vector<uint8_t> stream = {0x00};
    stream.insert(stream.end(), frame, frame + frameLen);
    Collector c;
    decoder.Feed(stream.data(), stream.size(), c);
    if (c.payloads.size() != 1 || !PayloadIs(c, 0, payload, len)) ++mismatches;
  }
  Check(mismatches == 0 && decoder.badFrames() == 0, "20000 random frames decode to their payloads");
}

}  // namespace

int main() {
  TestFrames();
  TestSplitAndBackToBack();
  TestBadFrames();
  TestRandomFrames();
  return failed ? 1 : 0;
}
