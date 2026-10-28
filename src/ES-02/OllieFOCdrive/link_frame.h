#ifndef LINK_FRAME_H
#define LINK_FRAME_H

#include <cstddef>
#include <cstdint>
#include "src/navbot_link/navbot_link.pb.h"

// Largest serialized Message (payload) the link carries.
constexpr size_t LINK_MAX_PAYLOAD = 252;
// payload + CRC-16, COBS code byte, 0x00 delimiter.
constexpr size_t LINK_MAX_FRAME_LEN = LINK_MAX_PAYLOAD + 2 + 1 + 1;

// Writes one frame COBS(payload || crc16_le(payload)) 0x00 into out.
// Returns the frame length, or 0 if encoding fails or the frame does not fit in capacity.
size_t LinkEncodeFrame(const navbot_link_v1_Message& msg, uint8_t* out, size_t capacity);

// Parses a CRC-checked payload into msg. Returns false for a payload nanopb rejects.
bool LinkDecodeMessage(const uint8_t* payload, size_t len, navbot_link_v1_Message& msg);

class LinkRxSink {
 public:
  virtual ~LinkRxSink() {}
  // The payload of one frame whose COBS and CRC-16 are valid.
  virtual void OnPayload(const uint8_t* payload, size_t len) = 0;
};

// Splits a received byte stream into frames at each 0x00 and checks COBS and CRC. A chunk that is too long, not valid
// COBS or has a wrong CRC is dropped and counted; the next 0x00 resynchronises. The PC writes 0x00 before each frame on
// the USB port, so a frame never follows other bytes in one chunk.
class LinkRxDecoder {
 public:
  void Feed(const uint8_t* data, size_t len, LinkRxSink& sink);
  uint32_t badFrames() const { return badFrames_; }

 private:
  void EndChunk(LinkRxSink& sink);

  uint8_t chunk_[LINK_MAX_FRAME_LEN];
  size_t chunkLen_ = 0;
  bool chunkOverflow_ = false;
  uint32_t badFrames_ = 0;
};

#endif
