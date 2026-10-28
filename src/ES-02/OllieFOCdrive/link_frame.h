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

// A text line on the USB port is held back when it is longer than this.
constexpr size_t LINK_MAX_TEXT_LINE = 64;

class LinkRxSink {
 public:
  virtual ~LinkRxSink() {}
  // The payload of one frame whose COBS and CRC-16 are valid.
  virtual void OnPayload(const uint8_t* payload, size_t len) = 0;
  // A line of printable ASCII ending in '\n' (a '\r' ends a line too and is replaced by '\n').
  virtual void OnTextLine(const uint8_t* line, size_t len) { (void)line; (void)len; }
};

// Splits a received byte stream into frames at each 0x00 and checks COBS and CRC. A chunk that is too long, not valid
// COBS or has a wrong CRC is dropped and counted; the next 0x00 resynchronises. With passTextLines (the USB port, where
// the letter commands share the line) printable lines are also handed out, as they arrive. A binary frame can end
// such a line only by chance, and then only if everything between two 0x0A bytes is printable. The PC writes 0x00 before
// each frame, so a frame never follows typed text in one chunk.
class LinkRxDecoder {
 public:
  explicit LinkRxDecoder(bool passTextLines) : passTextLines_(passTextLines) {}

  void Feed(const uint8_t* data, size_t len, LinkRxSink& sink);
  uint32_t badFrames() const { return badFrames_; }

 private:
  void EndChunk(LinkRxSink& sink);
  void TrackLine(uint8_t byte, LinkRxSink& sink);

  bool passTextLines_;
  uint8_t chunk_[LINK_MAX_FRAME_LEN];
  size_t chunkLen_ = 0;
  bool chunkOverflow_ = false;
  uint8_t line_[LINK_MAX_TEXT_LINE];
  size_t lineLen_ = 0;
  bool lineSpoiled_ = false;  // holds a byte that is not text, or is too long
  uint32_t badFrames_ = 0;
};

#endif
