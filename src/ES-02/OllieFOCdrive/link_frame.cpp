#include "link_frame.h"

#include <cstring>
#include "src/navbot_link/pb_encode.h"

namespace {

uint16_t Crc16CcittFalse(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

// Output never exceeds len + 1 bytes for len <= 254, so the caller's buffer of 255 bytes is enough.
size_t CobsEncode(const uint8_t* in, size_t len, uint8_t* out) {
  size_t codeIndex = 0;
  size_t outLen = 1;
  uint8_t code = 1;
  for (size_t i = 0; i < len; ++i) {
    if (in[i] == 0) {
      out[codeIndex] = code;
      codeIndex = outLen++;
      code = 1;
      continue;
    }
    out[outLen++] = in[i];
    if (++code == 0xFF) {
      out[codeIndex] = code;
      if (i == len - 1) return outLen;  // A full 254-byte block at the end needs no trailing code byte.
      codeIndex = outLen++;
      code = 1;
    }
  }
  out[codeIndex] = code;
  return outLen;
}

}  // namespace

size_t LinkEncodeFrame(const navbot_link_v1_Message& msg, uint8_t* out, size_t capacity) {
  uint8_t body[LINK_MAX_PAYLOAD + 2];
  pb_ostream_t stream = pb_ostream_from_buffer(body, LINK_MAX_PAYLOAD);
  if (!pb_encode(&stream, navbot_link_v1_Message_fields, &msg)) return 0;

  const size_t payloadLen = stream.bytes_written;
  const uint16_t crc = Crc16CcittFalse(body, payloadLen);
  body[payloadLen] = static_cast<uint8_t>(crc & 0xFF);
  body[payloadLen + 1] = static_cast<uint8_t>(crc >> 8);

  uint8_t frame[LINK_MAX_FRAME_LEN];
  const size_t cobsLen = CobsEncode(body, payloadLen + 2, frame);
  frame[cobsLen] = 0x00;
  const size_t frameLen = cobsLen + 1;
  if (frameLen > capacity) return 0;
  memcpy(out, frame, frameLen);
  return frameLen;
}
