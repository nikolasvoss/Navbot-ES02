#include "link_frame.h"

#include <cstring>
#include "src/navbot_link/pb_decode.h"
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

// Returns the decoded length, or SIZE_MAX when the input is not COBS. The encoder omits the trailing zero after its
// last block and so does this.
size_t CobsDecode(const uint8_t* in, size_t len, uint8_t* out) {
  size_t read = 0;
  size_t written = 0;
  while (read < len) {
    const uint8_t code = in[read++];
    if (code == 0 || read + code - 1 > len) return SIZE_MAX;
    memcpy(out + written, in + read, code - 1);
    read += code - 1;
    written += code - 1;
    if (code != 0xFF && read < len) out[written++] = 0;
  }
  return written;
}

// Decodes a COBS chunk into payload || CRC and checks the CRC. False when the COBS, the length or the CRC is wrong.
bool DecodeBody(const uint8_t* in, size_t len, uint8_t* body, size_t& payloadLen) {
  const size_t bodyLen = CobsDecode(in, len, body);
  // A payload is at least one byte, and the body holds payload and CRC.
  if (bodyLen == SIZE_MAX || bodyLen < 3 || bodyLen > LINK_MAX_PAYLOAD + 2) return false;
  payloadLen = bodyLen - 2;
  const uint16_t crc = static_cast<uint16_t>(body[payloadLen] | (body[payloadLen + 1] << 8));
  return Crc16CcittFalse(body, payloadLen) == crc;
}

// Length of the leading text (printable bytes, tab, CR, LF) up to its last line end. 0 when there is no line end.
size_t TypedLinesEnd(const uint8_t* chunk, size_t len) {
  size_t end = 0;
  for (size_t i = 0; i < len; ++i) {
    const uint8_t byte = chunk[i];
    const bool printable = (byte >= 0x20 && byte < 0x7F) || byte == '\t';
    if (byte == '\n' || byte == '\r')
      end = i + 1;
    else if (!printable)
      break;
  }
  return end;
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

bool LinkDecodeMessage(const uint8_t* payload, size_t len, navbot_link_v1_Message& msg) {
  msg = navbot_link_v1_Message_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(payload, len);
  return pb_decode(&stream, navbot_link_v1_Message_fields, &msg);
}

void LinkRxDecoder::Feed(const uint8_t* data, size_t len, LinkRxSink& sink) {
  for (size_t i = 0; i < len; ++i) {
    const uint8_t byte = data[i];
    if (byte == 0) {
      EndChunk(sink);
      continue;
    }
    if (chunkLen_ < sizeof(chunk_))
      chunk_[chunkLen_++] = byte;
    else
      chunkOverflow_ = true;
    if (passTextLines_) TrackLine(byte, sink);
  }
}

void LinkRxDecoder::EndChunk(LinkRxSink& sink) {
  uint8_t body[LINK_MAX_FRAME_LEN];
  const size_t chunkLen = chunkLen_;
  const bool overflow = chunkOverflow_;
  chunkLen_ = 0;
  chunkOverflow_ = false;
  lineLen_ = 0;
  lineSpoiled_ = false;
  if (chunkLen == 0 && !overflow) return;  // the 0x00 in front of a USB frame, or a repeated delimiter
  size_t payloadLen = 0;
  bool ok = !overflow && DecodeBody(chunk_, chunkLen, body, payloadLen);
  // Typed lines can sit in front of a frame with no 0x00 between them. The whole chunk is tried first, so a frame that
  // only contains a line end still decodes.
  if (!ok && !overflow && passTextLines_) {
    const size_t start = TypedLinesEnd(chunk_, chunkLen);
    ok = start > 0 && DecodeBody(chunk_ + start, chunkLen - start, body, payloadLen);
  }
  if (!ok) {
    ++badFrames_;
    return;
  }
  sink.OnPayload(body, payloadLen);
}

void LinkRxDecoder::TrackLine(uint8_t byte, LinkRxSink& sink) {
  if (byte == '\n' || byte == '\r') {
    if (lineLen_ > 0 && !lineSpoiled_) {
      line_[lineLen_++] = '\n';
      sink.OnTextLine(line_, lineLen_);
    }
    lineLen_ = 0;
    lineSpoiled_ = false;
    return;
  }
  const bool printable = (byte >= 0x20 && byte < 0x7F) || byte == '\t';
  if (!printable || lineLen_ + 1 >= sizeof(line_))
    lineSpoiled_ = true;
  else
    line_[lineLen_++] = byte;
}
