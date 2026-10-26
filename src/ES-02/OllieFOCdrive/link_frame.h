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

#endif
