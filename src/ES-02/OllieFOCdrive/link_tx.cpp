#include "link_tx.h"

#include <algorithm>
#include <cstring>

namespace {
constexpr size_t kHeaderLen = 2;
}

bool LinkItemQueue::Push(uint8_t tag, const uint8_t* data, size_t len) {
  if (len == 0 || len > LINK_MAX_ITEM_LEN || used_ + kHeaderLen + len > capacity_) {
    ++dropped_;
    return false;
  }
  const uint16_t header = static_cast<uint16_t>((tag << 10) | len);
  const uint8_t headerBytes[kHeaderLen] = {static_cast<uint8_t>(header & 0xFF), static_cast<uint8_t>(header >> 8)};
  size_t pos = (head_ + used_) % capacity_;
  for (size_t i = 0; i < kHeaderLen; ++i, pos = (pos + 1) % capacity_) storage_[pos] = headerBytes[i];
  for (size_t i = 0; i < len; ++i, pos = (pos + 1) % capacity_) storage_[pos] = data[i];
  used_ += kHeaderLen + len;
  return true;
}

bool LinkItemQueue::Pop(uint8_t& tag, uint8_t* out, size_t& len) {
  if (used_ == 0) return false;
  const uint16_t header = static_cast<uint16_t>(storage_[head_] | (storage_[(head_ + 1) % capacity_] << 8));
  tag = static_cast<uint8_t>(header >> 10);
  len = header & 0x3FF;
  size_t pos = (head_ + kHeaderLen) % capacity_;
  for (size_t i = 0; i < len; ++i, pos = (pos + 1) % capacity_) out[i] = storage_[pos];
  head_ = pos;
  used_ -= kHeaderLen + len;
  return true;
}

size_t LinkWriter::Pump(LinkItemQueue& queue, LinkByteSink& sink) {
  size_t written = 0;
  for (;;) {
    if (staged_ == 0) {
      size_t len = 0;
      // Popped one byte in, so the 0x00 prefix of a USB frame fits in front without a second copy.
      if (!queue.Pop(tag_, buffer_ + 1, len)) return written;
      if (tag_ == kLinkItemFrame && prefixFrames_) {
        buffer_[0] = 0x00;
        staged_ = len + 1;
      } else {
        memmove(buffer_, buffer_ + 1, len);
        staged_ = len;
      }
      sent_ = 0;
    }
    const int availableRaw = sink.AvailableForWrite();
    const size_t available = availableRaw > 0 ? static_cast<size_t>(availableRaw) : 0;
    const size_t remaining = staged_ - sent_;
    const size_t n = tag_ == kLinkItemFrame ? (available >= remaining ? remaining : 0) : std::min(available, remaining);
    if (n == 0) return written;
    sink.Write(buffer_ + sent_, n);
    written += n;
    sent_ += n;
    if (sent_ == staged_) staged_ = 0;
  }
}

bool LinkPortSend(LinkPort& port, navbot_link_v1_Message& msg, uint32_t tMs) {
  msg.seq = ++port.seq;
  msg.t_ms = tMs;
  uint8_t frame[LINK_MAX_FRAME_LEN];
  const size_t frameLen = LinkEncodeFrame(msg, frame, sizeof(frame));
  if (frameLen == 0) {
    port.queue.NoteDrop();
    return false;
  }
  return port.queue.Push(kLinkItemFrame, frame, frameLen);
}
