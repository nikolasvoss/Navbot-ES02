#include "link_tx.h"

#include <algorithm>
#include <cstring>

namespace {
constexpr size_t kHeaderLen = 2;
}

void LinkItemQueue::CopyIn(size_t pos, const uint8_t* data, size_t len) {
  const size_t first = std::min(len, capacity_ - pos);
  memcpy(storage_ + pos, data, first);
  memcpy(storage_, data + first, len - first);
}

void LinkItemQueue::CopyOut(size_t pos, uint8_t* out, size_t len) const {
  const size_t first = std::min(len, capacity_ - pos);
  memcpy(out, storage_ + pos, first);
  memcpy(out + first, storage_, len - first);
}

bool LinkItemQueue::Push(uint8_t tag, const uint8_t* data, size_t len) {
  if (len == 0 || len > LINK_MAX_ITEM_LEN) return false;
  if (lock_) lock_->Lock();
  const bool fits = used_ + kHeaderLen + len <= capacity_;
  if (fits) {
    const uint16_t header = static_cast<uint16_t>((tag << 10) | len);
    const uint8_t headerBytes[kHeaderLen] = {static_cast<uint8_t>(header & 0xFF), static_cast<uint8_t>(header >> 8)};
    const size_t pos = (head_ + used_) % capacity_;
    CopyIn(pos, headerBytes, kHeaderLen);
    CopyIn((pos + kHeaderLen) % capacity_, data, len);
    used_ += kHeaderLen + len;
  }
  if (lock_) lock_->Unlock();
  return fits;
}

bool LinkItemQueue::PushOrDrop(uint8_t tag, const uint8_t* data, size_t len) {
  if (Push(tag, data, len)) return true;
  NoteDrop();
  return false;
}

void LinkItemQueue::NoteDrop() {
  if (lock_) lock_->Lock();
  ++dropped_;
  if (lock_) lock_->Unlock();
}

bool LinkItemQueue::Pop(uint8_t& tag, uint8_t* out, size_t& len) {
  if (lock_) lock_->Lock();
  const bool any = used_ > 0;
  if (any) {
    uint8_t headerBytes[kHeaderLen];
    CopyOut(head_, headerBytes, kHeaderLen);
    const uint16_t header = static_cast<uint16_t>(headerBytes[0] | (headerBytes[1] << 8));
    tag = static_cast<uint8_t>(header >> 10);
    len = header & 0x3FF;
    CopyOut((head_ + kHeaderLen) % capacity_, out, len);
    head_ = (head_ + kHeaderLen + len) % capacity_;
    used_ -= kHeaderLen + len;
  }
  if (lock_) lock_->Unlock();
  return any;
}

size_t LinkWriter::Pump(LinkItemQueue& queue, LinkByteSink& sink) {
  size_t written = 0;
  for (;;) {
    if (staged_ == 0) {
      // The frame is popped one byte in when the port needs a 0x00 in front, so no second copy is made.
      const size_t offset = prefixFrames_ ? 1 : 0;
      uint8_t tag = 0;
      size_t len = 0;
      if (!queue.Pop(tag, buffer_ + offset, len)) return written;
      if (prefixFrames_) buffer_[0] = 0x00;
      staged_ = offset + len;
    }
    const int availableRaw = sink.AvailableForWrite();
    const size_t available = availableRaw > 0 ? static_cast<size_t>(availableRaw) : 0;
    if (available < staged_) return written;
    sink.Write(buffer_, staged_);
    written += staged_;
    staged_ = 0;
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
  return port.queue.PushOrDrop(kLinkItemFrame, frame, frameLen);
}
