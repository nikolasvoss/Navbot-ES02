#ifndef LINK_TX_H
#define LINK_TX_H

#include <cstddef>
#include <cstdint>
#include "link_frame.h"

// Pure transmit-path logic for one port: no Arduino or FreeRTOS, so tests/link host-tests it. The firmware wraps the
// queue in a lock and runs LinkWriter::Pump from one writer task per port (link_uart.cpp).

// Largest item the queue carries: a Logging.cpp text/CSV line is up to 576 bytes.
constexpr size_t LINK_MAX_ITEM_LEN = 640;

// Optional mutual exclusion around the queue; the firmware passes a spinlock, tests pass none.
class LinkLock {
 public:
  virtual ~LinkLock() {}
  virtual void Lock() = 0;
  virtual void Unlock() = 0;
};

// FIFO of whole items in a byte ring. An item is stored behind a 2-byte header (tag in the top 6 bits, length in the
// low 10), so a reader gets complete items in push order and never part of one. Safe for several producers and one
// consumer when given a lock.
class LinkItemQueue {
 public:
  LinkItemQueue(uint8_t* storage, size_t capacity, LinkLock* lock = nullptr)
      : storage_(storage), capacity_(capacity), lock_(lock) {}

  // Returns false when the item does not fit. Never blocks, never counts.
  bool Push(uint8_t tag, const uint8_t* data, size_t len);
  // Push, and count a drop when it does not fit.
  bool PushOrDrop(uint8_t tag, const uint8_t* data, size_t len);
  bool Pop(uint8_t& tag, uint8_t* out, size_t& len);
  void NoteDrop();
  uint32_t dropped() const { return dropped_; }

 private:
  void CopyIn(size_t pos, const uint8_t* data, size_t len);
  void CopyOut(size_t pos, uint8_t* out, size_t len) const;

  uint8_t* storage_;
  size_t capacity_;
  LinkLock* lock_;
  size_t head_ = 0;  // next byte to read
  size_t used_ = 0;
  uint32_t dropped_ = 0;
};

// Requests the control loop answers per tick. Each one runs inside the loop (a tick is about 1.4 ms), so a burst would
// stretch that tick. The rest wait in the queue for the next tick. A catalog read asks for one entry at a time, so the
// cap does not slow it.
constexpr size_t LINK_REQUESTS_PER_TICK = 2;

// Hands at most maxItems queued items to handle(port, payload, len), oldest first, copying each into scratch (at least
// LINK_MAX_ITEM_LEN bytes). Returns how many it handed out.
template <typename Handle>
size_t LinkServeQueued(LinkItemQueue& queue, size_t maxItems, uint8_t* scratch, Handle&& handle) {
  size_t served = 0;
  uint8_t port = 0;
  size_t len = 0;
  while (served < maxItems && queue.Pop(port, scratch, len)) {
    handle(port, scratch, len);
    ++served;
  }
  return served;
}

enum LinkItemTag : uint8_t {
  kLinkItemFrame = 0,  // one encoded frame, sent whole or not at all
  kLinkItemRaw = 1,    // text/CSV bytes, may go out in pieces
};

class LinkByteSink {
 public:
  virtual ~LinkByteSink() {}
  virtual int AvailableForWrite() = 0;
  virtual void Write(const uint8_t* data, size_t len) = 0;
};

// Moves items from a queue to a sink. A frame is written only when the sink has room for all of it, so a frame is never
// split across writes and a later item never starts before it. Raw bytes go out as far as the sink allows. With
// prefixFrames each frame gets a 0x00 in front (the USB port, where text shares the line).
class LinkWriter {
 public:
  explicit LinkWriter(bool prefixFrames) : prefixFrames_(prefixFrames) {}

  // Writes until the queue is empty or the sink is full. Returns the bytes written.
  size_t Pump(LinkItemQueue& queue, LinkByteSink& sink);

 private:
  bool prefixFrames_;
  uint8_t buffer_[LINK_MAX_ITEM_LEN + 1];
  uint8_t tag_ = kLinkItemFrame;
  size_t staged_ = 0;  // bytes in buffer_ for the current item; 0 = none
  size_t sent_ = 0;
};

// Frame queue plus its own sequence counter. Each port owns one, so a gap in a port's seq means a lost frame on that port.
struct LinkPort {
  LinkPort(uint8_t* storage, size_t capacity, bool prefixFrames, LinkLock* lock = nullptr)
      : queue(storage, capacity, lock), writer(prefixFrames) {}
  LinkItemQueue queue;
  LinkWriter writer;
  uint32_t seq = 0;
};

// Stamps seq and t_ms, encodes and queues the frame. Returns false when the frame was dropped. The seq is spent even
// then, so the receiver sees the gap. Call it from one task per port: seq order is the push order.
bool LinkPortSend(LinkPort& port, navbot_link_v1_Message& msg, uint32_t tMs);

#endif
