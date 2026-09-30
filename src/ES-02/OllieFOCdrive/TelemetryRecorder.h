#pragma once

#include "TelemetryWire.h"

#include <atomic>
#include <stddef.h>
#include <stdint.h>

namespace telemetry {

enum class SessionState : uint8_t {
  Idle,
  Prepared,
  Recording,
  Draining,
  AwaitAck,
  Complete,
  Failed,
  Unconfirmed,
};

enum class Failure : uint8_t {
  None,
  BufferFull,
  ClientDisconnected,
  ConfigChanged,
  SenderError,
  PrepareExpired,
  DrainExpired,
};

class RecordRing {
 public:
  explicit RecordRing(uint8_t *storage = nullptr) : storage_(storage) {}
  void bind(uint8_t *storage) { storage_ = storage; reset(); }
  void reset();
  bool push(const uint8_t record[kRecordSize]);
  bool pop(uint8_t record[kRecordSize]);
  uint32_t size() const;
  uint32_t highWater() const { return highWater_.load(std::memory_order_relaxed); }
  bool valid() const { return storage_ != nullptr; }

 private:
  uint8_t *storage_;
  std::atomic<uint32_t> head_{0};
  std::atomic<uint32_t> tail_{0};
  std::atomic<uint32_t> highWater_{0};
};

struct Session {
  SessionState state = SessionState::Idle;
  Failure failure = Failure::None;
  bool clientReady = false;
  uint64_t recordingId = 0;
  uint32_t durationSeconds = 0;
  uint64_t startUs = 0;
  uint64_t stopUs = 0;
  uint32_t generated = 0;
  uint32_t queued = 0;
  uint32_t sent = 0;
};

bool canPrepare(const Session &session);
bool acceptPrepare(Session &session, uint64_t recordingId, uint32_t durationSeconds);
bool acceptReady(Session &session, uint64_t recordingId);
bool acceptStart(Session &session, uint64_t recordingId, uint64_t startUs);
bool acceptStop(Session &session, uint64_t recordingId, uint64_t stopUs);
bool acceptDrainComplete(Session &session, uint64_t recordingId);
bool acceptAck(Session &session, uint64_t recordingId, uint32_t records, uint32_t recordsCrc,
               uint32_t expectedRecords, uint32_t expectedCrc);
void fail(Session &session, Failure reason);
bool durationElapsed(const Session &session, uint64_t sampleBoundaryUs);

}
