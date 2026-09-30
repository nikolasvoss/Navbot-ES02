#include "TelemetryRecorder.h"

#include <string.h>

namespace telemetry {

void RecordRing::reset() {
  head_.store(0, std::memory_order_relaxed);
  tail_.store(0, std::memory_order_relaxed);
  highWater_.store(0, std::memory_order_relaxed);
}

bool RecordRing::push(const uint8_t record[kRecordSize]) {
  if (!storage_) return false;
  const uint32_t head = head_.load(std::memory_order_relaxed);
  const uint32_t tail = tail_.load(std::memory_order_acquire);
  if ((uint32_t)(head - tail) >= 512u) return false;
  memcpy(storage_ + (head & 511u) * kRecordSize, record, kRecordSize);
  const uint32_t next = head + 1;
  head_.store(next, std::memory_order_release);
  const uint32_t used = next - tail;
  uint32_t high = highWater_.load(std::memory_order_relaxed);
  while (used > high && !highWater_.compare_exchange_weak(high, used, std::memory_order_relaxed)) {}
  return true;
}

bool RecordRing::pop(uint8_t record[kRecordSize]) {
  if (!storage_) return false;
  const uint32_t tail = tail_.load(std::memory_order_relaxed);
  if (tail == head_.load(std::memory_order_acquire)) return false;
  memcpy(record, storage_ + (tail & 511u) * kRecordSize, kRecordSize);
  tail_.store(tail + 1, std::memory_order_release);
  return true;
}

uint32_t RecordRing::size() const {
  return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
}

bool canPrepare(const Session &session) {
  return session.state == SessionState::Idle || session.state == SessionState::Complete ||
         session.state == SessionState::Failed || session.state == SessionState::Unconfirmed;
}

bool acceptPrepare(Session &session, uint64_t recordingId, uint32_t durationSeconds) {
  if (!canPrepare(session) || recordingId == 0 || durationSeconds < 20 || durationSeconds > 40) return false;
  session = Session{};
  session.state = SessionState::Prepared;
  session.recordingId = recordingId;
  session.durationSeconds = durationSeconds;
  return true;
}

bool acceptReady(Session &session, uint64_t recordingId) {
  if (session.state != SessionState::Prepared || session.recordingId != recordingId) return false;
  session.clientReady = true;
  return true;
}

bool acceptStart(Session &session, uint64_t recordingId, uint64_t startUs) {
  if (session.state != SessionState::Prepared || !session.clientReady || session.recordingId != recordingId || startUs == 0) return false;
  session.state = SessionState::Recording;
  session.startUs = startUs;
  return true;
}

bool acceptStop(Session &session, uint64_t recordingId, uint64_t stopUs) {
  if (session.recordingId != recordingId) return false;
  if (session.state == SessionState::Draining || session.state == SessionState::AwaitAck ||
      session.state == SessionState::Complete) return true;
  if (session.state != SessionState::Recording && session.state != SessionState::Prepared) return false;
  session.state = session.state == SessionState::Prepared ? SessionState::Complete : SessionState::Draining;
  session.stopUs = stopUs;
  return true;
}

bool acceptDrainComplete(Session &session, uint64_t recordingId) {
  if (session.state != SessionState::Draining || session.recordingId != recordingId) return false;
  session.state = SessionState::AwaitAck;
  return true;
}

bool acceptAck(Session &session, uint64_t recordingId, uint32_t records, uint32_t recordsCrc,
               uint32_t expectedRecords, uint32_t expectedCrc) {
  if (session.recordingId != recordingId || records != expectedRecords || recordsCrc != expectedCrc) return false;
  if (session.state == SessionState::AwaitAck) session.state = SessionState::Complete;
  else if (session.state != SessionState::Complete) return false;
  return true;
}

void fail(Session &session, Failure reason) {
  if (session.state == SessionState::Complete || session.state == SessionState::Failed ||
      session.state == SessionState::Unconfirmed) return;
  session.failure = reason;
  session.state = SessionState::Failed;
}

bool durationElapsed(const Session &session, uint64_t sampleBoundaryUs) {
  return session.state == SessionState::Recording &&
         sampleBoundaryUs >= session.startUs + (uint64_t)session.durationSeconds * 1000000ULL;
}

}
