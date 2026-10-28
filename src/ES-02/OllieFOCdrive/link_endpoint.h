#ifndef LINK_ENDPOINT_H
#define LINK_ENDPOINT_H

#include <stddef.h>
#include <stdint.h>

#include "link_frame.h"
#include "link_subscriptions.h"
#include "signal_table.h"

// The robot's side of the catalog, subscription and parameter messages (docs/integration/software/ros2-uart-protocol.md).
// Pure logic: requests come in as decoded payloads, replies and rows leave through a sink that stamps seq and t_ms and
// queues the frame for the port. The control loop is the only caller.

class LinkMessageSink {
 public:
  virtual ~LinkMessageSink() {}
  // Sends msg on port. The sink sets seq and t_ms.
  virtual void Send(uint8_t port, navbot_link_v1_Message& msg) = 0;
};

// Log frames from LinkPublishLog go to UART1 only.
inline bool LinkLogReachesPort(uint8_t port) { return port == kLinkPortUart; }

class LinkEndpoint : private LinkRowSink {
 public:
  // build is the firmware build ID reported with catalog entry 0 (up to 32 characters).
  LinkEndpoint(const SignalEntry* table, size_t count, const char* build, const LinkPortBudget (&budgets)[kLinkPortCount],
               LinkMessageSink& sink);

  // Applies one request that arrived on port and sends the reply to the same port. nowUs is the micros() value at the
  // start of the tick, and stamps lease renewals.
  void Handle(uint8_t port, const uint8_t* payload, size_t len, uint32_t nowUs);

  // End of a tick: sends the due rows of every subscription as SignalBatch frames to the port that owns them.
  void Sample(uint32_t tickStartUs);

  // Payloads that passed CRC but that nanopb could not parse.
  uint32_t undecodable() const { return undecodable_; }

 private:
  void OnRow(uint8_t port, uint8_t slot, const uint8_t* row, size_t len) override;
  void OnLeaseExpired(uint8_t port, uint8_t slot) override;

  void SendAck(uint8_t port, uint32_t requestSeq, const char* error, const uint8_t* value, size_t valueLen);
  void SendLog(uint8_t port, navbot_link_v1_Log_Level level, const char* tag, const char* text);
  void SendCatalogEntry(uint8_t port, uint32_t index);
  bool CheckIndex(uint8_t port, uint32_t requestSeq, uint32_t index);

  const SignalEntry* table_;
  size_t count_;
  const char* build_;
  LinkSubscriptions subscriptions_;
  LinkMessageSink& sink_;
  uint32_t undecodable_ = 0;
};

#endif
