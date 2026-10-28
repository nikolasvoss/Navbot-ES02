#include "link_endpoint.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace {

// The wire numbers of kinds and types are the table's own.
static_assert(static_cast<int>(SignalKind::Signal) == navbot_link_v1_CatalogEntry_Kind_SIGNAL, "kind numbers");
static_assert(static_cast<int>(SignalKind::Min) == navbot_link_v1_CatalogEntry_Kind_SIGNAL_MIN, "kind numbers");
static_assert(static_cast<int>(SignalKind::Max) == navbot_link_v1_CatalogEntry_Kind_SIGNAL_MAX, "kind numbers");
static_assert(static_cast<int>(SignalKind::Param) == navbot_link_v1_CatalogEntry_Kind_PARAM, "kind numbers");
static_assert(static_cast<int>(SignalKind::Action) == navbot_link_v1_CatalogEntry_Kind_ACTION, "kind numbers");
static_assert(static_cast<int>(SignalType::F32) == navbot_link_v1_CatalogEntry_Type_F32, "type numbers");
static_assert(static_cast<int>(SignalType::I32) == navbot_link_v1_CatalogEntry_Type_I32, "type numbers");
static_assert(static_cast<int>(SignalType::U32) == navbot_link_v1_CatalogEntry_Type_U32, "type numbers");
static_assert(static_cast<int>(SignalType::Bool) == navbot_link_v1_CatalogEntry_Type_BOOL, "type numbers");

void PutLe32(uint8_t* out, uint32_t value) {
  for (int shift = 0; shift < 4; ++shift) out[shift] = static_cast<uint8_t>(value >> (8 * shift));
}

template <size_t N>
void CopyText(char (&destination)[N], const char* source) {
  strncpy(destination, source, N - 1);
  destination[N - 1] = '\0';
}

}  // namespace

LinkEndpoint::LinkEndpoint(const SignalEntry* table, size_t count, const char* build,
                           const LinkPortBudget (&budgets)[kLinkPortCount], LinkMessageSink& sink)
    : table_(table), count_(count), build_(build), subscriptions_(table, count, budgets), sink_(sink) {}

void LinkEndpoint::SendAck(uint8_t port, uint32_t requestSeq, const char* error, const uint8_t* value, size_t valueLen) {
  navbot_link_v1_Message reply = navbot_link_v1_Message_init_zero;
  reply.which_body = navbot_link_v1_Message_ack_tag;
  navbot_link_v1_Ack& ack = reply.body.ack;
  ack.command_id = requestSeq;
  ack.accepted = error == nullptr;
  if (error) CopyText(ack.error, error);
  if (value) {
    memcpy(ack.value.bytes, value, valueLen);
    ack.value.size = static_cast<pb_size_t>(valueLen);
  }
  sink_.Send(port, reply);
}

void LinkEndpoint::SendLog(uint8_t port, navbot_link_v1_Log_Level level, const char* tag, const char* text) {
  navbot_link_v1_Message message = navbot_link_v1_Message_init_zero;
  message.which_body = navbot_link_v1_Message_log_tag;
  message.body.log.level = level;
  CopyText(message.body.log.tag, tag);
  CopyText(message.body.log.text, text);
  sink_.Send(port, message);
}

void LinkEndpoint::SendCatalogEntry(uint8_t port, uint32_t index) {
  const SignalEntry& entry = table_[index];
  navbot_link_v1_Message reply = navbot_link_v1_Message_init_zero;
  reply.which_body = navbot_link_v1_Message_catalog_entry_tag;
  navbot_link_v1_CatalogEntry& out = reply.body.catalog_entry;
  out.index = index;
  out.count = static_cast<uint32_t>(count_);
  CopyText(out.name, entry.name);
  CopyText(out.unit, entry.unit);
  out.kind = static_cast<navbot_link_v1_CatalogEntry_Kind>(entry.kind);
  out.type = static_cast<navbot_link_v1_CatalogEntry_Type>(entry.type);
  for (size_t i = 0; i < entry.argCount; ++i) {
    CopyText(out.args[i].name, entry.args[i].name);
    out.args[i].min = entry.args[i].min;
    out.args[i].max = entry.args[i].max;
  }
  out.args_count = entry.argCount;
  if (index == 0) CopyText(out.build, build_);
  sink_.Send(port, reply);
}

bool LinkEndpoint::CheckIndex(uint8_t port, uint32_t requestSeq, uint32_t index) {
  if (index < count_) return true;
  char error[sizeof(navbot_link_v1_Ack::error)];
  snprintf(error, sizeof(error), "no entry %u", static_cast<unsigned>(index));
  SendAck(port, requestSeq, error, nullptr, 0);
  return false;
}

void LinkEndpoint::Handle(uint8_t port, const uint8_t* payload, size_t len, uint32_t nowUs) {
  navbot_link_v1_Message request;
  if (!LinkDecodeMessage(payload, len, request)) {
    ++undecodable_;
    return;
  }
  const uint32_t seq = request.seq;
  char error[sizeof(navbot_link_v1_Ack::error)];
  switch (request.which_body) {
    case navbot_link_v1_Message_get_catalog_tag:
      if (CheckIndex(port, seq, request.body.get_catalog.index)) SendCatalogEntry(port, request.body.get_catalog.index);
      break;
    case navbot_link_v1_Message_subscribe_tag: {
      const navbot_link_v1_Subscribe& s = request.body.subscribe;
      const LinkSubscribeResult result = subscriptions_.Subscribe(port, s.slot, s.ids, s.ids_count, s.period_us, nowUs);
      uint8_t slot[4];
      PutLe32(slot, result.slot);
      if (result.accepted)
        SendAck(port, seq, nullptr, slot, sizeof(slot));
      else
        SendAck(port, seq, result.error, nullptr, 0);
      break;
    }
    case navbot_link_v1_Message_unsubscribe_tag: {
      const LinkSubscribeResult result = subscriptions_.Unsubscribe(port, request.body.unsubscribe.slot);
      SendAck(port, seq, result.accepted ? nullptr : result.error, nullptr, 0);
      break;
    }
    case navbot_link_v1_Message_get_param_tag: {
      const uint32_t id = request.body.get_param.id;
      if (!CheckIndex(port, seq, id)) break;
      const SignalEntry& entry = table_[id];
      if (entry.kind == SignalKind::Action) {
        snprintf(error, sizeof(error), "%s is an action", entry.name);
        SendAck(port, seq, error, nullptr, 0);
        break;
      }
      uint8_t value[4];
      PutLe32(value, entry.read());
      SendAck(port, seq, nullptr, value, sizeof(value));
      break;
    }
    case navbot_link_v1_Message_set_param_tag: {
      const uint32_t id = request.body.set_param.id;
      if (!CheckIndex(port, seq, id)) break;
      snprintf(error, sizeof(error), table_[id].kind == SignalKind::Param ? "%s cannot be set yet" : "%s is not a parameter",
               table_[id].name);
      SendAck(port, seq, error, nullptr, 0);
      break;
    }
    case navbot_link_v1_Message_call_tag: {
      const uint32_t id = request.body.call.id;
      if (!CheckIndex(port, seq, id)) break;
      snprintf(error, sizeof(error), table_[id].kind == SignalKind::Action ? "%s cannot be called yet" : "%s is not an action",
               table_[id].name);
      SendAck(port, seq, error, nullptr, 0);
      break;
    }
    default:
      break;  // Command and anything unknown: the firmware does not act on them yet
  }
}

void LinkEndpoint::Sample(uint32_t tickStartUs) { subscriptions_.Sample(tickStartUs, *this); }

void LinkEndpoint::OnRow(uint8_t port, uint8_t slot, const uint8_t* row, size_t len) {
  navbot_link_v1_Message message = navbot_link_v1_Message_init_zero;
  message.which_body = navbot_link_v1_Message_signal_batch_tag;
  message.body.signal_batch.slot = slot;
  memcpy(message.body.signal_batch.rows.bytes, row, len);
  message.body.signal_batch.rows.size = static_cast<pb_size_t>(len);
  sink_.Send(port, message);
}

void LinkEndpoint::OnLeaseExpired(uint8_t port, uint8_t slot) {
  if (!LinkLogReachesPort(port)) return;  // a Log between K rows on USB would corrupt the capture
  char text[sizeof(navbot_link_v1_Log::text)];
  snprintf(text, sizeof(text), "slot %u lease expired", static_cast<unsigned>(slot));
  SendLog(port, navbot_link_v1_Log_Level_WARN, "link", text);
}
