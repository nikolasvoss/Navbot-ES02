#include "link_endpoint.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "link_log.h"

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

uint32_t GetLe32(const uint8_t* in) {
  return static_cast<uint32_t>(in[0]) | static_cast<uint32_t>(in[1]) << 8 | static_cast<uint32_t>(in[2]) << 16 |
         static_cast<uint32_t>(in[3]) << 24;
}

// A value as the log shows it: a float with 6 significant digits, an integer or a bool as a plain number.
void FormatValue(SignalType type, uint32_t bits, char (&out)[24]) {
  if (type == SignalType::F32) {
    float value;
    memcpy(&value, &bits, sizeof(value));
    snprintf(out, sizeof(out), "%.6g", static_cast<double>(value));
  } else if (type == SignalType::I32) {
    snprintf(out, sizeof(out), "%ld", static_cast<long>(static_cast<int32_t>(bits)));
  } else {
    snprintf(out, sizeof(out), "%lu", static_cast<unsigned long>(bits));
  }
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

// A started action is logged with its arguments, so a capture on either port shows what was started and with what.
void LinkEndpoint::SendActionLog(const SignalEntry& entry, SignalArgs args, const float* values) {
  char text[sizeof(navbot_link_v1_Log::text)];
  int length = snprintf(text, sizeof(text), "%s started", entry.name);
  for (size_t i = 0; i < args.count && static_cast<size_t>(length) < sizeof(text); ++i)
    length += snprintf(text + length, sizeof(text) - static_cast<size_t>(length), " %s=%g", args.items[i].name,
                       static_cast<double>(values[i]));
  LinkSendLog(navbot_link_v1_Log_Level_INFO, "action", text, sink_);
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
  const SignalArgs args = entry.args ? entry.args() : SignalArgs{nullptr, 0};
  for (size_t i = 0; i < args.count; ++i) {
    CopyText(out.args[i].name, args.items[i].name);
    out.args[i].min = args.items[i].min;
    out.args[i].max = args.items[i].max;
  }
  out.args_count = args.count;
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

void LinkEndpoint::Handle(uint8_t port, const uint8_t* payload, size_t len, uint32_t nowUs, bool balanceRequested) {
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
      const navbot_link_v1_SetParam& set = request.body.set_param;
      if (!CheckIndex(port, seq, set.id)) break;
      const SignalEntry& entry = table_[set.id];
      if (entry.kind != SignalKind::Param) {
        snprintf(error, sizeof(error), "%s is not a parameter", entry.name);
        SendAck(port, seq, error, nullptr, 0);
        break;
      }
      // Every parameter feeds the balance controller or a sensor filter, so none changes while it runs.
      if (balanceRequested) {
        snprintf(error, sizeof(error), "%s: refused while balancing", entry.name);
        SendAck(port, seq, error, nullptr, 0);
        break;
      }
      if (set.value.size != 4) {
        snprintf(error, sizeof(error), "%s takes a 4-byte value", entry.name);
        SendAck(port, seq, error, nullptr, 0);
        break;
      }
      const uint32_t before = entry.read();
      const char* reason = nullptr;
      if (!entry.write(GetLe32(set.value.bytes), &reason)) {
        snprintf(error, sizeof(error), "%s: %s", entry.name, reason);
        SendAck(port, seq, error, nullptr, 0);
        break;
      }
      const uint32_t after = entry.read();
      uint8_t value[4];
      PutLe32(value, after);
      SendAck(port, seq, nullptr, value, sizeof(value));
      char oldText[24], newText[24], text[sizeof(navbot_link_v1_Log::text)];
      FormatValue(entry.type, before, oldText);
      FormatValue(entry.type, after, newText);
      snprintf(text, sizeof(text), "%s: %s -> %s", entry.name, oldText, newText);
      LinkSendLog(navbot_link_v1_Log_Level_INFO, "param", text, sink_);
      break;
    }
    case navbot_link_v1_Message_call_tag: {
      const navbot_link_v1_Call& call = request.body.call;
      if (!CheckIndex(port, seq, call.id)) break;
      const SignalEntry& entry = table_[call.id];
      if (entry.kind != SignalKind::Action) {
        snprintf(error, sizeof(error), "%s is not an action", entry.name);
        SendAck(port, seq, error, nullptr, 0);
        break;
      }
      const SignalArgs args = entry.args();
      if (call.args_count != args.count) {
        snprintf(error, sizeof(error), "%s takes %u arguments", entry.name, static_cast<unsigned>(args.count));
        SendAck(port, seq, error, nullptr, 0);
        break;
      }
      // The bounds are the table's; the handler checks what the bounds cannot say and the hardware state.
      bool inBounds = true;
      for (size_t i = 0; i < args.count && inBounds; ++i) {
        const float value = call.args[i];
        if (value >= args.items[i].min && value <= args.items[i].max) continue;
        snprintf(error, sizeof(error), "%s=%g is outside [%g, %g]", args.items[i].name, static_cast<double>(value),
                 static_cast<double>(args.items[i].min), static_cast<double>(args.items[i].max));
        SendAck(port, seq, error, nullptr, 0);
        inBounds = false;
      }
      if (!inBounds) break;
      const char* reason = nullptr;
      if (!entry.call(call.args, &reason)) {
        SendAck(port, seq, reason ? reason : "refused", nullptr, 0);
        break;
      }
      SendAck(port, seq, nullptr, nullptr, 0);
      SendActionLog(entry, args, call.args);
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

void LinkEndpoint::OnLeaseExpired(uint8_t, uint8_t slot) {
  char text[sizeof(navbot_link_v1_Log::text)];
  snprintf(text, sizeof(text), "slot %u lease expired", static_cast<unsigned>(slot));
  LinkSendLog(navbot_link_v1_Log_Level_WARN, "link", text, sink_);
}
