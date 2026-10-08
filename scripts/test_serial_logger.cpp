#include "Arduino.h"
#include "Logging.h"
#include "LoggingInternal.h"
#include "freertos/task.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <thread>

namespace {
void require(bool condition, const char *message) {
  if (condition) return;
  std::fprintf(stderr, "%s\n", message);
  std::fflush(stderr);
  std::_Exit(1);
}

void compareFixture(const std::map<std::string, std::string> &fixtures, const std::string &key,
                    const LoggingInternal::Record &record) {
  const std::map<std::string, std::string>::const_iterator fixture = fixtures.find(key);
  require(fixture != fixtures.end(), "format fixture is missing");
  char actual[384];
  const int length = LoggingInternal::formatRecord(record, actual, sizeof(actual), 0, 0);
  require(length == static_cast<int>(fixture->second.size()) &&
              std::memcmp(actual, fixture->second.data(), fixture->second.size()) == 0,
          "typed formatter differs from the checked-in legacy output fixture");
}

void verifyFormatters(const char *fixturePath) {
  std::ifstream input(fixturePath);
  require(input.good(), "format fixture file could not be opened");
  std::map<std::string, std::string> fixtures;
  std::string line;
  while (std::getline(input, line)) {
    const size_t separator = line.find('\t');
    require(separator != std::string::npos, "format fixture row has no key separator");
    std::string value = line.substr(separator + 1);
    for (size_t position = 0; (position = value.find("\\n", position)) != std::string::npos;)
      value.replace(position, 2, "\n");
    fixtures[line.substr(0, separator)] = value;
  }
  require(fixtures.size() == 66, "format fixture does not cover every active selector and profile");

  const LoggingInternal::RecordKind kinds[] = {
      LoggingInternal::RecordKind::Diagnostic, LoggingInternal::RecordKind::Trace,
      LoggingInternal::RecordKind::Control, LoggingInternal::RecordKind::Balance,
      LoggingInternal::RecordKind::Drive};
  for (size_t i = 0; i < 5; ++i) {
    LoggingInternal::Record record{};
    record.kind = kinds[i];
    compareFixture(fixtures, "csv." + std::to_string(i), record);
  }

  for (int selector = 1; selector <= 45; ++selector) {
    LoggingInternal::Record record{};
    record.kind = LoggingInternal::RecordKind::Debug;
    record.payload.debug.selector = static_cast<Logging::DebugSelector>(selector);
    compareFixture(fixtures, "debug." + std::to_string(selector), record);
  }
  for (int selector = 60; selector <= 75; ++selector) {
    LoggingInternal::Record record{};
    record.kind = LoggingInternal::RecordKind::Debug;
    record.payload.debug.selector = static_cast<Logging::DebugSelector>(selector);
    compareFixture(fixtures, "debug." + std::to_string(selector), record);
  }

  char actual[384];
  LoggingInternal::Record message{};
  message.kind = LoggingInternal::RecordKind::Message;
  message.payload.message.tagLength = 3;
  message.payload.message.textLength = 4;
  std::memcpy(message.payload.message.tag, "imu", 3);
  std::memcpy(message.payload.message.text, "ready", 4);
  const int messageLength = LoggingInternal::formatRecord(message, actual, sizeof(actual), 0, 0);
  require(messageLength == 10 && std::string(actual, messageLength) == "imu: read\n",
          "bounded message formatter changed its line layout");

  require(sizeof(LoggingInternal::Record) <= 168, "queue record exceeds its memory budget");
  std::printf("queue_record_bytes=%zu queue_depth=%u queue_storage_bytes=%zu formatters=all_profiles\n",
              sizeof(LoggingInternal::Record), static_cast<unsigned>(Logging::kQueueDepth),
              sizeof(LoggingInternal::Record) * Logging::kQueueDepth);
}

Logging::DiagnosticSample diagnostic(uint32_t id) {
  Logging::DiagnosticSample sample{};
  sample.timestampUs = id;
  sample.sequence = id;
  sample.receiverFrameAgeMs = -1;
  sample.receiverFailsafe = -1;
  return sample;
}

void verifyQueue() {
  require(Logging::begin(), "logger sender did not start");
  Logging::setProfile(Logging::Profile::Diagnostic);
  for (uint32_t id = 0; id < 32; ++id) {
    require(Logging::submit(diagnostic(id)) == Logging::Result::Accepted,
            "producer rejected a record into an empty queue");
    require(waitForSerialLines(id + 1), "sender did not drain a submitted row");
  }

  blockSerialWrites();
  require(Logging::submit(diagnostic(32)) == Logging::Result::Accepted,
          "blocked-sender record was not accepted");
  require(waitForSerialWriteBlocked(), "sender did not enter blocked UART write");
  for (uint32_t id = 33; id < 33 + Logging::kQueueDepth; ++id)
    require(Logging::submit(diagnostic(id)) == Logging::Result::Accepted,
            "queue filled before its declared bound");

  const auto start = std::chrono::steady_clock::now();
  require(Logging::submit(diagnostic(65)) == Logging::Result::QueueFull,
          "full queue did not reject the next record");
  require(std::chrono::steady_clock::now() - start < std::chrono::milliseconds(100),
          "producer waited while UART sender was stalled");
  Logging::Status status = Logging::status();
  require(status.incompleteReason == Logging::IncompleteReason::QueueFull,
          "overflow did not latch incomplete status");
  require(Logging::submit(diagnostic(66)) == Logging::Result::Incomplete,
          "logger accepted a row after overflow");
  releaseSerialWrites();
  require(waitForSerialLines(65), "sender did not drain accepted rows after release");

  std::istringstream rows(serialOutput());
  std::string row;
  uint32_t expected = 0;
  while (std::getline(rows, row)) {
    const size_t comma = row.find(',');
    require(comma != std::string::npos, "diagnostic row lacks its timestamp field");
    require(static_cast<uint32_t>(std::stoul(row.substr(0, comma))) == expected,
            "queue output order or record reuse is incorrect");
    ++expected;
  }
  require(expected == 65, "accepted queue rows were lost or duplicated");
  require(Logging::status().incompleteReason == Logging::IncompleteReason::QueueFull,
          "draining cleared the incomplete state");
  require(Logging::status().rejectedRecords == 2,
          "overflow and post-overflow rejection counters are incorrect");
  const uint32_t failedEpoch = Logging::status().profileEpoch;
  Logging::setProfile(Logging::Profile::Idle);
  require(Logging::status().incompleteReason == Logging::IncompleteReason::QueueFull,
          "profile change cleared the incomplete latch");
  require(Logging::status().profileEpoch == failedEpoch + 1,
          "profile change did not advance its epoch");
  std::puts("blocked_sender=nonblocking; capacity=32; overflow=sticky; fifo=65; reuse=verified");
}

void verifyMessagesAndBoundary() {
  require(Logging::begin(), "logger sender did not start");
  Logging::setLevel(Logging::Level::Info);
  require(Logging::message(Logging::Level::Debug, "filtered", "line") == Logging::Result::Filtered,
          "severity filter did not reject a low-priority message");
  require(Logging::message(Logging::Level::Warning, "inflight", "before") == Logging::Result::Accepted,
          "idle message was not accepted");
  require(waitForSerialLines(1), "idle message was not written");

  char longText[128];
  std::memset(longText, 'x', sizeof(longText) - 1);
  longText[sizeof(longText) - 1] = '\0';
  require(Logging::message(Logging::Level::Info, "long", "%s", longText) == Logging::Result::Truncated,
          "oversize message was not reported as truncated");

  blockSerialWrites();
  require(Logging::message(Logging::Level::Info, "inflight", "claimed") == Logging::Result::Accepted,
          "in-flight boundary message was not accepted");
  require(waitForSerialWriteBlocked(), "sender did not claim the boundary message");
  require(Logging::message(Logging::Level::Info, "stale", "queued") == Logging::Result::Accepted,
          "queued pre-boundary message was not accepted");
  Logging::setProfile(Logging::Profile::Trace);
  require(Logging::message(Logging::Level::Error, "suppressed", "capture") == Logging::Result::Suppressed,
          "text message was not suppressed during structured capture");
  Logging::TraceSample sample{};
  sample.timestampMs = 7;
  require(Logging::submit(sample) == Logging::Result::Accepted,
          "typed trace was not accepted after profile transition");
  releaseSerialWrites();
  require(waitForSerialLines(2), "in-flight message and trace were not emitted");
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  const std::string output = serialOutput();
  require(output.find("inflight: claimed\n") != std::string::npos,
          "already claimed pre-boundary write did not finish");
  require(output.find("stale:") == std::string::npos,
          "queued pre-boundary message contaminated structured capture");
  require(output.find("suppressed:") == std::string::npos,
          "capture-time message contaminated structured output");
  require(output.find("TRACE,7,") != std::string::npos,
          "typed record was not emitted after profile transition");
  const Logging::Status status = Logging::status();
  require(status.filteredMessages == 1 && status.suppressedMessages == 1 &&
              status.truncatedMessages == 1 && status.incompleteReason == Logging::IncompleteReason::None,
          "message status counters or truncation latch are incorrect");
}

void verifyShortWrite() {
  require(Logging::begin(), "logger sender did not start");
  Logging::setProfile(Logging::Profile::Diagnostic);
  shortNextSerialWrite();
  require(Logging::submit(diagnostic(1)) == Logging::Result::Accepted,
          "record before forced short write was rejected");
  for (int i = 0; i < 100 && Logging::status().incompleteReason == Logging::IncompleteReason::None; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  const Logging::Status status = Logging::status();
  require(status.incompleteReason == Logging::IncompleteReason::SenderWriteFailed && status.writeFailures == 1,
          "short UART write did not latch an incomplete recording");
}

void verifyStartFailure() {
  failNextTaskCreation();
  require(!Logging::begin(), "sender task creation failure was not reported");
  require(Logging::status().incompleteReason == Logging::IncompleteReason::SenderStartFailed,
          "sender task creation failure did not latch status");
  Logging::setProfile(Logging::Profile::Diagnostic);
  require(Logging::submit(diagnostic(1)) == Logging::Result::Incomplete,
          "logger accepted data after sender startup failed");
}

void verifySelectedPacing() {
  require(Logging::begin(), "logger sender did not start");
  Logging::setProfile(Logging::Profile::SelectedDebug);
  Logging::DebugSample sample{};
  sample.selector = Logging::DebugSelector::K1;
  size_t accepted = 0;
  for (size_t i = 0; i < 200; ++i) {
    const Logging::Result result = Logging::submit(sample);
    if (result == Logging::Result::Accepted) ++accepted;
    else require(result == Logging::Result::Paced, "selected debug submit returned an unexpected result");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(accepted >= 8 && accepted <= 12, "selected debug output did not retain its 50 Hz cap");
  require(Logging::status().incompleteReason == Logging::IncompleteReason::None,
          "paced selected-debug rows were counted as data loss");
}
}

int main(int argc, char **argv) {
  const char *mode = argc > 1 ? argv[1] : "format";
  if (std::strcmp(mode, "queue") == 0) verifyQueue();
  else if (std::strcmp(mode, "messages") == 0) verifyMessagesAndBoundary();
  else if (std::strcmp(mode, "writefail") == 0) verifyShortWrite();
  else if (std::strcmp(mode, "startfail") == 0) verifyStartFailure();
  else if (std::strcmp(mode, "pace") == 0) verifySelectedPacing();
  else verifyFormatters(argc > 2 ? argv[2] : "scripts/logging_formats.txt");
  std::fflush(stdout);
  std::_Exit(0);
}
