#include "SerialLogger.h"
#include "Arduino.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>

namespace {
void require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "%s\n", message);
    std::exit(1);
  }
}

SerialLogRecord sample(SerialLogKind kind, uint32_t id = 0) {
  SerialLogRecord row{};
  row.kind = kind;
  row.timestamp = id;
  row.sequence = id;
  row.dropped = UINT32_MAX;
  row.writeFailures = UINT32_MAX;
  for (size_t i = 0; i < 29; ++i) row.values[i] = -999.999f;
  return row;
}

size_t lineCount(const std::string &bytes) {
  size_t count = 0;
  for (char ch : bytes) if (ch == '\n') ++count;
  return count;
}
}

int main() {
  SerialLoggerSetSelectedMode(55);
  require(SerialLoggerSelectedMode(), "trace mode 55 was not selected");
  SerialLoggerSetSelectedMode(4);
  require(SerialLoggerSelectedMode(), "selected diagnostic mode was not marked as a log stream");
  SerialLoggerSetSelectedMode(59);
  require(!SerialLoggerSelectedMode(), "non-trace mode was treated as CSV");

  char line[384];
  const SerialLogKind kinds[] = {SERIAL_LOG_DIAGNOSTIC, SERIAL_LOG_TRACE,
      SERIAL_LOG_CONTROL, SERIAL_LOG_BALANCE, SERIAL_LOG_DRIVE};
  const size_t expectedFields[] = {17, 21, 18, 16, 33};
  for (size_t i = 0; i < 5; ++i) {
    const int length = formatSerialLogRecord(sample(kinds[i]), line, sizeof(line));
    require(length > 0 && static_cast<size_t>(length) < sizeof(line), "formatted row exceeds sender buffer");
    size_t fields = 1;
    for (int j = 0; j < length; ++j) if (line[j] == ',') ++fields;
    require(fields == expectedFields[i], "existing CSV row field count changed");
  }

  auto selected = sample(SERIAL_LOG_SELECTED_DEBUG);
  selected.selected.selector = 1;
  selected.selected.values[0] = 0.125f;
  selected.selected.values[1] = 1.25f;
  selected.selected.values[2] = -2.5f;
  selected.selected.values[3] = 3.75f;
  const int selectedLength = formatSerialLogRecord(selected, line, sizeof(line));
  require(selectedLength > 0 && std::string(line, selectedLength) ==
      "dt:0.125000 Roll:1.25 Pitch:-2.50 Yaw:3.75\n", "selected debug format changed");

  std::printf("queue_record_bytes=%zu queue_depth=%u queue_storage_bytes=%zu\n",
      sizeof(SerialLogRecord), static_cast<unsigned>(kSerialLoggerQueueDepth),
      sizeof(SerialLogRecord) * kSerialLoggerQueueDepth);
  SerialLoggerBegin();

  for (uint32_t id = 0; id < 32; ++id) {
    require(SerialLoggerSubmit(sample(SERIAL_LOG_DIAGNOSTIC, id)), "producer rejected a record into an empty queue");
    require(waitForSerialLines(id + 1), "sender did not drain a submitted row");
  }
  require(lineCount(serialOutput()) == 32, "drained queue emitted extra rows");

  blockSerialWrites();
  require(SerialLoggerSubmit(sample(SERIAL_LOG_DIAGNOSTIC, 32)), "blocked-sender record was not accepted");
  require(waitForSerialWriteBlocked(), "sender did not enter blocked UART write");
  for (uint32_t id = 33; id < 33 + kSerialLoggerQueueDepth; ++id)
    require(SerialLoggerSubmit(sample(SERIAL_LOG_DIAGNOSTIC, id)), "queue filled before its declared bound");

  const auto start = std::chrono::steady_clock::now();
  require(!SerialLoggerSubmit(sample(SERIAL_LOG_DIAGNOSTIC, 65)), "full queue did not reject the next record");
  const auto elapsed = std::chrono::steady_clock::now() - start;
  require(elapsed < std::chrono::milliseconds(100), "producer waited while UART sender was stalled");
  require(SerialLoggerIncomplete() == SerialLoggerIncompleteReason::QueueFull,
          "overflow did not latch incomplete status");
  require(!SerialLoggerSubmit(sample(SERIAL_LOG_DIAGNOSTIC, 66)), "logger accepted a row after overflow");
  require(SerialLoggerRejectedCount() == 2, "rejected-record counter is incorrect");
  releaseSerialWrites();
  require(waitForSerialLines(65), "sender did not drain accepted rows after release");

  const std::string output = serialOutput();
  std::istringstream rows(output);
  std::string row;
  uint32_t expected = 0;
  while (std::getline(rows, row)) {
    const size_t comma = row.find(',');
    require(comma != std::string::npos, "diagnostic row lacks sequence field");
    const uint32_t timestamp = static_cast<uint32_t>(std::stoul(row.substr(0, comma)));
    require(timestamp == expected, "queue output order or record reuse is incorrect");
    ++expected;
  }
  require(expected == 65, "accepted queue rows were lost or duplicated");
  require(SerialLoggerIncomplete() == SerialLoggerIncompleteReason::QueueFull,
          "draining cleared the incomplete state");
  std::puts("blocked_sender=producer_nonblocking; capacity=32; overflow=sticky; fifo=65; reuse=verified");
  std::fflush(stdout);
  std::_Exit(0);
}
