#include "SerialLogger.h"
#include "Arduino.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <thread>

namespace {
void require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "%s\n", message);
    std::exit(1);
  }
}

SerialLogRecord sample(SerialLogKind kind) {
  SerialLogRecord row{};
  row.kind = kind;
  row.timestamp = UINT32_MAX;
  row.sequence = UINT32_MAX;
  row.dropped = UINT32_MAX;
  row.writeFailures = UINT32_MAX;
  for (size_t i = 0; i < 29; ++i) row.values[i] = -999.999f;
  return row;
}
}

int main() {
  SerialLoggerSetTraceMode(55);
  require(SerialLoggerTraceModeSelected(), "trace mode 55 was not selected");
  SerialLoggerSetTraceMode(58);
  require(SerialLoggerTraceModeSelected(), "trace mode 58 was not selected");
  SerialLoggerSetTraceMode(59);
  require(!SerialLoggerTraceModeSelected(), "non-trace mode was treated as CSV");

  char line[384];
  for (SerialLogKind kind : {SERIAL_LOG_DIAGNOSTIC, SERIAL_LOG_TRACE, SERIAL_LOG_CONTROL,
                             SERIAL_LOG_BALANCE, SERIAL_LOG_DRIVE}) {
    const int length = formatSerialLogRecord(sample(kind), line, sizeof(line));
    require(length > 0 && static_cast<size_t>(length) < sizeof(line), "formatted row exceeds sender buffer");
    size_t fields = 1;
    for (int i = 0; i < length; ++i) {
      if (line[i] == ',') ++fields;
    }
    const size_t expected[] = {17, 21, 18, 16, 33};
    require(fields == expected[kind], "formatted row field count mismatch");
    const double sampleHz = kind == SERIAL_LOG_DIAGNOSTIC ? 100.0 : 1000.0 / 7.0;
    const double bitsPerSecond = static_cast<double>(length) * 10.0 * sampleHz;
    std::printf("kind=%u fields=%zu sample_bytes_with_LF=%d uart_framing_bits_per_row=%d sample_rate_hz=%.1f budget_bits_per_second=%.0f budget_pct_576000=%.1f\n",
                static_cast<unsigned>(kind), fields, length, length * 10, sampleHz,
                bitsPerSecond, 100.0 * bitsPerSecond / 576000.0);
  }
  std::printf("queue_record_bytes=%zu queue_depth=%u queue_storage_bytes=%zu\n",
              sizeof(SerialLogRecord), 16u, sizeof(SerialLogRecord) * 16u);

  blockSerialWrites();
  SerialLoggerBegin();
  auto row = sample(SERIAL_LOG_DIAGNOSTIC);
  require(SerialLoggerSubmit(row), "first record was not enqueued");
  require(waitForSerialWriteBlocked(), "sender did not enter blocked UART write");
  for (unsigned i = 0; i < 16; ++i) require(SerialLoggerSubmit(row), "queue filled before blocked sender held first row");
  const auto start = std::chrono::steady_clock::now();
  require(!SerialLoggerSubmit(row), "queue-full submission was not dropped");
  const auto elapsed = std::chrono::steady_clock::now() - start;
  require(elapsed < std::chrono::milliseconds(100), "producer blocked while UART sender was stalled");
  releaseSerialWrites();
  std::puts("blocked_sender=held; queue_full=drop; producer_submit_ms<100");
  std::fflush(stdout);
  std::_Exit(0);
}
