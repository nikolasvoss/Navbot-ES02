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
  char actual[LoggingInternal::kLineBufferSize];
  const int length = LoggingInternal::formatRecord(record, actual, sizeof(actual), 0, 0);
  if (length != static_cast<int>(fixture->second.size()) ||
      std::memcmp(actual, fixture->second.data(), fixture->second.size()) != 0) {
    std::fprintf(stderr, "formatter fixture mismatch for %s: actual='%.*s' expected='%s'\n", key.c_str(),
                 length > 0 ? length : 0, actual, fixture->second.c_str());
    require(false, "typed formatter differs from the checked-in legacy output fixture");
  }
}

// Every DRIVE float at -9999.0 and every integer at its widest printable value: the longest row a
// DRIVE record can print. The sender line buffer must hold it.
void fillWidestDrive(Logging::DriveSample &d) {
  const uint32_t u = 0xffffffffu;
  const float f = -9999.0f;
  d.timestampMs = d.sequence = d.imuSequence = d.imuReadTimeUs = d.imuAcceptedSamples = u;
  d.imuLongGapCount = d.imuMaxIntervalUs = d.controlStartUs = d.controlExecutionUs = d.maxLoopEntryIntervalUs = u;
  d.gainMode = d.maxServoRange = -999;
  d.minimumBatteryRawV = d.batteryV = d.imuDtSec = f;
  d.requestedSpeed = d.leftWheelVelocity = d.rightWheelVelocity = d.controlDtSec = f;
  d.bodyPitchFiltered = d.rollOk = f;
  d.leftMotorTarget = d.rightMotorTarget = d.leftCurrentSetpoint = d.rightCurrentSetpoint = f;
  d.leftVoltageQ = d.rightVoltageQ = d.leftShaftVelocity = d.rightShaftVelocity = f;
  d.batteryInstantRawV = d.batteryInstantFilteredV = d.ballX = d.bodyPitch = f;
}

// Column i of the DRIVE row (0-based, after the tag, in DRIVE_COLUMNS order) holds the value i + 1.
// logger_dropped (column 34) and uart_write_failures (column 35) are formatRecord arguments 34 and 35.
// scripts/capture_balance_trace.py reads these values back and checks each one against its column name.
void fillDistinctDrive(Logging::DriveSample &d) {
  d.timestampMs = 1;                 // time_ms
  d.gainMode = 2;                    // ch5_mode
  d.minimumBatteryRawV = 3;          // voltage_min_raw_v
  d.batteryV = 4;                    // voltage_filtered_v
  d.imuSequence = 5;                 // imu_sequence
  d.imuReadTimeUs = 6;               // imu_read_time_us
  d.imuDtSec = 7;                    // imu_dt_s
  d.imuAcceptedSamples = 8;          // imu_accepted_samples
  d.imuLongGapCount = 9;             // imu_long_gap_count
  d.imuMaxIntervalUs = 10;            // imu_max_interval_us
  d.controlStartUs = 11;              // control_start_us
  d.controlExecutionUs = 12;          // control_execution_us
  d.maxLoopEntryIntervalUs = 13;      // max_loop_entry_interval_us
  d.requestedSpeed = 14;              // ch3_target
  d.leftWheelVelocity = 15;           // motor1_velocity_f
  d.rightWheelVelocity = 16;          // motor2_velocity_f
  d.controlDtSec = 17;                // tick_dt_s
  d.bodyPitchFiltered = 18;           // body_pitching_f
  d.rollOk = 19;                      // roll_ok
  d.leftMotorTarget = 20;             // motor1_target
  d.rightMotorTarget = 21;            // motor2_target
  d.leftCurrentSetpoint = 22;         // motor1_current_sp
  d.rightCurrentSetpoint = 23;        // motor2_current_sp
  d.leftVoltageQ = 24;                // motor1_voltage_q
  d.rightVoltageQ = 25;               // motor2_voltage_q
  d.leftShaftVelocity = 26;           // motor1_shaft_velocity
  d.rightShaftVelocity = 27;          // motor2_shaft_velocity
  d.batteryInstantRawV = 28;          // battery_raw_instant_v
  d.batteryInstantFilteredV = 29;     // battery_filtered_instant_v
  d.ballX = 30;                       // top_ball_x
  d.bodyPitch = 31;                   // body_pitching
  d.maxServoRange = 32;               // max_servo_range_deg
  d.sequence = 33;                    // sequence
}

void printDistinctDrive() {
  LoggingInternal::Record record{};
  record.kind = LoggingInternal::RecordKind::Drive;
  fillDistinctDrive(record.payload.drive);
  char line[LoggingInternal::kLineBufferSize];
  const int length = LoggingInternal::formatRecord(record, line, sizeof(line), 34, 35);
  require(length > 0 && static_cast<size_t>(length) < sizeof(line), "distinct DRIVE row did not format");
  std::fwrite(line, 1, static_cast<size_t>(length), stdout);
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
  require(fixtures.size() == 52, "format fixture does not cover every active selector and profile");

  const LoggingInternal::RecordKind kinds[] = {
      LoggingInternal::RecordKind::Diagnostic, LoggingInternal::RecordKind::Trace,
      LoggingInternal::RecordKind::Control, LoggingInternal::RecordKind::Balance,
      LoggingInternal::RecordKind::Drive};
  for (size_t i = 0; i < 5; ++i) {
    LoggingInternal::Record record{};
    record.kind = kinds[i];
    compareFixture(fixtures, "csv." + std::to_string(i), record);
  }

  for (int selector = 1; selector <= 48; ++selector) {
    if (selector == 22 || selector == 23 || selector == 26 || selector == 27 || selector == 29 || selector == 30 || selector == 31 ||
        selector == 32 || selector == 34 || selector == 35 || selector == 36 || selector == 38 || selector == 39 ||
        selector == 40 || selector == 41 || selector == 42 || selector == 44)
      continue;  // Removed selectors have no formatter.
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

  char actual[LoggingInternal::kLineBufferSize];
  LoggingInternal::Record message{};
  message.kind = LoggingInternal::RecordKind::Message;
  message.payload.message.tagLength = 3;
  message.payload.message.textLength = 4;
  std::memcpy(message.payload.message.tag, "imu", 3);
  std::memcpy(message.payload.message.text, "ready", 4);
  const int messageLength = LoggingInternal::formatRecord(message, actual, sizeof(actual), 0, 0);
  require(messageLength == 10 && std::string(actual, messageLength) == "imu: read\n",
          "bounded message formatter changed its line layout");

  require(sizeof(LoggingInternal::Record) <= 224, "queue record exceeds its memory budget");
  LoggingInternal::Record widestDrive{};
  widestDrive.kind = LoggingInternal::RecordKind::Drive;
  fillWidestDrive(widestDrive.payload.drive);
  char widestLine[LoggingInternal::kLineBufferSize];
  const int widestLength = LoggingInternal::formatRecord(widestDrive, widestLine, sizeof(widestLine), 0xffffffffu,
                                                         0xffffffffu);
  require(widestLength > 0 && static_cast<size_t>(widestLength) < sizeof(widestLine),
          "widest DRIVE row exceeds the sender line buffer");
  std::printf("drive_row_widest_bytes=%d sender_line_bytes=%zu\n", widestLength, sizeof(widestLine));
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
  require(Logging::submit(diagnostic(66)) == Logging::Result::Incomplete,
          "draining cleared the incomplete state");
  Logging::setProfile(Logging::Profile::Idle);
  Logging::setProfile(Logging::Profile::Diagnostic);
  require(Logging::submit(diagnostic(66)) == Logging::Result::Incomplete,
          "profile change cleared the incomplete latch");
  std::puts("blocked_sender=nonblocking; capacity=32; overflow=sticky; fifo=65; reuse=verified");
}

void verifyMessagesAndBoundary() {
  require(Logging::begin(), "logger sender did not start");
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
}

void verifyShortWrite() {
  require(Logging::begin(), "logger sender did not start");
  Logging::setProfile(Logging::Profile::Diagnostic);
  shortNextSerialWrite();
  require(Logging::submit(diagnostic(1)) == Logging::Result::Accepted,
          "record before forced short write was rejected");
  Logging::Result result = Logging::Result::Accepted;
  for (uint32_t id = 2; id < 102 && result != Logging::Result::Incomplete; ++id) {
    result = Logging::submit(diagnostic(id));
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  require(result == Logging::Result::Incomplete, "short UART write did not latch an incomplete recording");
}

void verifyStartFailure() {
  failNextTaskCreation();
  require(!Logging::begin(), "sender task creation failure was not reported");
  Logging::setProfile(Logging::Profile::Diagnostic);
  require(Logging::submit(diagnostic(1)) == Logging::Result::Incomplete,
          "logger accepted data after sender startup failed");
}

void verifySelectedPacing() {
  require(Logging::begin(), "logger sender did not start");
  Logging::setProfile(Logging::Profile::SelectedDebug);
  Logging::DebugSample sample{};
  sample.selector = Logging::DebugSelector::K1;
  // Offers arrive every 1 ms. The 2.5 ms minimum interval accepts one row every 3 ms
  // (t = 0, 3, 6, ... ms), so 200 offers spanning 0..199 ms yield 67 accepted rows.
  size_t accepted = 0;
  for (uint32_t i = 0; i < 200; ++i) {
    setMicrosForTest(i * 1000);
    const Logging::Result result = Logging::submit(sample);
    if (result == Logging::Result::Accepted) ++accepted;
    else require(result == Logging::Result::Paced, "selected debug submit returned an unexpected result");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(accepted == 67, "selected debug output exceeded its 2.5 ms minimum interval");
  setMicrosForTest(199000 + 2500);
  require(Logging::submit(sample) == Logging::Result::Accepted,
          "paced selected-debug rows latched the logger as incomplete");
}
}

int main(int argc, char **argv) {
  const char *mode = argc > 1 ? argv[1] : "format";
  if (std::strcmp(mode, "queue") == 0) verifyQueue();
  else if (std::strcmp(mode, "messages") == 0) verifyMessagesAndBoundary();
  else if (std::strcmp(mode, "writefail") == 0) verifyShortWrite();
  else if (std::strcmp(mode, "startfail") == 0) verifyStartFailure();
  else if (std::strcmp(mode, "pace") == 0) verifySelectedPacing();
  else if (std::strcmp(mode, "drive") == 0) printDistinctDrive();
  else verifyFormatters(argc > 2 ? argv[2] : "tests/serial_logger/logging_formats.txt");
  std::fflush(stdout);
  std::_Exit(0);
}
