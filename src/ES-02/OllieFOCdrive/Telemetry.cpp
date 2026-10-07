#include "Telemetry.h"

#include <stdarg.h>
#include <stdio.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <esp_attr.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#endif

namespace Telemetry {
namespace {

constexpr ChannelInfo kChannels[] = {
    {"Unfiltered gyro X", "rad/s"},
    {"Unfiltered gyro Y", "rad/s"},
    {"Unfiltered gyro Z", "rad/s"},
    {"Filtered gyro X", "rad/s"},
    {"Filtered gyro Y", "rad/s"},
    {"Filtered gyro Z", "rad/s"},
    {"Unfiltered accel X", "g"},
    {"Unfiltered accel Y", "g"},
    {"Unfiltered accel Z", "g"},
    {"Filtered accel X", "g"},
    {"Filtered accel Y", "g"},
    {"Filtered accel Z", "g"},
    {"Mahony roll", "deg"},
    {"Mahony pitch", "deg"},
    {"Mahony yaw", "deg"},
    {"Complementary roll", "deg"},
    {"Complementary pitch", "deg"},
    {"Complementary yaw", "deg"},
    {"IMU temperature", "°C"},
    {"Battery raw", "V"},
    {"Battery filtered", "V"},
    {"Receiver frame age", "ms"},
    {"Receiver failsafe", ""},
    {"IMU ready", ""},
};

constexpr size_t kFrameBufferSize = 512;
static_assert(sizeof(kChannels) / sizeof(kChannels[0]) == kChannelCount,
              "Telemetry metadata must cover every channel");
static_assert((kQueueCapacity & (kQueueCapacity - 1)) == 0,
              "Telemetry queue capacity must be a power of two");
StaticQueue_t gQueueControl DRAM_ATTR;
uint8_t gQueueStorage[kQueueCapacity * sizeof(Record)] DRAM_ATTR;
QueueHandle_t gQueueHandle = nullptr;
uint32_t gFailure = 0;
uint32_t gRejected = 0;
portMUX_TYPE gStateMux = portMUX_INITIALIZER_UNLOCKED;
bool gSenderStarted = false;

bool append(char *buffer, size_t capacity, size_t &length, const char *format, double value) {
  if (length >= capacity) return false;
  const int written = snprintf(buffer + length, capacity - length, format, value);
  if (written < 0 || static_cast<size_t>(written) >= capacity - length) return false;
  length += static_cast<size_t>(written);
  return true;
}

bool appendf(char *buffer, size_t capacity, size_t &length, const char *format, ...) {
  if (length >= capacity) return false;
  va_list args;
  va_start(args, format);
  const int written = vsnprintf(buffer + length, capacity - length, format, args);
  va_end(args);
  if (written < 0 || static_cast<size_t>(written) >= capacity - length) return false;
  length += static_cast<size_t>(written);
  return true;
}

void latchFailure(IncompleteReason reason) {
  portENTER_CRITICAL(&gStateMux);
  if (gFailure == 0) gFailure = static_cast<uint32_t>(reason);
  portEXIT_CRITICAL(&gStateMux);
}

void rejectRecord(IncompleteReason reason) {
  portENTER_CRITICAL(&gStateMux);
  if (gFailure == 0) gFailure = static_cast<uint32_t>(reason);
  if (gRejected < 255) ++gRejected;
  portEXIT_CRITICAL(&gStateMux);
}

bool enqueueRecord(const Record &record) {
  if (incompleteReason() != IncompleteReason::None) {
    rejectRecord(IncompleteReason::BufferFull);
    return false;
  }
  if (gQueueHandle == nullptr || xQueueSend(gQueueHandle, &record, 0) != pdPASS) {
    rejectRecord(gQueueHandle == nullptr ? IncompleteReason::SenderStartFailed :
                                           IncompleteReason::BufferFull);
    return false;
  }
  return true;
}

bool formatSelected(char *buffer, size_t capacity, size_t &length, const SelectedDebug &row) {
  const float *v = row.values;
  const int32_t *n = row.integers;
  switch (row.selector) {
    case 1: return appendf(buffer, capacity, length, "dt:%.6f Roll:%.2f Pitch:%.2f Yaw:%.2f\n", v[0], v[1], v[2], v[3]);
    case 2: return appendf(buffer, capacity, length, "dt:%.6f accx:%.2f accy:%.2f accz:%.2f\n", v[0], v[1], v[2], v[3]);
    case 3: return appendf(buffer, capacity, length, "dt:%.6f gyrox:%.4f gyroy:%.4f gyroz:%.4f\n", v[0], v[1], v[2], v[3]);
    case 4: return appendf(buffer, capacity, length, "dt:%.6f Roll:%.2f Pitch:%.2f Yaw:%.2f\n", v[0], v[1], v[2], v[3]);
    case 5: return appendf(buffer, capacity, length, "dt:%.6f eRoll:%.2f ePitch:%.2f eYaw:%.2f\n", v[0], v[1], v[2], v[3]);
    case 6: return appendf(buffer, capacity, length, " v1:%.2f v2:%.2f\n", v[0], v[1]);
    case 7: return appendf(buffer, capacity, length, " v1:%.2f v1f:%.2f\n", v[0], v[1]);
    case 8:
      for (size_t i = 0; i < 10; ++i)
        if (!appendf(buffer, capacity, length, " ch:%d", static_cast<int>(n[i]))) return false;
      return appendf(buffer, capacity, length, " sbus_dt_ms:%.2f \n", v[0]);
    case 9: return appendf(buffer, capacity, length, " PP:%.2f PI:%.2f PD:%.2f SP:%.2f SI:%.2f SD:%.2f YP:%.2f YI:%.2f YD:%.2fdt:%.6f\n", v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9]);
    case 10: return appendf(buffer, capacity, length, " twoKp:%.2f twoKi:%.2f Roll:%.2f Pitch:%.2fIMUdt:%.6f\n", v[0], v[1], v[2], v[3], v[4]);
    case 11: return appendf(buffer, capacity, length, " x:%.2f y:%.2f z:%.2f gx:%.2f gy:%.2f gz:%.2f IMUdt:%.6f\n", v[0], v[1], v[2], v[3], v[4], v[5], v[6]);
    case 12: return appendf(buffer, capacity, length, " x:%.2f Roll:%.2f\n", v[0], v[1]);
    case 13: return appendf(buffer, capacity, length, "dt:%.6f gyroxf:%.2f gyroyf:%.2f gyrozf:%.2f\n", v[0], v[1], v[2], v[3]);
    case 14: return appendf(buffer, capacity, length, "dt:%.6f accx:%.2f accy:%.2f accz:%.2f\n", v[0], v[1], v[2], v[3]);
    case 15: return appendf(buffer, capacity, length, " accy:%.2f accyf:%.2f\n", v[0], v[1]);
    case 16: return appendf(buffer, capacity, length, " gyro:%.2f gyrof:%.2f\n", v[0], v[1]);
    case 17: return appendf(buffer, capacity, length, " current_sp:%.6f\n", v[0]);
    case 18: return appendf(buffer, capacity, length, " t:%.6f a1:%.6f a11:%.6f a2:%.6f a22:%.6f\n", v[0], v[1], v[2], v[3], v[4]);
    case 19: return appendf(buffer, capacity, length, " servo1:%.2f servo2:%.2f servo3:%.2f servo4:%.2f\n", v[0], v[1], v[2], v[3]);
    case 20: return appendf(buffer, capacity, length, " vra:%.6f BodyRoll:%.6f LegLength:%.6f\n", v[0], v[1], v[2]);
    case 21: return appendf(buffer, capacity, length, " roll_ok:%.6f BodyPitching:%.6f\n", v[0], v[1]);
    case 22: return appendf(buffer, capacity, length, " it:%.5f il:%.5f oI:%.5f out:%.5f\n", v[0], v[1], v[2], v[3]);
    case 23: return appendf(buffer, capacity, length, " it:%.5f il:%.5f oI:%.5f A:%.5f out:%.5f\n", v[0], v[1], v[2], v[3], v[4]);
    case 24: return appendf(buffer, capacity, length, " EN:%.2f HZ:%.5f\n", v[0], v[1]);
    case 25: return appendf(buffer, capacity, length, " LpfOut:%.6f BodyPitching:%.6f\n", v[0], v[1]);
    case 26:
      if (n[0] == 1) return appendf(buffer, capacity, length, "  aX:%d  aY:%d\n", static_cast<int>(n[1]), static_cast<int>(n[2]));
      if (n[0] == 0) return appendf(buffer, capacity, length, "  tX:%d  tY:%d\n", static_cast<int>(n[1]), static_cast<int>(n[2]));
      return true;
    case 27: return appendf(buffer, capacity, length, "  aX:%d  aY:%d  aXF:%.2f  aYF:%.2f\n", static_cast<int>(n[0]), static_cast<int>(n[1]), v[0], v[1]);
    case 28: return appendf(buffer, capacity, length, "  P:%.2f  R:%.5f  H:%.5f  S:%.2f  vra:%.2f  vra:%.2f\n", v[0], v[1], v[2], v[3], v[4], v[5]);
    case 29: return appendf(buffer, capacity, length, " Kp:%.6f Ki:%.6f Kd:%.6f deriv:%.2f out:%.2f\n", v[0], v[1], v[2], v[3], v[4]);
    case 30: return appendf(buffer, capacity, length, " deriv:%.2f\n", v[0]);
    case 31: return appendf(buffer, capacity, length, " E:%.6f it:%.5f il:%.5f oI:%.5f out:%.5f\n", v[0], v[1], v[2], v[3], v[4]);
    case 32: return appendf(buffer, capacity, length, " RP:%.6f RI:%.6f RD:%.6f\n", v[0], v[1], v[2]);
    case 33: return appendf(buffer, capacity, length, " it:%.5f il:%.5f oI:%.5f A:%.5f out:%.5f\n", v[0], v[1], v[2], v[3], v[4]);
    case 34: return appendf(buffer, capacity, length, " Kp:%.6f Ki:%.6f Kd:%.6f deriv:%.2f out:%.2f\n", v[0], v[1], v[2], v[3], v[4]);
    case 35: return appendf(buffer, capacity, length, " deriv:%.2f\n", v[0]);
    case 36: return appendf(buffer, capacity, length, " state:%d start:%d\n", static_cast<int>(n[0]), static_cast<int>(n[1]));
    case 37: return appendf(buffer, capacity, length, " sbus_vra:%.2f sbus_vraf:%.2f sbus_vrb:%.2f sbus_vrbf:%.2f\n", v[0], v[1], v[2], v[3]);
    case 38: return appendf(buffer, capacity, length, " X OUT:%.2f Y OUT:%.2f\n", v[0], v[1]);
    case 39: return appendf(buffer, capacity, length, " it:%.5f il:%.5f oI:%.5f out:%.5f\n", v[0], v[1], v[2], v[3]);
    case 40:
      if (n[0] == 1) return appendf(buffer, capacity, length, "  aX:%d  aY:%d\n", static_cast<int>(n[1]), static_cast<int>(n[2]));
      if (n[0] == 0) return appendf(buffer, capacity, length, "  tX:%d  tX:%d\n", static_cast<int>(n[1]), static_cast<int>(n[2]));
      return true;
    case 41: return appendf(buffer, capacity, length, " roll_ok:%.5f pa:%.5f out:%.5f\n", v[0], v[1], v[2]);
    case 42: return appendf(buffer, capacity, length, " P:%.5f P1:%.5f P3:%.5f\n", v[0], v[1], v[2]);
    case 43: return appendf(buffer, capacity, length, " Vdat:%d Vdatf:%.2f V:%.5f\n", static_cast<int>(n[0]), v[0], v[1]);
    case 44: return appendf(buffer, capacity, length, " PidParameterTuning:%d TargetLegLength:%.6f\n", static_cast<int>(n[0]), v[0]);
    case 45: return appendf(buffer, capacity, length, " RobotTumble:%d roll_ok:%.6f Angle_Pid.error:%.6f\n", static_cast<int>(n[0]), v[0], v[1]);
    case 55: return appendf(buffer, capacity, length, "TRACE,%lu,%d,%d,%.3f,%.3f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f\n", static_cast<unsigned long>(row.timestampMs), static_cast<int>(n[0]), static_cast<int>(n[1]), v[0], v[1], v[2], v[3], static_cast<int>(n[2]), static_cast<int>(n[3]), static_cast<int>(n[4]), static_cast<int>(n[5]), static_cast<int>(n[6]), static_cast<int>(n[7]), static_cast<int>(n[8]), static_cast<int>(n[9]), v[4], v[5]);
    case 56: return appendf(buffer, capacity, length, "CTRL,%lu,%d,%.3f,%.3f,%.2f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n", static_cast<unsigned long>(row.timestampMs), static_cast<int>(n[0]), v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10], static_cast<int>(n[1]));
    case 57: return appendf(buffer, capacity, length, "BAL,%lu,%d,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.2f,%.2f,%d\n", static_cast<unsigned long>(row.timestampMs), static_cast<int>(n[0]), v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], static_cast<int>(n[1]));
    case 58: return appendf(buffer, capacity, length, "DRIVE,%lu,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d\n", static_cast<unsigned long>(row.timestampMs), static_cast<int>(n[0]), v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10], v[11], v[12], v[13], v[14], v[15], v[16], v[17], v[18], v[19], v[20], v[21], v[22], v[23], v[24], v[25], static_cast<int>(n[1]));
    default: return true;
  }
}

bool formatRecord(char *buffer, size_t capacity, size_t &length, const Record &record) {
  if (record.kind == RecordKind::SelectedDebug)
    return formatSelected(buffer, capacity, length, record.selected);

  for (size_t i = 0; i < kChannelCount; ++i) {
    if (i != 0) buffer[length++] = ',';
    if (!append(buffer, capacity, length, "%.6f", record.diagnostic.values[i])) return false;
  }
  buffer[length++] = ',';
  if (!append(buffer, capacity, length, "%.6f",
              static_cast<double>(record.diagnostic.timestampUs) / 1000000.0)) return false;
  buffer[length++] = '\n';
  return true;
}

void telemetrySenderTask(void *) {
  for (;;) {
    if (!sendOne(Serial)) vTaskDelay(1);
  }
}

}  // namespace

const ChannelInfo &channelInfo(Channel channel) {
  return kChannels[static_cast<size_t>(channel)];
}

bool enqueueDiagnostic(const Frame &frame) {
  Record record;
  record.kind = RecordKind::Diagnostic;
  record.diagnostic = frame;
  return enqueueRecord(record);
}

bool enqueueSelected(const SelectedDebug &selected) {
  Record record;
  record.kind = RecordKind::SelectedDebug;
  record.selected = selected;
  return enqueueRecord(record);
}

bool sendOne(Print &sink) {
  Record record;
  if (gQueueHandle == nullptr || xQueueReceive(gQueueHandle, &record, 0) != pdPASS)
    return false;

  char buffer[kFrameBufferSize];
  size_t length = 0;
  if (!formatRecord(buffer, sizeof(buffer), length, record)) {
    latchFailure(IncompleteReason::SenderWriteFailed);
    return true;
  }
  if (length == 0) return true;
  if (sink.write(reinterpret_cast<const uint8_t *>(buffer), length) != length)
    latchFailure(IncompleteReason::SenderWriteFailed);
  return true;
}

bool startSender() {
  if (gSenderStarted) return true;
  if (gQueueHandle == nullptr)
    gQueueHandle = xQueueCreateStatic(kQueueCapacity, sizeof(Record), gQueueStorage,
                                      &gQueueControl);
  if (gQueueHandle == nullptr) {
    latchFailure(IncompleteReason::SenderStartFailed);
    return false;
  }
  gSenderStarted = xTaskCreatePinnedToCore(telemetrySenderTask, "telemetry_sender", 4096,
                                           nullptr, 0, nullptr, 0) == pdPASS;
  if (!gSenderStarted) latchFailure(IncompleteReason::SenderStartFailed);
  return gSenderStarted;
}

IncompleteReason incompleteReason() {
  portENTER_CRITICAL(&gStateMux);
  const uint32_t failure = gFailure;
  portEXIT_CRITICAL(&gStateMux);
  return static_cast<IncompleteReason>(failure);
}

uint8_t rejectedCount() {
  portENTER_CRITICAL(&gStateMux);
  const uint32_t rejected = gRejected;
  portEXIT_CRITICAL(&gStateMux);
  return static_cast<uint8_t>(rejected);
}

}  // namespace Telemetry
