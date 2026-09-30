#include "WifiTuning.h"
#if WIFI_TUNING_ENABLE || WIFI_RECORDING_ENABLE
#include "TelemetryRecorder.h"
#include <WiFi.h>
#include <WiFiProv.h>
#include <ArduinoJson.h>
#include <esp_system.h>
#include <esp_mac.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <math.h>
#include <atomic>
#include <new>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <lwip/sockets.h>
#include <fcntl.h>
#include <errno.h>


namespace {
constexpr size_t kQueueCapacity = 4;
constexpr size_t kBodyMax = 2048;
constexpr size_t kHeaderMax = 1536;
constexpr uint32_t kRequestTtlMs = 700;
constexpr uint32_t kReplyWaitMs = 850;
constexpr uint32_t kHttpReadTimeoutMs = 1000;
#if WIFI_RECORDING_ENABLE
enum RequestKind : uint8_t { STATUS_REQUEST, PARAMETERS_REQUEST, WRITE_REQUEST,
  RECORD_STATUS_REQUEST, RECORD_PREPARE_REQUEST,
  RECORD_START_REQUEST, RECORD_STOP_REQUEST, RECORD_ACK_REQUEST, RECORD_RELEASE_REQUEST };
#else
enum RequestKind : uint8_t { STATUS_REQUEST, PARAMETERS_REQUEST, WRITE_REQUEST };
#endif
struct Request {
  RequestKind kind;
  uint32_t sequence;
  char requestId[41];
  char expectedBoot[17];
  uint16_t bodyLength;
  char body[kBodyMax+1];
  uint32_t receivedAt;
  uint8_t count;
  TuningValue values[TUNING_PARAMETER_COUNT];
  char names[TUNING_PARAMETER_COUNT][3];
#if WIFI_RECORDING_ENABLE
  uint64_t recordingId;
  uint32_t durationSeconds;
  uint32_t receivedRecords;
  uint32_t recordsCrc;
  char profile[32];
#endif
};
struct Reply {
  RequestKind kind;
  uint32_t sequence;
  bool ok;
  char error[32];
  char requestId[41];
  char bootId[17];
  bool supportedMode;
  bool rcValid;
  uint32_t rcAgeMs;
  bool ch5Off;
  float tuningMode;
  uint8_t count;
  char names[TUNING_PARAMETER_COUNT][3];
  float values[TUNING_PARAMETER_COUNT];
#if WIFI_RECORDING_ENABLE
  char recorderState[16];
  char failure[24];
  char recordingId[17];
  char streamTicket[33];
  uint32_t durationSeconds;
  uint32_t generatedRecords;
  uint32_t queuedRecords;
  uint32_t sentRecords;
  uint32_t ringCapacity;
  uint32_t ringUse;
  uint32_t ringHighWater;
  uint16_t dataPort;
  bool streamReady;
  uint64_t startUs;
  uint32_t recordsCrc;
#endif
};
QueueHandle_t requestQueue = nullptr;
QueueHandle_t replyQueue = nullptr;
WiFiServer server(80);
#if WIFI_RECORDING_ENABLE
constexpr uint16_t kRecordingPort = 8766;
WiFiServer recordingServer(kRecordingPort);
TaskHandle_t recordingServerTaskHandle = nullptr;
TaskHandle_t recordingSenderTaskHandle = nullptr;
telemetry::Session recordingSession;
telemetry::RecordRing recordingRing;
uint8_t *recordingStorage = nullptr;
std::atomic<uint8_t> recordingState((uint8_t)telemetry::SessionState::Idle);
std::atomic<bool> streamReady(false), senderRunning(false), streamHandshakeBusy(false);
std::atomic<uint32_t> generatedRecords(0), queuedRecords(0), sentRecords(0);
std::atomic<uint32_t> recordsCrc(0), stopReason(0), frameSequence(0);
uint64_t recordingId = 0, recordingStartUs = 0, recordingStopUs = 0;
uint64_t firstRecordTimestampUs = 0, lastRecordTimestampUs = 0, intervalSumUs = 0;
uint32_t firstRecordSequence = 0, lastRecordSequence = 0;
uint32_t intervalCount = 0, intervalMinUs = UINT32_MAX, intervalMaxUs = 0;
uint32_t recordingDurationSeconds = 0;
uint32_t prepareDeadlineMs = 0, drainDeadlineMs = 0, ackDeadlineMs = 0;
char recordingTicket[33] = {};
char recordingBootId[17] = {};
struct MetadataSnapshot {
  uint32_t durationSeconds = 0;
  uint32_t imuOdrHz = 0;
  uint32_t filterCutoffHz = 0;
  uint32_t bufferCapacity = 512;
  uint64_t startUs = 0;
  bool calibrationInactive = false;
  float rollZeroBiasDeg = 0;
  float pitchZeroBiasDeg = 0;
  float gyroBiasXCounts = 0;
  float gyroBiasYCounts = 0;
  float gyroBiasZCounts = 0;
  bool supportedMode = false;
  bool rcValid = false;
  bool ch5Off = true;
  float tuningMode = 0;
  char names[TUNING_PARAMETER_COUNT][3] = {};
  float values[TUNING_PARAMETER_COUNT] = {};
} metadataSnapshot;
struct CachedRecording { bool valid=false; char id[41]={0}; uint16_t length=0;
  char body[kBodyMax+1]={0}; Reply reply{}; } recordingCache;
#endif
TaskHandle_t wifiTaskHandle = nullptr;
uint32_t bootId = 0;
uint64_t bootId64 = 0;
uint32_t nextSequence = 1;
struct CachedWrite { bool valid=false; char id[41]={0}; uint16_t bodyLength=0; char body[kBodyMax+1]={0}; Reply reply{}; } cache;

String currentBootString() { char out[17]; snprintf(out,sizeof(out),"%016llx",(unsigned long long)bootId64); return String(out); }
const char *reasonPhrase(int status) {
  switch(status){case 200:return "OK";case 400:return "Bad Request";case 404:return "Not Found";case 409:return "Conflict";case 413:return "Payload Too Large";case 503:return "Service Unavailable";case 504:return "Gateway Timeout";default:return "Error";}
}
void sendJson(WiFiClient &client,int status,const String &payload) {
  client.printf("HTTP/1.1 %d %s\r\n",status,reasonPhrase(status));
  client.print("Content-Type: application/json\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Length: ");
  client.print(payload.length());client.print("\r\n\r\n");client.write((const uint8_t *)payload.c_str(),payload.length());
}
void sendError(WiFiClient &client,int status,const char *requestId,const char *code) {
  JsonDocument doc;doc["ok"]=false;if(requestId&&*requestId)doc["request_id"]=requestId;doc["boot_id"]=currentBootString();doc["error"]=code;
  String payload;serializeJson(doc,payload);sendJson(client,status,payload);
}
void sendReply(WiFiClient &client,int status,const Reply &reply) {
  JsonDocument doc;doc["ok"]=reply.ok;doc["api_version"]=1;
  if(reply.requestId[0])doc["request_id"]=reply.requestId;
  doc["boot_id"]=reply.bootId;
  if(!reply.ok)doc["error"]=reply.error;
  if(reply.kind==STATUS_REQUEST||reply.kind==PARAMETERS_REQUEST){
    doc["supported_mode"]=reply.supportedMode;doc["rc_valid"]=reply.rcValid;doc["rc_age_ms"]=reply.rcAgeMs;doc["ch5_off"]=reply.ch5Off;doc["tuning_mode"]=reply.tuningMode;doc["build_id"]="wifi-tuning-phase1";
  }
  if(reply.ok&&(reply.kind==PARAMETERS_REQUEST||reply.kind==WRITE_REQUEST)){
    JsonObject values=doc["values"].to<JsonObject>();for(uint8_t i=0;i<reply.count;i++)values[reply.names[i]]=reply.values[i];
    if(reply.kind==PARAMETERS_REQUEST){
      JsonArray metadata=doc["parameters"].to<JsonArray>();
      for(uint8_t i=0;i<reply.count;i++){const TuningParameter *parameter=findTuningParameter(reply.names[i]);if(!parameter)continue;JsonObject item=metadata.add<JsonObject>();item["name"]=parameter->name;item["min"]=parameter->minimum;item["max"]=parameter->maximum;item["description"]=parameter->description;}
    }
  }
#if WIFI_RECORDING_ENABLE
  if (reply.kind >= RECORD_STATUS_REQUEST) {
    doc["boot_id"] = currentBootString();
    doc["capability"] = true;
    doc["schema"] = "balance_v1";
    doc["state"] = reply.recorderState;
    doc["recording_id"] = reply.recordingId;
    doc["duration_s"] = reply.durationSeconds;
    doc["generated_records"] = reply.generatedRecords;
    doc["queued_records"] = reply.queuedRecords;
    doc["sent_records"] = reply.sentRecords;
    doc["buffer_capacity"] = reply.ringCapacity;
    doc["buffer_use"] = reply.ringUse;
    doc["ring_high_water"] = reply.ringHighWater;
    doc["stream_ready"] = reply.streamReady;
    doc["port"] = reply.dataPort;
    doc["failure"] = reply.failure;
    if (reply.startUs) doc["t_start_us"] = reply.startUs;
    if (reply.kind == RECORD_PREPARE_REQUEST && reply.ok) doc["stream_ticket"] = reply.streamTicket;
  }
#endif
  String payload;serializeJson(doc,payload);sendJson(client,status,payload);
}
bool readLine(WiFiClient &client,char *out,size_t capacity,uint32_t deadline) {
  size_t length=0;
  while((int32_t)(millis()-deadline)<0){
    if(!client.available()){vTaskDelay(pdMS_TO_TICKS(1));continue;}
    int value=client.read();if(value<0)continue;
    if(value=='\n'){if(length&&out[length-1]=='\r')length--;out[length]='\0';return true;}
    if(length+1>=capacity)return false;
    out[length++]=(char)value;
  }
  return false;
}

void fillReply(Reply &reply,const WifiTuningState &state) {
  strlcpy(reply.bootId,currentBootString().c_str(),sizeof(reply.bootId));reply.supportedMode=state.supportedMode;reply.rcValid=state.rcValid;reply.rcAgeMs=state.rcAgeMs;reply.ch5Off=state.ch5Off;reply.tuningMode=state.tuningMode;
  reply.count=TUNING_PARAMETER_COUNT;for(size_t i=0;i<TUNING_PARAMETER_COUNT;i++){const TuningParameter *parameter=tuningParameterAt(i);strlcpy(reply.names[i],parameter->name,sizeof(reply.names[i]));reply.values[i]=parameter->value?*parameter->value:0;}
}

#if WIFI_RECORDING_ENABLE
const char *sessionName(telemetry::SessionState state) {
  switch (state) {
    case telemetry::SessionState::Idle: return "IDLE";
    case telemetry::SessionState::Prepared: return "PREPARED";
    case telemetry::SessionState::Recording: return "RECORDING";
    case telemetry::SessionState::Draining: return "DRAINING";
    case telemetry::SessionState::AwaitAck: return "AWAIT_ACK";
    case telemetry::SessionState::Complete: return "COMPLETE";
    case telemetry::SessionState::Failed: return "FAILED";
    case telemetry::SessionState::Unconfirmed: return "UNCONFIRMED";
  }
  return "FAILED";
}

const char *failureName(uint8_t failure) {
  switch ((telemetry::Failure)failure) {
    case telemetry::Failure::None: return "";
    case telemetry::Failure::BufferFull: return "BUFFER_FULL";
    case telemetry::Failure::ClientDisconnected: return "CLIENT_DISCONNECTED";
    case telemetry::Failure::ConfigChanged: return "CONFIG_CHANGED";
    case telemetry::Failure::SenderError: return "SENDER_ERROR";
    case telemetry::Failure::PrepareExpired: return "PREPARE_EXPIRED";
    case telemetry::Failure::DrainExpired: return "DRAIN_EXPIRED";
  }
  if (failure == 7) return "USER_STOP";
  return "UNKNOWN";
}

String recordingIdString(uint64_t value) {
  char out[17];
  snprintf(out, sizeof(out), "%016llx", (unsigned long long)value);
  return String(out);
}

void fillRecordingReply(Reply &reply) {
  const telemetry::SessionState state = (telemetry::SessionState)recordingState.load(std::memory_order_acquire);
  strlcpy(reply.recorderState, sessionName(state), sizeof(reply.recorderState));
  strlcpy(reply.failure, failureName(stopReason.load(std::memory_order_relaxed)), sizeof(reply.failure));
  strlcpy(reply.recordingId, recordingIdString(recordingId).c_str(), sizeof(reply.recordingId));
  strlcpy(reply.streamTicket, recordingTicket, sizeof(reply.streamTicket));
  reply.durationSeconds = recordingDurationSeconds;
  reply.generatedRecords = generatedRecords.load(std::memory_order_relaxed);
  reply.queuedRecords = queuedRecords.load(std::memory_order_relaxed);
  reply.sentRecords = sentRecords.load(std::memory_order_relaxed);
  reply.ringCapacity = recordingRing.valid() ? 512 : 0;
  reply.ringUse = recordingRing.size();
  reply.ringHighWater = recordingRing.highWater();
  reply.dataPort = kRecordingPort;
  reply.streamReady = streamReady.load(std::memory_order_acquire);
  reply.startUs = recordingStartUs;
  reply.recordsCrc = recordsCrc.load(std::memory_order_relaxed);
}

bool recordingRequestLocked() {
  const telemetry::SessionState state = (telemetry::SessionState)recordingState.load(std::memory_order_acquire);
  return state == telemetry::SessionState::Prepared || state == telemetry::SessionState::Recording ||
         state == telemetry::SessionState::Draining || state == telemetry::SessionState::AwaitAck ||
         (state == telemetry::SessionState::Failed && senderRunning.load(std::memory_order_acquire));
}

bool parseHexId(const char *text, uint64_t &value) {
  if (!text || strlen(text) != 16) return false;
  char *end = nullptr;
  value = strtoull(text, &end, 16);
  return end && *end == '\0';
}

void createTicket() {
  for (unsigned i = 0; i < 4; ++i)
    snprintf(recordingTicket + i * 8, 9, "%08lx", (unsigned long)esp_random());
}

void serviceRecordingTimers() {
  const uint32_t now = millis();
  telemetry::SessionState state = (telemetry::SessionState)recordingState.load(std::memory_order_acquire);
  if (state == telemetry::SessionState::Prepared && (int32_t)(now - prepareDeadlineMs) >= 0) {
    stopReason.store((uint8_t)telemetry::Failure::PrepareExpired, std::memory_order_relaxed);
    recordingSession.failure = telemetry::Failure::PrepareExpired;
    recordingSession.state = telemetry::SessionState::Failed;
    recordingState.store((uint8_t)telemetry::SessionState::Failed, std::memory_order_release);
  } else if (state == telemetry::SessionState::Draining && (int32_t)(now - drainDeadlineMs) >= 0) {
    stopReason.store((uint8_t)telemetry::Failure::DrainExpired, std::memory_order_relaxed);
    recordingSession.failure = telemetry::Failure::DrainExpired;
    recordingSession.state = telemetry::SessionState::Failed;
    recordingState.store((uint8_t)telemetry::SessionState::Failed, std::memory_order_release);
  } else if (state == telemetry::SessionState::AwaitAck && (int32_t)(now - ackDeadlineMs) >= 0) {
    recordingSession.state = telemetry::SessionState::Unconfirmed;
    recordingState.store((uint8_t)telemetry::SessionState::Unconfirmed, std::memory_order_release);
  }
}

bool tuningConfigurationUnchanged(const WifiTuningState &state) {
  if (state.imuOdrHz != metadataSnapshot.imuOdrHz ||
      state.filterCutoffHz != metadataSnapshot.filterCutoffHz ||
      state.calibrationInactive != metadataSnapshot.calibrationInactive ||
      state.rollZeroBiasDeg != metadataSnapshot.rollZeroBiasDeg ||
      state.pitchZeroBiasDeg != metadataSnapshot.pitchZeroBiasDeg ||
      state.gyroBiasXCounts != metadataSnapshot.gyroBiasXCounts ||
      state.gyroBiasYCounts != metadataSnapshot.gyroBiasYCounts ||
      state.gyroBiasZCounts != metadataSnapshot.gyroBiasZCounts) return false;
  for (size_t i = 0; i < TUNING_PARAMETER_COUNT; ++i) {
    const TuningParameter *parameter = tuningParameterAt(i);
    if (!parameter || !parameter->value || !isfinite(*parameter->value) ||
        *parameter->value != metadataSnapshot.values[i]) return false;
  }
  return true;
}

void snapshotMetadata(const WifiTuningState &state, const Reply &reply) {
  metadataSnapshot = MetadataSnapshot{};
  metadataSnapshot.durationSeconds = recordingDurationSeconds;
  metadataSnapshot.startUs = recordingStartUs;
  metadataSnapshot.imuOdrHz = state.imuOdrHz;
  metadataSnapshot.filterCutoffHz = state.filterCutoffHz;
  metadataSnapshot.calibrationInactive = state.calibrationInactive;
  metadataSnapshot.rollZeroBiasDeg = state.rollZeroBiasDeg;
  metadataSnapshot.pitchZeroBiasDeg = state.pitchZeroBiasDeg;
  metadataSnapshot.gyroBiasXCounts = state.gyroBiasXCounts;
  metadataSnapshot.gyroBiasYCounts = state.gyroBiasYCounts;
  metadataSnapshot.gyroBiasZCounts = state.gyroBiasZCounts;
  metadataSnapshot.supportedMode = state.supportedMode;
  metadataSnapshot.rcValid = state.rcValid;
  metadataSnapshot.ch5Off = state.ch5Off;
  metadataSnapshot.tuningMode = state.tuningMode;
  for (size_t i = 0; i < TUNING_PARAMETER_COUNT; ++i) {
    strlcpy(metadataSnapshot.names[i], reply.names[i], sizeof(metadataSnapshot.names[i]));
    metadataSnapshot.values[i] = reply.values[i];
  }
}

bool sendAll(WiFiClient &client, const uint8_t *bytes, size_t length) {
  const int fd = client.fd();
  size_t offset = 0;
  const uint32_t deadline = millis() + 5000;
  while (offset < length && client.connected() && (int32_t)(millis() - deadline) < 0) {
    const int sent = lwip_send(fd, bytes + offset, length - offset, MSG_DONTWAIT);
    if (sent > 0) { offset += (size_t)sent; continue; }
    if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) return false;
    fd_set writeSet;
    FD_ZERO(&writeSet);
    FD_SET(fd, &writeSet);
    timeval timeout{0, 10000};
    const int ready = lwip_select(fd + 1, nullptr, &writeSet, nullptr, &timeout);
    if (ready < 0 && errno != EINTR) return false;
  }
  return offset == length;
}

bool sendFrame(WiFiClient &client, telemetry::FrameType type, uint64_t id, uint32_t sequence,
               const uint8_t *payload, uint32_t payloadLength, uint32_t recordCount) {
  telemetry::FrameHeader header{type, id, sequence, payloadLength, recordCount, 0};
  uint8_t encoded[telemetry::kFrameHeaderSize];
  if (!telemetry::encodeFrameHeader(header, encoded)) return false;
  uint32_t crc = telemetry::crc32(encoded, 28);
  crc = telemetry::crc32(payload, payloadLength, crc);
  header.crc32 = crc;
  if (!telemetry::encodeFrameHeader(header, encoded)) return false;
  return sendAll(client, encoded, sizeof(encoded)) && (!payloadLength || sendAll(client, payload, payloadLength));
}

bool sendMetadata(WiFiClient &client) {
  JsonDocument doc;
  doc["schema"] = "balance_v1";
  doc["boot_id"] = recordingBootId;
  doc["recording_id"] = recordingIdString(recordingId);
  doc["t_start_us"] = metadataSnapshot.startUs;
  doc["requested_duration_s"] = metadataSnapshot.durationSeconds;
  doc["build_id"] = "wifi-recording-v1";
  doc["source_hash_available"] = false;
  doc["build_flags"] = "WIFI_TUNING_ENABLE=1,WIFI_RECORDING_ENABLE=1";
  doc["supported_mode"] = metadataSnapshot.supportedMode;
  doc["manual_tuning"] = metadataSnapshot.tuningMode;
  doc["rc_valid_at_prepare"] = metadataSnapshot.rcValid;
  doc["ch5_off_at_prepare"] = metadataSnapshot.ch5Off;
  doc["imu_odr_hz"] = metadataSnapshot.imuOdrHz;
  doc["filter_cutoff_hz"] = metadataSnapshot.filterCutoffHz;
  doc["calibration_inactive_at_prepare"] = metadataSnapshot.calibrationInactive;
  doc["roll_zero_bias_deg"] = metadataSnapshot.rollZeroBiasDeg;
  doc["pitch_zero_bias_deg"] = metadataSnapshot.pitchZeroBiasDeg;
  JsonArray gyroBias = doc["gyro_bias_counts"].to<JsonArray>();
  gyroBias.add(metadataSnapshot.gyroBiasXCounts);
  gyroBias.add(metadataSnapshot.gyroBiasYCounts);
  gyroBias.add(metadataSnapshot.gyroBiasZCounts);
  doc["effective_gain_notes"] = "Speed PID P/I/D use configured values divided by 100; Angle PID uses configured values directly; Yaw Ki is zero unless roll_mode is AUTO; wheel speed gain is constrained to 0..0.4 and its output to +/-8";
  doc["buffer_capacity_records"] = metadataSnapshot.bufferCapacity;
  doc["voltage_units"] = "existing battery estimate in volts from the GPIO17 divider";
  doc["gyro_units"] = "filtered rad/s";
  doc["roll_ok_units"] = "degrees from quaternionToEuler minus the stored roll zero bias";
  doc["BodyPitching_f_units"] = "degrees from SBUS/Touch controller input and the configured software filter";
  doc["BodyPitching_f_semantics"] = "stored value consumed by angleError=roll_ok-(-BodyPitching_f); recorded without sign inversion";
  doc["motor_targets"] = "commands, not measured currents";
  doc["wheel_speed_units"] = "firmware-native filtered encoder velocity; see build and controller code";
  doc["mode_bits"] = "flags bits 0-1 gain mode, 2 tumble, 3 regulator computed, 4 fresh RC, 5 IMU read time valid, 6 numeric fault, 7-8 roll mode, 9-10 attitude mode, 11-12 posture/mark mode";
  JsonObject configured = doc["configured_gains"].to<JsonObject>();
  for (size_t i = 0; i < TUNING_PARAMETER_COUNT; ++i) configured[metadataSnapshot.names[i]] = metadataSnapshot.values[i];
  JsonArray order = doc["field_order"].to<JsonArray>();
  const char *fields[] = {"roll_ok","BodyPitching_f","gyro_x","gyro_y","gyro_z","MovementSpeed","BodyTurn",
    "Motor1_Velocity_f","Motor2_Velocity_f","Speed_Pid.error","Speed_Pid.outP","Speed_Pid.outI","Speed_Pid.outD",
    "Speed_Pid.output","Angle_Pid.error","Angle_Pid.outP","Angle_Pid.outI","Angle_Pid.outD","Angle_Pid.output",
    "Yaw_Pid.output","BodyX","motor1.target","motor2.target","wheelSpeedFeedbackOutput","Voltage","wheelSpeedFeedbackGain"};
  for (const char *field : fields) order.add(field);
  String payload;
  if (serializeJson(doc, payload) == 0 || payload.length() > 8192) return false;
  return sendFrame(client, telemetry::FrameType::Meta, recordingId, frameSequence++,
                   (const uint8_t *)payload.c_str(), payload.length(), 0);
}

const char *endReason() {
  const uint8_t reason = stopReason.load(std::memory_order_relaxed);
  if (reason == 1) return "BUFFER_FULL";
  if (reason == 2) return "CLIENT_DISCONNECTED";
  if (reason == 3) return "CONFIG_CHANGED";
  if (reason == 4) return "SENDER_ERROR";
  if (reason == 5) return "PREPARE_EXPIRED";
  if (reason == 6) return "DRAIN_EXPIRED";
  if (reason == 7) return "USER_STOP";
  return "DURATION";
}

void recordingSenderTask(void *argument) {
  WiFiClient client = *((WiFiClient *)argument);
  delete (WiFiClient *)argument;
  client.setNoDelay(true);
  while ((telemetry::SessionState)recordingState.load(std::memory_order_acquire) == telemetry::SessionState::Prepared && client.connected())
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
  if ((telemetry::SessionState)recordingState.load(std::memory_order_acquire) != telemetry::SessionState::Recording) {
    if ((telemetry::SessionState)recordingState.load(std::memory_order_relaxed) == telemetry::SessionState::Prepared) {
      stopReason.store((uint8_t)telemetry::Failure::ClientDisconnected, std::memory_order_relaxed);
      recordingState.store((uint8_t)telemetry::SessionState::Failed, std::memory_order_release);
    }
    streamReady.store(false, std::memory_order_release);
    client.stop();
    recordingSenderTaskHandle = nullptr;
    senderRunning.store(false, std::memory_order_release);
    vTaskDelete(nullptr);
    return;
  }
  const int socketFlags = fcntl(client.fd(), F_GETFL, 0);
  if (socketFlags >= 0) fcntl(client.fd(), F_SETFL, socketFlags | O_NONBLOCK);
  frameSequence.store(0, std::memory_order_relaxed);
  recordsCrc.store(0, std::memory_order_relaxed);
  bool transportOk = true;
  if (!sendMetadata(client)) {
    stopReason.store((uint8_t)telemetry::Failure::SenderError, std::memory_order_relaxed);
    recordingState.store((uint8_t)telemetry::SessionState::Failed, std::memory_order_release);
    transportOk = false;
  }
  uint8_t batch[telemetry::kBatchPayloadLimit];
  uint8_t one[telemetry::kRecordSize];
  while (transportOk) {
    uint32_t count = 0;
    while (count < telemetry::kBatchRecordLimit && recordingRing.pop(one)) {
      telemetry::Sample sample{};
      if (telemetry::decodeSample(one, sample)) {
        if (intervalCount == 0) {
          firstRecordTimestampUs = sample.timestampUs;
          firstRecordSequence = sample.sequence;
        }
        lastRecordTimestampUs = sample.timestampUs;
        lastRecordSequence = sample.sequence;
        intervalSumUs += sample.controlDtUs;
        ++intervalCount;
        if (sample.controlDtUs < intervalMinUs) intervalMinUs = sample.controlDtUs;
        if (sample.controlDtUs > intervalMaxUs) intervalMaxUs = sample.controlDtUs;
      }
      memcpy(batch + count * telemetry::kRecordSize, one, telemetry::kRecordSize);
      ++count;
    }
    if (count) {
      const uint32_t bytes = count * telemetry::kRecordSize;
      recordsCrc.store(telemetry::crc32(batch, bytes, recordsCrc.load(std::memory_order_relaxed)), std::memory_order_relaxed);
      if (!sendFrame(client, telemetry::FrameType::Data, recordingId, frameSequence++, batch, bytes, count)) {
        transportOk = false; break;
      }
      sentRecords.fetch_add(count, std::memory_order_relaxed);
      continue;
    }
    const telemetry::SessionState state = (telemetry::SessionState)recordingState.load(std::memory_order_acquire);
    if ((state == telemetry::SessionState::Draining || state == telemetry::SessionState::Failed) && recordingRing.size() == 0) break;
    if (!client.connected()) { transportOk = false; break; }
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
  }
  if (transportOk && client.connected()) {
    JsonDocument doc;
    doc["reason"] = endReason();
    doc["generated_records"] = generatedRecords.load(std::memory_order_relaxed);
    doc["queued_records"] = queuedRecords.load(std::memory_order_relaxed);
    doc["sent_records"] = sentRecords.load(std::memory_order_relaxed);
    doc["first_timestamp_us"] = firstRecordTimestampUs;
    doc["last_timestamp_us"] = lastRecordTimestampUs;
    doc["first_sample_sequence"] = firstRecordSequence;
    doc["last_sample_sequence"] = lastRecordSequence;
    doc["stop_timestamp_us"] = recordingStopUs;
    doc["requested_duration_s"] = recordingDurationSeconds;
    doc["actual_elapsed_us"] = recordingStopUs > recordingStartUs ? recordingStopUs - recordingStartUs : 0;
    doc["ring_high_water"] = recordingRing.highWater();
    doc["overflow_count"] = stopReason.load() == (uint8_t)telemetry::Failure::BufferFull ? 1 : 0;
    doc["records_crc32"] = recordsCrc.load(std::memory_order_relaxed);
    JsonObject intervals = doc["interval_statistics_us"].to<JsonObject>();
    intervals["count"] = intervalCount;
    intervals["min"] = intervalCount ? intervalMinUs : 0;
    intervals["max"] = intervalMaxUs;
    intervals["mean"] = intervalCount ? (double)intervalSumUs / intervalCount : 0;
    String payload;
    serializeJson(doc, payload);
    transportOk = sendFrame(client, telemetry::FrameType::End, recordingId, frameSequence++,
                            (const uint8_t *)payload.c_str(), payload.length(), 0);
  }
  const telemetry::SessionState finalState = (telemetry::SessionState)recordingState.load(std::memory_order_acquire);
  const uint8_t finalReason = stopReason.load(std::memory_order_relaxed);
  if (transportOk && (finalState == telemetry::SessionState::Draining || finalReason == 7 || finalReason == 0)) {
    ackDeadlineMs = millis() + 10000;
    recordingState.store((uint8_t)telemetry::SessionState::AwaitAck, std::memory_order_release);
    while ((telemetry::SessionState)recordingState.load(std::memory_order_acquire) == telemetry::SessionState::AwaitAck && client.connected())
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
  } else if (!transportOk) {
    stopReason.store((uint8_t)telemetry::Failure::ClientDisconnected, std::memory_order_relaxed);
    recordingState.store((uint8_t)telemetry::SessionState::Failed, std::memory_order_release);
  }
  client.stop();
  recordingSenderTaskHandle = nullptr;
  senderRunning.store(false, std::memory_order_release);
  vTaskDelete(nullptr);
}

bool readDataLine(WiFiClient &client, char *out, size_t capacity) {
  size_t length = 0;
  const uint32_t deadline = millis() + 3000;
  while ((int32_t)(millis() - deadline) < 0) {
    if (!client.available()) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
    const int value = client.read();
    if (value < 0) continue;
    if (value == '\n') {
      if (length && out[length - 1] == '\r') --length;
      out[length] = '\0'; return true;
    }
    if (length + 1 >= capacity) return false;
    out[length++] = (char)value;
  }
  return false;
}

void recordingServerTask(void *) {
  recordingServer.begin();
  for (;;) {
    WiFiClient client = recordingServer.accept();
    if (!client) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
    streamHandshakeBusy.store(true, std::memory_order_release);
    char line[513];
    bool valid = !senderRunning.load(std::memory_order_acquire) &&
      (telemetry::SessionState)recordingState.load(std::memory_order_acquire) == telemetry::SessionState::Prepared &&
      readDataLine(client, line, sizeof(line));
    JsonDocument doc;
    if (valid) {
      DeserializationError error = deserializeJson(doc, line, strlen(line), DeserializationOption::NestingLimit(3));
      const char *boot = doc["boot_id"] | "";
      const char *id = doc["recording_id"] | "";
      const char *ticket = doc["ticket"] | "";
      valid = !error && doc.size() == 4 && (doc["protocol"] | 0) == 1 &&
        strcmp(boot, recordingBootId) == 0 && strcmp(ticket, recordingTicket) == 0 &&
        strlen(id) == 16 && strtoull(id, nullptr, 16) == recordingId && !streamReady.load();
    }
    if (!valid) { client.stop(); streamHandshakeBusy.store(false, std::memory_order_release); continue; }
    WiFiClient *ownedClient = new (std::nothrow) WiFiClient(client);
    if (!ownedClient) { client.stop(); streamHandshakeBusy.store(false, std::memory_order_release); continue; }
    senderRunning.store(true, std::memory_order_release);
    if (xTaskCreatePinnedToCore(recordingSenderTask, "recording-sender", 8192, ownedClient, 1,
                                &recordingSenderTaskHandle, 0) != pdPASS) {
      delete ownedClient;
      senderRunning.store(false, std::memory_order_release);
      client.stop(); streamHandshakeBusy.store(false, std::memory_order_release); continue;
    }
    if (client.print("OK\n") != 3) {
      client.stop();
      streamHandshakeBusy.store(false, std::memory_order_release);
      continue;
    }
    streamReady.store(true, std::memory_order_release);
    streamHandshakeBusy.store(false, std::memory_order_release);
  }
}

bool parseRecordingRequest(const char *method, const char *path, const char *body,
                           size_t length, Request &request, WiFiClient &client) {
  if (!strcmp(method, "GET") && !strcmp(path, "/api/v1/recording/status")) {
    request.kind = RECORD_STATUS_REQUEST; return true;
  }
  if (strcmp(method, "POST") || strncmp(path, "/api/v1/recording/", 18) != 0) return false;
  const char *action = path + 18;
  if (!strcmp(action, "prepare")) request.kind = RECORD_PREPARE_REQUEST;
  else if (!strcmp(action, "start")) request.kind = RECORD_START_REQUEST;
  else if (!strcmp(action, "stop")) request.kind = RECORD_STOP_REQUEST;
  else if (!strcmp(action, "ack")) request.kind = RECORD_ACK_REQUEST;
  else if (!strcmp(action, "release")) request.kind = RECORD_RELEASE_REQUEST;
  else return false;
  if (length > 1024 || hasDuplicateJsonObjectKeys(body, length)) {
    sendError(client, length > 1024 ? 413 : 400, "", length > 1024 ? "BODY_TOO_LARGE" : "DUPLICATE_KEY");
    return true;
  }
  JsonDocument doc;
  if (deserializeJson(doc, body, length, DeserializationOption::NestingLimit(3)) || !doc.is<JsonObject>()) {
    sendError(client, 400, "", "INVALID_SCHEMA"); return true;
  }
  const char *requestId = doc["request_id"] | "";
  const char *expectedBoot = doc["expected_boot_id"] | "";
  if (!requestId[0] || strlen(requestId) > 40 || strlen(expectedBoot) != 16) {
    sendError(client, 400, requestId, "INVALID_SCHEMA"); return true;
  }
  strlcpy(request.requestId, requestId, sizeof(request.requestId));
  strlcpy(request.expectedBoot, expectedBoot, sizeof(request.expectedBoot));
  request.bodyLength = (uint16_t)length;
  if (request.kind == RECORD_PREPARE_REQUEST) {
    if (doc.size() != 4 || !doc["duration_s"].is<uint32_t>() ||
        !doc["profile"].is<const char *>() || strlen(doc["profile"] | "") > 31) {
      sendError(client, 400, requestId, "INVALID_SCHEMA"); return true;
    }
    request.durationSeconds = doc["duration_s"].as<uint32_t>();
    if (request.durationSeconds < 20 || request.durationSeconds > 40) {
      sendError(client, 400, requestId, "INVALID_DURATION"); return true;
    }
    strlcpy(request.profile, doc["profile"] | "", sizeof(request.profile));
  } else {
    const char *id = doc["recording_id"] | "";
    if (!parseHexId(id, request.recordingId)) { sendError(client, 400, requestId, "INVALID_SCHEMA"); return true; }
    const size_t expectedSize = request.kind == RECORD_ACK_REQUEST ? 5 : 3;
    if (doc.size() != expectedSize) { sendError(client, 400, requestId, "INVALID_SCHEMA"); return true; }
    if (request.kind == RECORD_ACK_REQUEST) {
      if (!doc["received_records"].is<uint32_t>() || !doc["records_crc32"].is<uint32_t>()) {
        sendError(client, 400, requestId, "INVALID_SCHEMA"); return true;
      }
      request.receivedRecords = doc["received_records"].as<uint32_t>();
      request.recordsCrc = doc["records_crc32"].as<uint32_t>();
    }
  }
  request.receivedAt = millis();
  return true;
}
#endif
bool waitForReply(const Request &request,Reply &reply,bool &enqueued) {
  enqueued=false;
  if(!replyQueue)return false;
  xQueueReset(replyQueue);
  if(xQueueSend(requestQueue,&request,0)!=pdTRUE)return false;
  enqueued=true;
  const TickType_t deadline=xTaskGetTickCount()+pdMS_TO_TICKS(kReplyWaitMs);
  for(;;){
    const TickType_t now=xTaskGetTickCount();if((int32_t)(deadline-now)<=0)return false;
    Reply received{};
    if(xQueueReceive(replyQueue,&received,deadline-now)!=pdTRUE)return false;
    if(received.sequence==request.sequence){reply=received;return true;}
  }
}
void sendQueued(WiFiClient &client,Request &request) {
  if(!requestQueue||!replyQueue){sendError(client,503,request.requestId,"QUEUE_FULL");return;}
  request.sequence=nextSequence++;if(nextSequence==0)nextSequence=1;request.receivedAt=millis();
  Reply reply{};bool enqueued=false;
  if(!waitForReply(request,reply,enqueued)){sendError(client,enqueued?504:503,request.requestId,enqueued?"OUTCOME_UNKNOWN":"QUEUE_FULL");return;}
  if (!reply.ok) {
    const char *error = reply.error;
    const int status = !strcmp(error, "REQUEST_EXPIRED") ? 504 :
      (!strcmp(error, "QUEUE_FULL") || !strcmp(error, "RESOURCE_BUSY") ? 503 :
       (!strcmp(error, "DRIVE_ACTIVE") || !strcmp(error, "RC_UNAVAILABLE") ||
        !strcmp(error, "AUTO_MODE") || !strcmp(error, "BOOT_CHANGED") ||
        !strcmp(error, "UNSUPPORTED_MODE") || !strcmp(error, "UNSUPPORTED_PROFILE") ||
        !strcmp(error, "CALIBRATION_ACTIVE") || !strcmp(error, "REQUEST_ID_CONFLICT") ||
        !strcmp(error, "RECORDING_BUSY") || !strcmp(error, "WRONG_STATE") ||
        !strcmp(error, "WRONG_ID") || !strcmp(error, "ACK_MISMATCH") ||
        !strcmp(error, "STREAM_NOT_READY") || !strcmp(error, "CONFIG_CHANGED") ? 409 : 400));
    sendReply(client, status, reply);
    return;
  }
  sendReply(client,200,reply);
}
void handleWrite(WiFiClient &client,Request &request) {
  const char *body=request.body;const size_t length=request.bodyLength;
  if(hasDuplicateJsonObjectKeys(body,length)){sendError(client,400,"","DUPLICATE_KEY");return;}
  JsonDocument doc;DeserializationError error=deserializeJson(doc,body,length,DeserializationOption::NestingLimit(4));
  if(error||!doc.is<JsonObject>()||doc.size()!=3){sendError(client,400,"","INVALID_SCHEMA");return;}
  const char *requestId=doc["request_id"]|"";const char *expectedBoot=doc["expected_boot_id"]|"";JsonObject values=doc["values"].as<JsonObject>();
  if(!requestId[0]||strlen(requestId)>40||strlen(expectedBoot)!=16||values.isNull()||values.size()==0||values.size()>TUNING_PARAMETER_COUNT){sendError(client,400,requestId,"INVALID_SCHEMA");return;}
  request.kind=WRITE_REQUEST;strlcpy(request.requestId,requestId,sizeof(request.requestId));strlcpy(request.expectedBoot,expectedBoot,sizeof(request.expectedBoot));request.count=values.size();
  uint8_t index=0;
  for(JsonPair item:values){const char *name=item.key().c_str();if(!name||strlen(name)>2||!findTuningParameter(name)||item.value().is<bool>()||!item.value().is<float>()||!isfinite(item.value().as<float>())){sendError(client,400,requestId,"INVALID_VALUE");return;}
    strlcpy(request.names[index],name,sizeof(request.names[index]));request.values[index].value=item.value().as<float>();index++;
  }
  sendQueued(client,request);
}
void handleClient(WiFiClient &client) {
  client.setNoDelay(true);
  const uint32_t headerDeadline=millis()+kHttpReadTimeoutMs;
  char line[256];char method[8]={0},path[96]={0},protocol[16]={0};
  char extraField=0;
  if(!readLine(client,line,sizeof(line),headerDeadline)||sscanf(line,"%7s %95s %15s %c",method,path,protocol,&extraField)!=3||(strcmp(protocol,"HTTP/1.1")&&strcmp(protocol,"HTTP/1.0"))){sendError(client,400,"","INVALID_REQUEST");return;}
  char contentType[64]={0};bool hasContentType=false,hasLength=false,hasTransferEncoding=false;size_t contentLength=0,totalHeaderBytes=0;
  for(;;){
    if(!readLine(client,line,sizeof(line),headerDeadline)){sendError(client,400,"","INVALID_HEADERS");return;}
    totalHeaderBytes+=strlen(line)+2;if(totalHeaderBytes>kHeaderMax){sendError(client,400,"","HEADERS_TOO_LARGE");return;}
    if(!line[0])break;
    char *colon=strchr(line,':');if(!colon){sendError(client,400,"","INVALID_HEADERS");return;}*colon='\0';char *value=colon+1;while(isspace((unsigned char)*value))value++;char *end=value+strlen(value);while(end>value&&isspace((unsigned char)end[-1]))*--end='\0';
    if(!strcasecmp(line,"Content-Type")){if(hasContentType||strlen(value)>=sizeof(contentType)){sendError(client,400,"","INVALID_HEADERS");return;}hasContentType=true;strlcpy(contentType,value,sizeof(contentType));}
    else if(!strcasecmp(line,"Content-Length")){if(hasLength||!value[0]){sendError(client,400,"","INVALID_HEADERS");return;}for(char*p=value;*p;p++)if(!isdigit((unsigned char)*p)){sendError(client,400,"","INVALID_HEADERS");return;}unsigned long parsed=strtoul(value,nullptr,10);if(parsed>0xffffffffUL){sendError(client,400,"","INVALID_HEADERS");return;}contentLength=(size_t)parsed;hasLength=true;}
    else if(!strcasecmp(line,"Transfer-Encoding")){hasTransferEncoding=true;}
  }
  if(strchr(path,'?')){sendError(client,400,"","INVALID_REQUEST");return;}
#if WIFI_RECORDING_ENABLE
  if (!strcmp(method, "GET") && !strcmp(path, "/api/v1/recording/status")) {
    if ((hasLength && contentLength != 0) || hasTransferEncoding) { sendError(client, 400, "", "INVALID_SCHEMA"); return; }
    Request request{}; request.kind = RECORD_STATUS_REQUEST; sendQueued(client, request); return;
  }
  if (!strncmp(path, "/api/v1/recording/", 18)) {
    if (strcmp(method, "POST") || hasTransferEncoding || !hasLength) { sendError(client, 400, "", "INVALID_SCHEMA"); return; }
    if (contentLength > kBodyMax) { sendError(client, 413, "", "BODY_TOO_LARGE"); return; }
    if (!hasContentType || strncasecmp(contentType, "application/json", 16) != 0) { sendError(client, 400, "", "INVALID_SCHEMA"); return; }
    Request request{}; char *body=request.body; size_t received = 0; const uint32_t deadline = millis() + kHttpReadTimeoutMs;
    while (received < contentLength && (int32_t)(millis() - deadline) < 0) {
      if (!client.available()) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
      const int count = client.read((uint8_t *)body + received, contentLength - received);
      if (count > 0) received += (size_t)count;
    }
    if (received != contentLength) { sendError(client, 400, "", "INCOMPLETE_BODY"); return; }
    body[received] = '\0'; request.bodyLength = (uint16_t)received;
    if (!parseRecordingRequest(method, path, body, received, request, client)) { sendError(client, 404, "", "NOT_FOUND"); return; }
    if (!request.requestId[0] && request.kind != RECORD_STATUS_REQUEST) return;
    sendQueued(client, request); return;
  }
#endif
  if(!strcmp(method,"GET")&&(!strcmp(path,"/api/v1/status")||!strcmp(path,"/api/v1/parameters"))){
    if((hasLength&&contentLength!=0)||hasTransferEncoding){sendError(client,400,"","INVALID_SCHEMA");return;}
    Request request{};request.kind=!strcmp(path,"/api/v1/status")?STATUS_REQUEST:PARAMETERS_REQUEST;sendQueued(client,request);return;
  }
  if(strcmp(method,"POST")||strcmp(path,"/api/v1/parameters")){sendError(client,404,"","NOT_FOUND");return;}
  if(hasTransferEncoding||!hasLength){sendError(client,400,"","INVALID_SCHEMA");return;}
  if(contentLength>kBodyMax){sendError(client,413,"","BODY_TOO_LARGE");return;}
  if(!hasContentType||strncasecmp(contentType,"application/json",16)!=0||(contentType[16]&&contentType[16]!=';'&&!isspace((unsigned char)contentType[16]))){sendError(client,400,"","INVALID_SCHEMA");return;}
  Request request{};char *body=request.body;size_t received=0;const uint32_t bodyDeadline=millis()+kHttpReadTimeoutMs;
  while(received<contentLength&&(int32_t)(millis()-bodyDeadline)<0){if(!client.available()){vTaskDelay(pdMS_TO_TICKS(1));continue;}int count=client.read((uint8_t*)body+received,contentLength-received);if(count>0)received+=(size_t)count;}
  if(received!=contentLength){sendError(client,400,"","INCOMPLETE_BODY");return;}
  body[received]='\0';request.bodyLength=(uint16_t)received;handleWrite(client,request);
}
void wifiTask(void*) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char serviceName[16];
  snprintf(serviceName, sizeof(serviceName), "PROV_%02X%02X%02X", mac[3], mac[4], mac[5]);
  WiFiProv.beginProvision(NETWORK_PROV_SCHEME_SOFTAP,
                          NETWORK_PROV_SCHEME_HANDLER_NONE,
                          NETWORK_PROV_SECURITY_0,
                          nullptr, serviceName, nullptr, nullptr, false);
  Serial.printf("Wi-Fi provisioning service initialized (AP name: %s).\n", serviceName);
  bool serverStarted=false;
  bool wasConnected=false;
  for(;;){
    if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
      Serial.println("Clearing saved Wi-Fi settings and restarting.");
      WiFiProv.endProvision();
      WiFi.mode(WIFI_STA);
      if (WiFi.STA.erase()) ESP.restart();
      Serial.println("Wi-Fi reset failed; settings were not cleared.");
    }
    const bool connected = WiFi.status() == WL_CONNECTED;
    if (connected && !wasConnected) Serial.println("Wi-Fi connected.");
    if (!connected && wasConnected) Serial.println("Wi-Fi disconnected; reconnecting.");
    wasConnected = connected;
    if (connected && requestQueue && replyQueue && !serverStarted) {
      server.begin();
      serverStarted = true;
      Serial.println("Wi-Fi tuning API started.");
#if WIFI_RECORDING_ENABLE
      if (xTaskCreatePinnedToCore(recordingServerTask, "recording-listener", 4096, nullptr, 1,
                                  &recordingServerTaskHandle, 0) != pdPASS)
        Serial.println("Recording listener failed to start.");
#endif
    }
    if (serverStarted) {
      WiFiClient client=server.accept();
      if(client){handleClient(client);client.stop();}
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}
}

void WifiTuningBegin() {
  requestQueue=xQueueCreate(kQueueCapacity,sizeof(Request));
  replyQueue=xQueueCreate(1,sizeof(Reply));
  if (!requestQueue || !replyQueue) Serial.println("Wi-Fi tuning API queues failed to initialize.");
  bootId=esp_random();
  bootId64=((uint64_t)esp_random()<<32)|esp_random();
#if WIFI_RECORDING_ENABLE
  snprintf(recordingBootId, sizeof(recordingBootId), "%016llx", (unsigned long long)bootId64);
  recordingStorage=(uint8_t *)heap_caps_malloc(512 * telemetry::kRecordSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (recordingStorage) recordingRing.bind(recordingStorage);
#endif
  if (xTaskCreatePinnedToCore(wifiTask,"wifi-tuning",8192,nullptr,1,&wifiTaskHandle,0) != pdPASS) {
    wifiTaskHandle=nullptr;
    Serial.println("Wi-Fi task failed to start.");
  }
}
void WifiTuningReprovision(char *cmd) {
  if (!cmd || strcmp(cmd, "RESET") != 0) {
    Serial.println("Use WRESET to clear saved Wi-Fi settings.");
    return;
  }
  if (!wifiTaskHandle) {
    Serial.println("Wi-Fi task is not ready.");
    return;
  }
  xTaskNotifyGive(wifiTaskHandle);
  Serial.println("Wi-Fi reset requested.");
}
#if WIFI_RECORDING_ENABLE
bool WifiTuningRecordingActive() {
  const telemetry::SessionState state = (telemetry::SessionState)recordingState.load(std::memory_order_acquire);
  return state == telemetry::SessionState::Recording || state == telemetry::SessionState::Draining ||
         (state == telemetry::SessionState::Failed && senderRunning.load(std::memory_order_acquire));
}

bool WifiTuningMutationLocked() { return recordingRequestLocked(); }

void WifiTuningRecordingTick(const telemetry::Sample &input, const WifiTuningState &state) {
  if ((telemetry::SessionState)recordingState.load(std::memory_order_acquire) != telemetry::SessionState::Recording) return;
  telemetry::Sample sample = input;
  sample.sequence = generatedRecords.load(std::memory_order_relaxed);
  if (telemetry::durationElapsed(recordingSession, sample.timestampUs)) {
    recordingStopUs = sample.timestampUs;
    stopReason.store(0, std::memory_order_relaxed);
    recordingSession.stopUs = recordingStopUs;
    recordingSession.state = telemetry::SessionState::Draining;
    drainDeadlineMs = millis() + 5000;
    recordingState.store((uint8_t)telemetry::SessionState::Draining, std::memory_order_release);
    if (recordingSenderTaskHandle) xTaskNotifyGive(recordingSenderTaskHandle);
    return;
  }
  if (!tuningConfigurationUnchanged(state)) {
    stopReason.store((uint8_t)telemetry::Failure::ConfigChanged, std::memory_order_relaxed);
    recordingStopUs = sample.timestampUs;
    recordingSession.failure = telemetry::Failure::ConfigChanged;
    recordingSession.stopUs = recordingStopUs;
    recordingSession.state = telemetry::SessionState::Failed;
    drainDeadlineMs = millis() + 5000;
    recordingState.store((uint8_t)telemetry::SessionState::Failed, std::memory_order_release);
    if (recordingSenderTaskHandle) xTaskNotifyGive(recordingSenderTaskHandle);
    return;
  }
  uint8_t encoded[telemetry::kRecordSize];
  if (!telemetry::encodeSample(sample, encoded)) {
    stopReason.store((uint8_t)telemetry::Failure::SenderError, std::memory_order_relaxed);
    recordingStopUs = sample.timestampUs;
    drainDeadlineMs = millis() + 5000;
    recordingSession.failure = telemetry::Failure::SenderError;
    recordingSession.stopUs = recordingStopUs;
    recordingSession.state = telemetry::SessionState::Failed;
    recordingState.store((uint8_t)telemetry::SessionState::Failed, std::memory_order_release);
    if (recordingSenderTaskHandle) xTaskNotifyGive(recordingSenderTaskHandle);
    return;
  }
  generatedRecords.fetch_add(1, std::memory_order_relaxed);
  if (!recordingRing.push(encoded)) {
    stopReason.store((uint8_t)telemetry::Failure::BufferFull, std::memory_order_relaxed);
    recordingStopUs = sample.timestampUs;
    drainDeadlineMs = millis() + 5000;
    recordingSession.failure = telemetry::Failure::BufferFull;
    recordingSession.stopUs = recordingStopUs;
    recordingSession.state = telemetry::SessionState::Failed;
    recordingState.store((uint8_t)telemetry::SessionState::Failed, std::memory_order_release);
    if (recordingSenderTaskHandle) xTaskNotifyGive(recordingSenderTaskHandle);
    return;
  }
  queuedRecords.fetch_add(1, std::memory_order_relaxed);
  if (recordingSenderTaskHandle) xTaskNotifyGive(recordingSenderTaskHandle);
}
#else
bool WifiTuningRecordingActive() { return false; }
bool WifiTuningMutationLocked() { return false; }
void WifiTuningRecordingTick(const telemetry::Sample &, const WifiTuningState &) {}
#endif

void WifiTuningProcessOne(const WifiTuningState &state) {
#if WIFI_RECORDING_ENABLE
  serviceRecordingTimers();
  if (streamReady.load(std::memory_order_acquire) && recordingSession.state == telemetry::SessionState::Prepared)
    telemetry::acceptReady(recordingSession, recordingId);
  const telemetry::SessionState observed = (telemetry::SessionState)recordingState.load(std::memory_order_acquire);
  if (observed == telemetry::SessionState::AwaitAck || observed == telemetry::SessionState::Unconfirmed)
    recordingSession.state = observed;
  else if (observed == telemetry::SessionState::Failed && recordingSession.state != telemetry::SessionState::Failed) {
    const uint8_t reason = stopReason.load(std::memory_order_relaxed);
    telemetry::fail(recordingSession, reason <= (uint8_t)telemetry::Failure::DrainExpired ?
                    (telemetry::Failure)reason : telemetry::Failure::SenderError);
  }
#endif
  if (!requestQueue || !replyQueue) return;
  Request request{};
  if (xQueueReceive(requestQueue, &request, 0) != pdTRUE) return;
  Reply reply{};
  reply.kind = request.kind;
  reply.sequence = request.sequence;
  fillReply(reply, state);
  reply.ok = true;
  strlcpy(reply.requestId, request.requestId, sizeof(reply.requestId));
  for (uint8_t i = 0; i < request.count; ++i) request.values[i].name = request.names[i];
  if ((uint32_t)(millis() - request.receivedAt) > kRequestTtlMs) {
    reply.ok = false; strlcpy(reply.error, "REQUEST_EXPIRED", sizeof(reply.error));
  } else if (request.kind == WRITE_REQUEST) {
    if (cache.valid && strcmp(cache.id, request.requestId) == 0) {
      if (cache.bodyLength == request.bodyLength && memcmp(cache.body, request.body, request.bodyLength) == 0) {
        reply = cache.reply; reply.sequence = request.sequence;
      } else { reply.ok = false; strlcpy(reply.error, "REQUEST_ID_CONFLICT", sizeof(reply.error)); }
    } else if (strcmp(request.expectedBoot, currentBootString().c_str()) != 0) {
      reply.ok = false; strlcpy(reply.error, "BOOT_CHANGED", sizeof(reply.error));
    } else if (WifiTuningMutationLocked()) {
      reply.ok = false; strlcpy(reply.error, "RECORDING_BUSY", sizeof(reply.error));
    } else {
      const char *policyError = validateTuningWritePolicy(state.supportedMode, state.rcValid,
        state.ch5Off, request.values, request.count, state.tuningMode);
      if (policyError) { reply.ok = false; strlcpy(reply.error, policyError, sizeof(reply.error)); }
      else {
        for (uint8_t i = 0; i < request.count; ++i) {
          const TuningParameter *parameter = findTuningParameter(request.values[i].name);
          if (parameter && parameter->value) *parameter->value = request.values[i].value;
        }
        fillReply(reply, state);
      }
      cache.valid = true; strlcpy(cache.id, request.requestId, sizeof(cache.id));
      cache.bodyLength = request.bodyLength; memcpy(cache.body, request.body, request.bodyLength);
      cache.body[request.bodyLength] = '\0'; cache.reply = reply;
    }
#if WIFI_RECORDING_ENABLE
  } else if (request.kind >= RECORD_STATUS_REQUEST) {
    const bool mutation = request.kind != RECORD_STATUS_REQUEST;
    bool replayed = false;
    if (mutation && recordingCache.valid && strcmp(recordingCache.id, request.requestId) == 0) {
      if (recordingCache.length == request.bodyLength && memcmp(recordingCache.body, request.body, request.bodyLength) == 0) {
        reply = recordingCache.reply; reply.sequence = request.sequence; replayed = true;
      } else { reply.ok = false; strlcpy(reply.error, "REQUEST_ID_CONFLICT", sizeof(reply.error)); }
    }
    if (mutation && !replayed && reply.ok && strcmp(request.expectedBoot, currentBootString().c_str()) != 0) {
      reply.ok = false; strlcpy(reply.error, "BOOT_CHANGED", sizeof(reply.error));
    }
    if (!replayed && reply.ok) {
      const telemetry::SessionState current = (telemetry::SessionState)recordingState.load(std::memory_order_acquire);
      if (request.kind == RECORD_STATUS_REQUEST) {
        fillRecordingReply(reply);
      } else if (request.kind == RECORD_PREPARE_REQUEST) {
        if (strcmp(request.profile, "balance_v1") != 0) { reply.ok = false; strlcpy(reply.error, "UNSUPPORTED_PROFILE", sizeof(reply.error)); }
        else if (!state.supportedMode) { reply.ok = false; strlcpy(reply.error, "UNSUPPORTED_MODE", sizeof(reply.error)); }
        else if (state.tuningMode != 1.0f) { reply.ok = false; strlcpy(reply.error, "AUTO_MODE", sizeof(reply.error)); }
        else if (!state.calibrationInactive) { reply.ok = false; strlcpy(reply.error, "CALIBRATION_ACTIVE", sizeof(reply.error)); }
        else if (!recordingRing.valid() || senderRunning.load(std::memory_order_acquire) ||
                 streamHandshakeBusy.load(std::memory_order_acquire)) { reply.ok = false; strlcpy(reply.error, "RESOURCE_BUSY", sizeof(reply.error)); }
        else if (!telemetry::acceptPrepare(recordingSession, ((uint64_t)esp_random() << 32) | esp_random(), request.durationSeconds)) {
          reply.ok = false; strlcpy(reply.error, "WRONG_STATE", sizeof(reply.error));
        } else {
          recordingId = recordingSession.recordingId;
          recordingDurationSeconds = request.durationSeconds;
          streamReady.store(false, std::memory_order_release);
          createTicket();
          strlcpy(recordingBootId, currentBootString().c_str(), sizeof(recordingBootId));
          recordingRing.reset();
          generatedRecords.store(0); queuedRecords.store(0); sentRecords.store(0);
          recordsCrc.store(0); frameSequence.store(0); stopReason.store(0);
          recordingStartUs = recordingStopUs = 0;
          firstRecordTimestampUs = lastRecordTimestampUs = intervalSumUs = 0;
          firstRecordSequence = lastRecordSequence = 0;
          intervalCount = intervalMaxUs = 0; intervalMinUs = UINT32_MAX;
          prepareDeadlineMs = millis() + 15000;
          snapshotMetadata(state, reply);
          recordingState.store((uint8_t)telemetry::SessionState::Prepared, std::memory_order_release);
          fillRecordingReply(reply);
        }
      } else if (request.kind == RECORD_START_REQUEST) {
        if (request.recordingId != recordingId) { reply.ok = false; strlcpy(reply.error, "WRONG_ID", sizeof(reply.error)); }
        else if (current == telemetry::SessionState::Recording || current == telemetry::SessionState::Draining || current == telemetry::SessionState::AwaitAck) {
          fillRecordingReply(reply);
        } else if (!telemetry::acceptStart(recordingSession, recordingId, esp_timer_get_time())) {
          reply.ok = false; strlcpy(reply.error, streamReady.load() ? "WRONG_STATE" : "STREAM_NOT_READY", sizeof(reply.error));
        } else {
          recordingStartUs = recordingSession.startUs;
          metadataSnapshot.startUs = recordingStartUs;
          recordingState.store((uint8_t)telemetry::SessionState::Recording, std::memory_order_release);
          if (recordingSenderTaskHandle) xTaskNotifyGive(recordingSenderTaskHandle);
          fillRecordingReply(reply);
        }
      } else if (request.kind == RECORD_STOP_REQUEST) {
        if (!telemetry::acceptStop(recordingSession, request.recordingId, esp_timer_get_time())) {
          reply.ok = false; strlcpy(reply.error, request.recordingId == recordingId ? "WRONG_STATE" : "WRONG_ID", sizeof(reply.error));
        } else {
          if (recordingSession.state == telemetry::SessionState::Draining) {
            stopReason.store(7); recordingStopUs = recordingSession.stopUs;
            drainDeadlineMs = millis() + 5000;
            recordingState.store((uint8_t)telemetry::SessionState::Draining, std::memory_order_release);
          } else {
            streamReady.store(false, std::memory_order_release);
            recordingState.store((uint8_t)recordingSession.state, std::memory_order_release);
          }
          if (recordingSenderTaskHandle) xTaskNotifyGive(recordingSenderTaskHandle);
          fillRecordingReply(reply);
        }
      } else if (request.kind == RECORD_ACK_REQUEST) {
        const uint32_t sent = sentRecords.load(std::memory_order_acquire);
        const uint32_t crc = recordsCrc.load(std::memory_order_acquire);
        if (current == telemetry::SessionState::AwaitAck && recordingSession.state == telemetry::SessionState::Draining)
          recordingSession.state = telemetry::SessionState::AwaitAck;
        if (request.recordingId != recordingId || !telemetry::acceptAck(recordingSession, request.recordingId,
              request.receivedRecords, request.recordsCrc, sent, crc)) {
          reply.ok = false; strlcpy(reply.error, "ACK_MISMATCH", sizeof(reply.error));
        } else {
          recordingState.store((uint8_t)telemetry::SessionState::Complete, std::memory_order_release);
          if (recordingSenderTaskHandle) xTaskNotifyGive(recordingSenderTaskHandle);
          fillRecordingReply(reply);
        }
      } else if (request.kind == RECORD_RELEASE_REQUEST) {
        if (request.recordingId != recordingId || !telemetry::canPrepare(recordingSession) ||
            senderRunning.load() || streamHandshakeBusy.load()) {
          reply.ok = false; strlcpy(reply.error, "WRONG_STATE", sizeof(reply.error));
        } else {
          recordingSession = telemetry::Session{};
          recordingState.store((uint8_t)telemetry::SessionState::Idle, std::memory_order_release);
          recordingId = 0; recordingDurationSeconds = 0;
          streamReady.store(false, std::memory_order_release);
          fillRecordingReply(reply);
        }
      }
    }
    if (mutation && !(recordingCache.valid && strcmp(recordingCache.id, request.requestId) == 0)) {
      recordingCache.valid = true; strlcpy(recordingCache.id, request.requestId, sizeof(recordingCache.id));
      recordingCache.length = request.bodyLength; memcpy(recordingCache.body, request.body, request.bodyLength);
      recordingCache.body[request.bodyLength] = '\0'; recordingCache.reply = reply;
    }
#endif
  }
  xQueueSend(replyQueue, &reply, 0);
}
#else
void WifiTuningBegin() {}
void WifiTuningReprovision(char *) {}
void WifiTuningProcessOne(const WifiTuningState &) {}
bool WifiTuningRecordingActive() { return false; }
bool WifiTuningMutationLocked() { return false; }
void WifiTuningRecordingTick(const telemetry::Sample &, const WifiTuningState &) {}
#endif
