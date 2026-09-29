#include "WifiTuning.h"
#if WIFI_TUNING_ENABLE
#include <WiFi.h>
#include <WiFiProv.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <esp_system.h>
#include <esp_mac.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <math.h>


namespace {
constexpr size_t kQueueCapacity = 4;
constexpr size_t kTuningTokenCapacity = 129;
constexpr size_t kBodyMax = 2048;
constexpr size_t kHeaderMax = 1536;
constexpr uint32_t kRequestTtlMs = 700;
constexpr uint32_t kReplyWaitMs = 850;
constexpr uint32_t kHttpReadTimeoutMs = 1000;
enum RequestKind : uint8_t { STATUS_REQUEST, PARAMETERS_REQUEST, WRITE_REQUEST };
struct Request {
  RequestKind kind;
  uint32_t sequence;
  char requestId[41];
  char expectedBoot[9];
  uint16_t bodyLength;
  char body[kBodyMax+1];
  uint32_t receivedAt;
  uint8_t count;
  TuningValue values[TUNING_PARAMETER_COUNT];
  char names[TUNING_PARAMETER_COUNT][3];
};
struct Reply {
  RequestKind kind;
  uint32_t sequence;
  bool ok;
  char error[32];
  char requestId[41];
  char bootId[9];
  bool supportedMode;
  bool rcValid;
  uint32_t rcAgeMs;
  bool ch5Off;
  float tuningMode;
  uint8_t count;
  char names[TUNING_PARAMETER_COUNT][3];
  float values[TUNING_PARAMETER_COUNT];
};
QueueHandle_t requestQueue = nullptr;
QueueHandle_t replyQueue = nullptr;
WiFiServer server(80);
TaskHandle_t wifiTaskHandle = nullptr;
char tuningToken[kTuningTokenCapacity] = {};
bool tuningTokenAvailable = false;
uint32_t bootId = 0;
uint32_t nextSequence = 1;
struct CachedWrite { bool valid=false; char id[41]={0}; uint16_t bodyLength=0; char body[kBodyMax+1]={0}; Reply reply{}; } cache;

String bootString(uint32_t value) { char out[9]; snprintf(out,sizeof(out),"%08lx",(unsigned long)value); return String(out); }
bool loadTuningToken() {
  Preferences preferences;
  if (!preferences.begin("wifi-tuning", true)) return false;
  const size_t length = preferences.getString("token", tuningToken, sizeof(tuningToken));
  preferences.end();
  return length >= 32 && length < sizeof(tuningToken);
}
bool authorized(const char *header) {
  static const char prefix[] = "Bearer ";
  if (!header || strncmp(header,prefix,sizeof(prefix)-1)!=0) return false;
  return tuningTokenAvailable && strcmp(header+sizeof(prefix)-1,tuningToken)==0;
}
const char *reasonPhrase(int status) {
  switch(status){case 200:return "OK";case 400:return "Bad Request";case 401:return "Unauthorized";case 404:return "Not Found";case 409:return "Conflict";case 413:return "Payload Too Large";case 503:return "Service Unavailable";case 504:return "Gateway Timeout";default:return "Error";}
}
void sendJson(WiFiClient &client,int status,const String &payload) {
  client.printf("HTTP/1.1 %d %s\r\n",status,reasonPhrase(status));
  client.print("Content-Type: application/json\r\nCache-Control: no-store\r\nConnection: close\r\nContent-Length: ");
  client.print(payload.length());client.print("\r\n\r\n");client.write((const uint8_t *)payload.c_str(),payload.length());
}
void sendError(WiFiClient &client,int status,const char *requestId,const char *code) {
  JsonDocument doc;doc["ok"]=false;if(requestId&&*requestId)doc["request_id"]=requestId;doc["error"]=code;
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
  if(reply.ok&&reply.kind!=STATUS_REQUEST){
    JsonObject values=doc["values"].to<JsonObject>();for(uint8_t i=0;i<reply.count;i++)values[reply.names[i]]=reply.values[i];
    if(reply.kind==PARAMETERS_REQUEST){
      JsonArray metadata=doc["parameters"].to<JsonArray>();
      for(uint8_t i=0;i<reply.count;i++){const TuningParameter *parameter=findTuningParameter(reply.names[i]);if(!parameter)continue;JsonObject item=metadata.add<JsonObject>();item["name"]=parameter->name;item["min"]=parameter->minimum;item["max"]=parameter->maximum;item["description"]=parameter->description;}
    }
  }
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
  strlcpy(reply.bootId,bootString(bootId).c_str(),sizeof(reply.bootId));reply.supportedMode=state.supportedMode;reply.rcValid=state.rcValid;reply.rcAgeMs=state.rcAgeMs;reply.ch5Off=state.ch5Off;reply.tuningMode=state.tuningMode;
  reply.count=TUNING_PARAMETER_COUNT;for(size_t i=0;i<TUNING_PARAMETER_COUNT;i++){const TuningParameter *parameter=tuningParameterAt(i);strlcpy(reply.names[i],parameter->name,sizeof(reply.names[i]));reply.values[i]=parameter->value?*parameter->value:0;}
}
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
  if(!reply.ok){int status=strcmp(reply.error,"REQUEST_EXPIRED")==0?504:(strcmp(reply.error,"DRIVE_ACTIVE")==0||strcmp(reply.error,"RC_UNAVAILABLE")==0||strcmp(reply.error,"AUTO_MODE")==0||strcmp(reply.error,"BOOT_CHANGED")==0||strcmp(reply.error,"UNSUPPORTED_MODE")==0||strcmp(reply.error,"REQUEST_ID_CONFLICT")==0?409:400);sendReply(client,status,reply);return;}
  sendReply(client,200,reply);
}
void handleWrite(WiFiClient &client,const char *body,size_t length) {
  if(hasDuplicateJsonObjectKeys(body,length)){sendError(client,400,"","DUPLICATE_KEY");return;}
  JsonDocument doc;DeserializationError error=deserializeJson(doc,body,length,DeserializationOption::NestingLimit(4));
  if(error||!doc.is<JsonObject>()||doc.size()!=3){sendError(client,400,"","INVALID_SCHEMA");return;}
  const char *requestId=doc["request_id"]|"";const char *expectedBoot=doc["expected_boot_id"]|"";JsonObject values=doc["values"].as<JsonObject>();
  if(!requestId[0]||strlen(requestId)>40||strlen(expectedBoot)!=8||values.isNull()||values.size()==0||values.size()>TUNING_PARAMETER_COUNT){sendError(client,400,requestId,"INVALID_SCHEMA");return;}
  Request request{};request.kind=WRITE_REQUEST;strlcpy(request.requestId,requestId,sizeof(request.requestId));strlcpy(request.expectedBoot,expectedBoot,sizeof(request.expectedBoot));request.bodyLength=(uint16_t)length;memcpy(request.body,body,length);request.body[length]='\0';request.count=values.size();
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
  char extraToken=0;
  if(!readLine(client,line,sizeof(line),headerDeadline)||sscanf(line,"%7s %95s %15s %c",method,path,protocol,&extraToken)!=3||(strcmp(protocol,"HTTP/1.1")&&strcmp(protocol,"HTTP/1.0"))){sendError(client,400,"","INVALID_REQUEST");return;}
  char authorization[256]={0},contentType[64]={0};bool hasAuthorization=false,hasContentType=false,hasLength=false,hasTransferEncoding=false;size_t contentLength=0,totalHeaderBytes=0;
  for(;;){
    if(!readLine(client,line,sizeof(line),headerDeadline)){sendError(client,400,"","INVALID_HEADERS");return;}
    totalHeaderBytes+=strlen(line)+2;if(totalHeaderBytes>kHeaderMax){sendError(client,400,"","HEADERS_TOO_LARGE");return;}
    if(!line[0])break;
    char *colon=strchr(line,':');if(!colon){sendError(client,400,"","INVALID_HEADERS");return;}*colon='\0';char *value=colon+1;while(isspace((unsigned char)*value))value++;char *end=value+strlen(value);while(end>value&&isspace((unsigned char)end[-1]))*--end='\0';
    if(!strcasecmp(line,"Authorization")){if(hasAuthorization||strlen(value)>=sizeof(authorization)){sendError(client,400,"","INVALID_HEADERS");return;}hasAuthorization=true;strlcpy(authorization,value,sizeof(authorization));}
    else if(!strcasecmp(line,"Content-Type")){if(hasContentType||strlen(value)>=sizeof(contentType)){sendError(client,400,"","INVALID_HEADERS");return;}hasContentType=true;strlcpy(contentType,value,sizeof(contentType));}
    else if(!strcasecmp(line,"Content-Length")){if(hasLength||!value[0]){sendError(client,400,"","INVALID_HEADERS");return;}for(char*p=value;*p;p++)if(!isdigit((unsigned char)*p)){sendError(client,400,"","INVALID_HEADERS");return;}unsigned long parsed=strtoul(value,nullptr,10);if(parsed>0xffffffffUL){sendError(client,400,"","INVALID_HEADERS");return;}contentLength=(size_t)parsed;hasLength=true;}
    else if(!strcasecmp(line,"Transfer-Encoding")){hasTransferEncoding=true;}
  }
  if(!authorized(hasAuthorization?authorization:nullptr)){sendError(client,401,"","UNAUTHORIZED");return;}
  if(strchr(path,'?')){sendError(client,400,"","INVALID_REQUEST");return;}
  if(!strcmp(method,"GET")&&(!strcmp(path,"/api/v1/status")||!strcmp(path,"/api/v1/parameters"))){
    if((hasLength&&contentLength!=0)||hasTransferEncoding){sendError(client,400,"","INVALID_SCHEMA");return;}
    Request request{};request.kind=!strcmp(path,"/api/v1/status")?STATUS_REQUEST:PARAMETERS_REQUEST;sendQueued(client,request);return;
  }
  if(strcmp(method,"POST")||strcmp(path,"/api/v1/parameters")){sendError(client,404,"","NOT_FOUND");return;}
  if(hasTransferEncoding||!hasLength){sendError(client,400,"","INVALID_SCHEMA");return;}
  if(contentLength>kBodyMax){sendError(client,413,"","BODY_TOO_LARGE");return;}
  if(!hasContentType||strncasecmp(contentType,"application/json",16)!=0||(contentType[16]&&contentType[16]!=';'&&!isspace((unsigned char)contentType[16]))){sendError(client,400,"","INVALID_SCHEMA");return;}
  char body[kBodyMax+1];size_t received=0;const uint32_t bodyDeadline=millis()+kHttpReadTimeoutMs;
  while(received<contentLength&&(int32_t)(millis()-bodyDeadline)<0){if(!client.available()){vTaskDelay(pdMS_TO_TICKS(1));continue;}int count=client.read((uint8_t*)body+received,contentLength-received);if(count>0)received+=(size_t)count;}
  if(received!=contentLength){sendError(client,400,"","INCOMPLETE_BODY");return;}
  body[received]='\0';handleWrite(client,body,received);
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
    if (connected && tuningTokenAvailable && !serverStarted) {
      server.begin();
      serverStarted = true;
      Serial.println("Wi-Fi tuning API started.");
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
  tuningTokenAvailable=loadTuningToken() && requestQueue && replyQueue;
  if (!tuningTokenAvailable) Serial.println("Wi-Fi tuning API disabled; no runtime token is stored.");
  bootId=esp_random();
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
void WifiTuningProcessOne(const WifiTuningState &state) {
  if(!requestQueue||!replyQueue)return;Request request{};if(xQueueReceive(requestQueue,&request,0)!=pdTRUE)return;
  Reply reply{};reply.kind=request.kind;reply.sequence=request.sequence;fillReply(reply,state);reply.ok=true;strlcpy(reply.requestId,request.requestId,sizeof(reply.requestId));
  for(uint8_t i=0;i<request.count;i++)request.values[i].name=request.names[i];
  if((uint32_t)(millis()-request.receivedAt)>kRequestTtlMs){reply.ok=false;strlcpy(reply.error,"REQUEST_EXPIRED",sizeof(reply.error));}
  else if(request.kind==WRITE_REQUEST){
    bool replayed=false;
    if(cache.valid&&strcmp(cache.id,request.requestId)==0){
      if(cache.bodyLength==request.bodyLength&&memcmp(cache.body,request.body,request.bodyLength)==0){reply=cache.reply;reply.sequence=request.sequence;replayed=true;}
      else{reply.ok=false;strlcpy(reply.error,"REQUEST_ID_CONFLICT",sizeof(reply.error));}
    } else if(strcmp(request.expectedBoot,bootString(bootId).c_str())!=0){reply.ok=false;strlcpy(reply.error,"BOOT_CHANGED",sizeof(reply.error));}
    else{
      const char *policyError=validateTuningWritePolicy(state.supportedMode,state.rcValid,state.ch5Off,request.values,request.count,state.tuningMode);
      if(policyError){reply.ok=false;strlcpy(reply.error,policyError,sizeof(reply.error));}
      else{for(uint8_t i=0;i<request.count;i++){const TuningParameter *parameter=findTuningParameter(request.values[i].name);if(parameter&&parameter->value)*parameter->value=request.values[i].value;}fillReply(reply,state);reply.ok=true;strlcpy(reply.requestId,request.requestId,sizeof(reply.requestId));reply.sequence=request.sequence;}
      cache.valid=true;strlcpy(cache.id,request.requestId,sizeof(cache.id));cache.bodyLength=request.bodyLength;memcpy(cache.body,request.body,request.bodyLength);cache.body[request.bodyLength]='\0';cache.reply=reply;
    }
  }
  xQueueSend(replyQueue,&reply,0);
}
#else
void WifiTuningBegin() {}
void WifiTuningReprovision(char *) {}
void WifiTuningProcessOne(const WifiTuningState &) {}
#endif
