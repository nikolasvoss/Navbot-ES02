#ifndef FUTABA_SBUS_h
#define FUTABA_SBUS_h

#include <Arduino.h>

class FUTABA_SBUS {
 public:
  static constexpr uint8_t kSignalOk = 0x00;

  int16_t channels[18];
  int toChannels;

  void begin();
  void FeedLine();
  void UpdateChannels();
  uint8_t Failsafe();

 private:
  uint8_t sbusData[25];
  uint8_t failsafe_status;
  uint8_t inBuffer[25];
  int bufferIndex;
  uint8_t inData;
  int feedState;
};

#endif
