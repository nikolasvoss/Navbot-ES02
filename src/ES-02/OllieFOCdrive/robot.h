#pragma once

#include <ArduinoJson.h>

typedef struct {
  const String GET_DEVICE_INFO = "get_device_info";
} Message_TypeDef;

extern const Message_TypeDef MESSAGE_TYPE;

class RobotProtocol {
public:
  double battery_voltage = 8.4f;
  double battery_level;
  bool ble_connected = false;

  double get_battery_voltage(void);
  double get_battery_level(void);
  void build_dev_name(char dev_name[]);
  void isSys(StaticJsonDocument<300> &doc);
  void parseJson(StaticJsonDocument<300> &doc);
  void json_test(char *json_arr);
private:
  String get_device_info(void);
  void send_device_info();
};

extern RobotProtocol rp;
