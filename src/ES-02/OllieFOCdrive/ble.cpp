#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include "ble.h"
#include <cppQueue.h>
#include "robot.h"
#include "Logging.h"
#include "string.h"

BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic;

// Reassembly state, private to ble_loop() (cpu0_task).
static BleDataTypDef ble_rx;
CmdManeuverTypDef ble_ctrler;

// Single-slot handoff from the BLE stack task (onWrite) to ble_rx_processing().
// A frame written while the slot is still occupied overwrites it.
static portMUX_TYPE ble_rx_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t ble_rx_slot_frame[20];
static uint8_t ble_rx_slot_seed;
static bool ble_rx_slot_full = false;

cppQueue	ble_tx_q(sizeof(BleDataTypDef*), 10, FIFO);	//Create a Bluetooth transmission queue, with first-in-first-out principle.

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "6E400011-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400012-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400013-B5A3-F393-E0A9-E50E24DCCA9E"

void ble_tx_processing(void);
void ble_rx_processing(void);
void ble_rx_data_add(uint8_t* data, uint8_t len);
void ble_rx_data_clear(void);
void ble_cmd_maneuver_processing(void);
void ble_cmd_json_processing(void);
bool ble_frames_validation(void);


class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    uint8_t* rxData = pCharacteristic->getData();
    if (pCharacteristic->getLength() == 20) {
      portENTER_CRITICAL(&ble_rx_mux);
      // The first non-zero pack count among unprocessed frames seeds remaining_pack,
      // so an overwritten frame still makes the sequence check fail as it did before the handoff.
      if (ble_rx_slot_seed == 0) {
        ble_rx_slot_seed = rxData[3];
      }
      memcpy(ble_rx_slot_frame, rxData, 20);
      ble_rx_slot_full = true;
      portEXIT_CRITICAL(&ble_rx_mux);
    }
  }
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    rp.ble_connected = true;
    Logging::message(Logging::Level::Debug, "ble", "ble connected");
  };

  void onDisconnect(BLEServer* pServer) {
    rp.ble_connected = false;
    // Restart the broadcast to allow for reconnection
    pServer->getAdvertising()->start();

    memset(&ble_ctrler, 0, sizeof(CmdManeuverTypDef));
    Logging::message(Logging::Level::Debug, "ble", "ble disconnected");
  }
};

void ble_init() {
  char ble_name[20] = { 0 };
  rp.build_dev_name(ble_name);
  // Create the BLE Device
  BLEDevice::init(ble_name);
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService* pService = pServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY);

  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE);

  pRxCharacteristic->setCallbacks(new MyCallbacks());
  // Start the service
  pService->start();
  // Start advertising
  pServer->getAdvertising()->start();
  Logging::message(Logging::Level::Debug, "ble", "Waiting a client connection to notify...");

  ble_rx_data_clear();
}
void ble_send_data(uint8_t* data, uint8_t len) {
  if (len > 20) {
    len = 20;
  }
  pTxCharacteristic->setValue(data, len);  //reply
  pTxCharacteristic->notify();
}


void ble_loop(void) {
  ble_rx_processing();
  ble_tx_processing();
}
void ble_rx_processing(void) {
  uint8_t slot_seed = 0;
  bool received = false;
  portENTER_CRITICAL(&ble_rx_mux);
  if (ble_rx_slot_full) {
    memcpy(ble_rx.frame, ble_rx_slot_frame, 20);
    slot_seed = ble_rx_slot_seed;
    ble_rx_slot_seed = 0;
    ble_rx_slot_full = false;
    received = true;
  }
  portEXIT_CRITICAL(&ble_rx_mux);

  //The BLE data reception has been completed
  if (received) {
    if (ble_rx.remaining_pack == 0) {
      ble_rx.remaining_pack = slot_seed;
    }
    ble_rx.cmd = ble_rx.frame[2];

    if (ble_frames_validation() == false) {
      ble_rx_data_clear();
      Logging::message(Logging::Level::Debug, "ble", "ble data err");
      return;
    }
    //BLE reply
    ble_send_data((uint8_t*)ble_rx.frame, 20);
    //Merge data packets
    if (ble_rx.remaining_pack > 0) {
      ble_rx_data_add(&ble_rx.frame[5], 15);
      ble_rx.remaining_pack--;
    } else  //Process data packets
    {
      //Merge the last package of data; if there is only one package of data, that is also the last one
      ble_rx_data_add(&ble_rx.frame[5], 15);
      switch (ble_rx.cmd) {
        case CMD_MANEUVER:
          {
            ble_cmd_maneuver_processing();
          }
          break;
        case CMD_JSON:
          {
            ble_cmd_json_processing();
          }
          break;
      }
      ble_rx_data_clear();
    }
  }
}
void ble_tx_processing(void) {

  /*
  If the queue is empty, no sending will be carried out.
  */
  if(ble_tx_q.getCount()==0){
    return;
  }
  /*
  If the queue is not empty but the Bluetooth connection is not established, then clear the queue.
  */
  if(rp.ble_connected == false){
    BleDataTypDef* _ble_tx;
    ble_tx_q.peek(&_ble_tx);
    ble_tx_q.drop();
    free(_ble_tx);
    Logging::message(Logging::Level::Debug, "ble", "BLE not connected, deleting queue; Queue remaining: %u", static_cast<unsigned int>(ble_tx_q.getCount()));
    return;
  }
  

  /*
  Read the queue data. After sending is completed, manual queue deletion and memory release are required.
  */
  BleDataTypDef* ble_tx;
  ble_tx_q.peek(&ble_tx);
  /*
  `ble_tx.index` is the index of the current data being sent.
  If it is greater than or equal to the total length,
  it indicates that the sending is complete or there is no data to be sent.
  */ 
  if (ble_tx->index >= ble_tx->len) {
    ble_tx->state = BLE_STATE_SEND_FINISH;
    Logging::message(Logging::Level::Debug, "ble", "finish!!!  ble_tx.index >= ble_tx.len  , ble_tx.index : %d , ble_tx.len : %d", static_cast<int>(ble_tx->index), static_cast<int>(ble_tx->len));
    ble_tx_q.drop();
    free(ble_tx);
    Logging::message(Logging::Level::Debug, "ble", "Queue remaining: %u", static_cast<unsigned int>(ble_tx_q.getCount()));
    return;
  }
  /*
  Control the interval time of Bluetooth data frames
  */
  static int8_t time_tick = 100;
  if (time_tick > 0) {
    time_tick -= 10;
    return;
  } else {
    time_tick = 100;
  }

  ble_tx->state = BLE_STATE_SEND_BEING;

  ble_tx->frame[0] = 0x55;
  ble_tx->frame[1] = 0xAA;
  ble_tx->frame[2] = 0x02;  //json
  ble_tx->frame[3] = (ble_tx->len - ble_tx->index) / 15 - ((ble_tx->len - ble_tx->index)%15 ? 0:1);
  ble_tx->frame[4] = 0;

  // The last frame is zero-padded to 15 bytes.
  int16_t chunk = ble_tx->len - ble_tx->index;
  if (chunk > 15) {
    chunk = 15;
  }
  memset(&ble_tx->frame[5], 0, 15);
  memcpy(&ble_tx->frame[5], &ble_tx->data[ble_tx->index], chunk);

  ble_send_data((uint8_t*)ble_tx->frame, 20);

  Logging::message(Logging::Level::Debug, "ble", "ble send frame -> %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X ----",
                   static_cast<unsigned int>(ble_tx->frame[0]), static_cast<unsigned int>(ble_tx->frame[1]),
                   static_cast<unsigned int>(ble_tx->frame[2]), static_cast<unsigned int>(ble_tx->frame[3]),
                   static_cast<unsigned int>(ble_tx->frame[4]), static_cast<unsigned int>(ble_tx->frame[5]),
                   static_cast<unsigned int>(ble_tx->frame[6]), static_cast<unsigned int>(ble_tx->frame[7]),
                   static_cast<unsigned int>(ble_tx->frame[8]), static_cast<unsigned int>(ble_tx->frame[9]),
                   static_cast<unsigned int>(ble_tx->frame[10]), static_cast<unsigned int>(ble_tx->frame[11]),
                   static_cast<unsigned int>(ble_tx->frame[12]), static_cast<unsigned int>(ble_tx->frame[13]),
                   static_cast<unsigned int>(ble_tx->frame[14]), static_cast<unsigned int>(ble_tx->frame[15]),
                   static_cast<unsigned int>(ble_tx->frame[16]), static_cast<unsigned int>(ble_tx->frame[17]),
                   static_cast<unsigned int>(ble_tx->frame[18]), static_cast<unsigned int>(ble_tx->frame[19]));

  ble_tx->index += 15;
}
bool ble_frames_validation(void) {
  static uint8_t last_remaining_pack;
  static uint8_t last_cmd;
  //Fixed header
  if (ble_rx.frame[0] != 0x55) return false;
  if (ble_rx.frame[1] != 0xAA) return false;
  //If there is a subsequent frame in the previous data, the command must be the same.
  if ((ble_rx.frame[2] != last_cmd) && (last_remaining_pack > 0)) return false;
  if (ble_rx.frame[3] != ble_rx.remaining_pack) return false;

  last_cmd = ble_rx.frame[2];
  last_remaining_pack = ble_rx.remaining_pack;
  return true;
}

void ble_rx_data_add(uint8_t* data, uint8_t len) {
  int i;
  for (i = 0; i < len; i++) {
    ble_rx.data[ble_rx.index] = *(data + i);
    ble_rx.index++;
    if (ble_rx.index > (BLE_DATA_SIZE - 1)) {
      ble_rx.index = BLE_DATA_SIZE - 1;
    }
  }
  ble_rx.data[ble_rx.index] = 0;
}

void ble_rx_data_clear() {
  ble_rx.index = 0;
  ble_rx.remaining_pack = 0;
}

//
void ble_cmd_maneuver_processing(void) {
  memcpy(ble_ctrler.ch, ble_rx.data, 10);
}
void ble_cmd_json_processing(void) {
  String payload_str = String((char*)ble_rx.data);
  StaticJsonDocument<300> doc;
  deserializeJson(doc, payload_str);
  rp.parseJson(doc);
}
void ble_tx_add_data(const char* data, int len) {

  if(ble_tx_q.isFull() ==true){
    Logging::message(Logging::Level::Debug, "ble", "Queue is full, failed to add data");
    return ;
  }

  if(len>BLE_DATA_SIZE-1){
    len = BLE_DATA_SIZE-1;
  }

  BleDataTypDef* ble_tx;
  ble_tx = (BleDataTypDef*)malloc(sizeof(BleDataTypDef));
  if(ble_tx == NULL)
  {
    Logging::message(Logging::Level::Debug, "ble", "error!!!!malloc false,ble txdata");
    return;
  }
  memcpy(ble_tx->data, data, len);
  ble_tx->data[len] = 0;

  ble_tx->len = len;
  ble_tx->index = 0;
  ble_tx->state = BLE_STATE_SEND_READY;

  ble_tx_q.push(&ble_tx);
  Logging::message(Logging::Level::Debug, "ble", "ble_tx_add_data: %.44s...", ble_tx->data);
  Logging::message(Logging::Level::Debug, "ble", "Queue remaining: %u", static_cast<unsigned int>(ble_tx_q.getCount()));

}
void ble_tx_add_string(const String& str) {
  // The terminating NUL is part of the payload, as it was with the copy through a local buffer.
  ble_tx_add_data(str.c_str(), str.length() + 1);
}
