#ifndef TID_DISPLAY_HPP
#define TID_DISPLAY_HPP

#include <avr/interrupt.h>
#include <util/twi.h>
#include <Arduino.h>

#define SLAVE_ADDRESS 0x4A
#define RECEIVE_TIMEOUT_MILLIS 500

class TIDDisplay {
public:
  static TIDDisplay* instance;

  struct DisplayData {
    bool dot;
    bool rds;
    bool tp;
    bool stereo;
    bool as;
    bool tp_brackets;
    bool cd_in;
    bool dolby_c;
    bool dolby_b;
    bool cr;
    bool cps;
    char text[8];
  };

  using Callback = void (*)();
  using DataCallback = void (*)(const DisplayData&);

  TIDDisplay(uint8_t sclPin);

  void begin();
  void onBusError(Callback callback);
  void onDataReceived(DataCallback callback);
  void process();
  static void interruptHandler();

private:
  uint8_t mrqPin;

  volatile bool mrqTriggered;
  volatile bool transmissionStarted;
  volatile bool dataReceived;
  volatile uint8_t received_data[10];

  Callback busErrorCallback;
  DataCallback dataReceivedCallback;

  void i2cInit();
  void handleInterrupt();
  void handleMRQ();
  bool waitForData();
  void notifyDataReceived();
  void resetState();
  void TWI_disable();
  void TWI_enable();

  static void onMRQInterruptWrapper();
};

#endif