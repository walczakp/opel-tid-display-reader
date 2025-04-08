#include "TIDDisplay.hpp"

TIDDisplay* TIDDisplay::instance = nullptr;

TIDDisplay::TIDDisplay(uint8_t mrqPin)
  : mrqPin(mrqPin),
    mrqTriggered(false), transmissionStarted(false), dataReceived(false),
    busErrorCallback(nullptr), dataReceivedCallback(nullptr) {
  noInterrupts();
  memset((void*)received_data, 0, sizeof(received_data));
  instance = this;
  interrupts();
}

void TIDDisplay::begin() {
  pinMode(mrqPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(mrqPin), onMRQInterruptWrapper, CHANGE);
  i2cInit();
  interrupts();
}

void TIDDisplay::onBusError(Callback callback) {
  busErrorCallback = callback;
}

void TIDDisplay::onDataReceived(DataCallback callback) {
  dataReceivedCallback = callback;
}

void TIDDisplay::i2cInit() {
  TWAR = (SLAVE_ADDRESS << 1);                     // set slave address
  TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA);  // enable I2C, interrupts, ACKs
  TWSR = 0x00;                                     // reset state register
}


ISR(TWI_vect) {
  TIDDisplay::instance->interruptHandler();
}

void TIDDisplay::interruptHandler() {
  if (TIDDisplay::instance == nullptr) {
    return;
  }

  TIDDisplay* instance = TIDDisplay::instance;
  static uint8_t byte_index = 0;
  switch (TW_STATUS) {
    // start condition
    case TW_SR_SLA_ACK:
      byte_index = 0;
      break;
    // data received & acked
    case TW_SR_DATA_ACK:
      if (byte_index < sizeof(instance->received_data)) {
        instance->received_data[byte_index++] = TWDR;
      }
      break;
    case TW_BUS_ERROR:
      TWCR = 0;
      instance->busErrorCallback();
      return;
      // for some reason this does not get triggered, though it's visible in the bus capture
      // case TW_SR_STOP:
      // break;
  }
  TWCR = (1 << TWIE) | (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
}

void TIDDisplay::handleInterrupt() {
  if (digitalRead(2) == HIGH) {
    // rising edge
    if (transmissionStarted)  // master set MRQ to high at the end of comms
      dataReceived = true;
  } else {
    // falling edge
    if (!transmissionStarted)  // master set MRQ to low at the start of comms
      mrqTriggered = true;
  }
}

void TIDDisplay::handleMRQ() {   // this gets executed on first falling edge on MRQ during start of the transmission
    TWI_disable();
    pinMode(SDA, OUTPUT);
    delayMicroseconds(100);      // await 100us and...
    digitalWrite(SDA, LOW);  // pull SDA low

    while (digitalRead(mrqPin) != HIGH);  // await master to pull MRQ high

    delayMicroseconds(100);       // await 100us and...
    digitalWrite(SDA, HIGH);  // pull SDA high
    pinMode(SDA, INPUT);      // let master control SDA for real I2C

    noInterrupts();
    transmissionStarted = true;
    interrupts();

    TWI_enable();
  }

  bool TIDDisplay::waitForData() {
    unsigned long startTime = millis();
    while (!dataReceived) {
      if (millis() - startTime >= RECEIVE_TIMEOUT_MILLIS) {
        return false;
      }
    }
    return true;
  }

  void TIDDisplay::notifyDataReceived() {
    if (dataReceivedCallback) {
      DisplayData data;
      data.dot = received_data[0] & 0b10000000;
      data.rds = received_data[0] & 0b01000000;
      data.tp = received_data[0] & 0b00100000;
      data.stereo = received_data[0] & 0b00010000;
      data.as = received_data[0] & 0b00000100;
      data.tp_brackets = received_data[0] & 0b00000010;

      data.cd_in = received_data[1] & 0b10000000;
      data.dolby_c = received_data[1] & 0b01000000;
      data.dolby_b = received_data[1] & 0b00100000;
      data.cr = received_data[1] & 0b00010000;
      data.cps = received_data[1] & 0b00001000;

      for (int i = 0; i < 8; ++i) {
        data.text[i] = (received_data[i+2] >> 1);
      }

      dataReceivedCallback(data);
    }
  }

  void TIDDisplay::resetState() {
    noInterrupts();
    transmissionStarted = false;
    dataReceived = false;
    mrqTriggered = false;
    memset((void*)received_data, 0, sizeof(received_data));
    interrupts();
  }

  void TIDDisplay::TWI_disable() {
    noInterrupts();
    TWCR &= ~(1 << TWEN);
    interrupts();
  }

  void TIDDisplay::TWI_enable() {
    noInterrupts();
    TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
    interrupts();
  }

  void TIDDisplay::process() {
    if (mrqTriggered) {
      handleMRQ();

      if (waitForData()) {
        notifyDataReceived();
      } else {
        TWI_disable();
      }

      resetState();
    }
  }

  void TIDDisplay::onMRQInterruptWrapper() {
    instance->handleInterrupt();
  }
