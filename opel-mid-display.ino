#include <avr/interrupt.h>
#include <util/twi.h>

#define MRQ_PIN 2 
#define SDA_PIN A4
#define SCL_PIN A5
#define SLAVE_ADDRESS 0x4A

volatile bool mrqTriggered = false;         // MRQ triggered by master
volatile bool transmissionStarted = false;  // real I2C transmission started after MRQ foreplay
volatile bool dataReceived = false;         // master signals transmission end
volatile uint8_t received_data[10];


void emergencyReset() {
  asm volatile ("jmp 0");
}

void i2cInit() {
    TWAR = (SLAVE_ADDRESS << 1); // set slave address
    TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA); // enable I2C, interrupts, ACKs
    TWSR = 0x00; // reset state register
}

void onMRQInterrupt() {
  if (digitalRead(2) == HIGH) {
    // rising edge
    if (transmissionStarted) // master set MRQ to high at the end of comms
      dataReceived = true;
  } else {
    // falling edge
    if (!transmissionStarted) // master set MRQ to low at the start of comms
      mrqTriggered = true; 
  }
}


ISR(TWI_vect)
{
  static uint8_t byte_index = 0;
  switch(TW_STATUS)
  {
    // start condition
    case TW_SR_SLA_ACK:
      byte_index = 0;
      break;
    // data received & acked
    case TW_SR_DATA_ACK:
      if (byte_index < 10) {
        received_data[byte_index++] = TWDR;
      }
      break;
    case TW_BUS_ERROR:
      TWCR = 0;
      emergencyReset();
      return;
    // for some reason this does not get triggered, though it's visible in the bus capture
    // case TW_SR_STOP:
      // break;
  }
  TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
} 

void TWI_disable() {
    noInterrupts();
    TWCR &= ~(1 << TWEN);
    interrupts();
}

void TWI_enable() {
    noInterrupts();
    TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA); 
    interrupts();
}

void setup() {
  attachInterrupt(digitalPinToInterrupt(MRQ_PIN), onMRQInterrupt, CHANGE);

  i2cInit();
  interrupts();

  Serial.begin(9600);
}

void resetState() {
    noInterrupts();
    transmissionStarted = false;
    dataReceived = false;
    mrqTriggered = false;
    memset(received_data, 0, sizeof(received_data));
    interrupts();
}

void handleMRQ() {  // this gets executed on first falling edge on MRQ during start of the transmission
    TWI_disable();
    pinMode(SDA_PIN, OUTPUT);
    delayMicroseconds(100); // await 100us and...
    digitalWrite(SDA_PIN, LOW); // pull SDA low

    while (digitalRead(MRQ_PIN) != HIGH); // await master to pull MRQ high 
    
    delayMicroseconds(100); // await 100us and...
    digitalWrite(SDA_PIN, HIGH); // pull SDA high
    pinMode(SDA_PIN, INPUT); // let master control SDA for real I2C
    
    noInterrupts();
    transmissionStarted = true;
    interrupts();
    
    TWI_enable();
}

bool waitForData() {
  unsigned long startTime = millis();
  while (!dataReceived) {
    if (millis() - startTime >= 1000) 
      return false;
  };
  return true;
}

void loop() {
  
  if (mrqTriggered) {
    handleMRQ();

    if (waitForData()) {
      interpretDisplayData();
    } else {
      TWI_disable();
    }

    resetState();
  }
}

void interpretDisplayData() {
  Serial.print("Symbols: ");
  Serial.print((received_data[0] & 0b10000000) ? "Dot " : "");
  Serial.print((received_data[0] & 0b01000000) ? "RDS " : "");
  Serial.print((received_data[0] & 0b00100000) ? "TP " : "");
  Serial.print((received_data[0] & 0b00010000) ? "Stereo " : "");
  Serial.print((received_data[0] & 0b00000100) ? "AS " : "");
  Serial.print((received_data[0] & 0b00000010) ? "[TP] " : "");
  Serial.print((received_data[1] & 0b10000000) ? "CD-In " : "");
  Serial.print((received_data[1] & 0b01000000) ? "Dolby C " : "");
  Serial.print((received_data[1] & 0b00100000) ? "Dolby B " : "");
  Serial.print((received_data[1] & 0b00010000) ? "cr " : "");
  Serial.print((received_data[1] & 0b00001000) ? "CPS " : "");
  Serial.println();

  Serial.print("Text: ");
  for (int i = 2; i < 10; i++) {
    uint8_t character = received_data[i] >> 1; // ignore parity bit
    if (character >= 32 && character <= 126) { 
      Serial.print((char)character);
    }
  }
  Serial.println();
}
