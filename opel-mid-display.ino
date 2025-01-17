#include <avr/interrupt.h>
#include <util/twi.h>
#include "tid-display.hpp"

#define MRQ_PIN 2 

TidDisplay tidDisplay(MRQ_PIN);

void emergencyReset() {
    asm volatile("jmp 0");
}

void dataReceived(const TidDisplay::DisplayData& data) {
    if (data.rds) Serial.print("RDS ");
    if (data.stereo) Serial.print("Stereo ");
    Serial.println();

    Serial.print("Text: ");
    for (int i = 0; i < 8; i++) {
        if (data.text[i] >= 32 && data.text[i] <= 126) {
            Serial.print((char)data.text[i]);
        }
    }
    Serial.println();
}

void setup() {
  Serial.begin(9600);
  
  tidDisplay.begin();
  tidDisplay.onBusError(emergencyReset);
  tidDisplay.onDataReceived(dataReceived);
}

void loop() {
  tidDisplay.process();
}

