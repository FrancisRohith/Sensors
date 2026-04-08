#include <SPI.h>
#include "RF24.h"

RF24 myRadio(9, 10);  // CE -> 9, CSN -> 10
byte addresses[][6] = {"0"};

struct package {
  int pwm1 = 0;
};

typedef struct package Package;
Package dataTransmit;

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 50; // send every 50ms (~20Hz)

void setup() {
  Serial.begin(115200);
  Serial.println("Arduino Transmitter Ready...");

  myRadio.begin();
  myRadio.setChannel(115);
  myRadio.setAutoAck(false);
  myRadio.setPALevel(RF24_PA_MAX);
  myRadio.setDataRate(RF24_250KBPS); // for long range
  myRadio.openWritingPipe(addresses[0]);
}

void loop() {
  if (Serial.available() > 0) {
    int val = Serial.parseInt();  // Read integer
    if (val >= 0 && val <= 1000) {
      dataTransmit.pwm1 = val;
      Serial.print("New value set: ");
      Serial.println(val);
    } else {
      Serial.println("Please enter value between 0 - 1000");
    }
  }

  if (millis() - lastSendTime > SEND_INTERVAL) {
    myRadio.write(&dataTransmit, sizeof(dataTransmit));
    lastSendTime = millis();

    // Optional: print for debug
    Serial.print("Transmitting: ");
    Serial.println(dataTransmit.pwm1);
  }
}