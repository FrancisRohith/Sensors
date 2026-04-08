#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>  // For ESC or Servo control

// CE, CSN pins for STM32 (adjust PB0, PA4 if needed)
RF24 myRadio(PB0, PA4);

byte addresses[][6] = {"0"};

// Define structure (must match transmitter exactly)
struct __attribute__((packed)) package {
  int16_t pwm1 = 0;
};
typedef struct package Package;
Package dataReceive;

// Create Servo objects for motor and steering
Servo esc1;    // v1 controls throttle
Servo esc2; 
Servo esc3; 
Servo esc4; 

// Define output pins for PWM
#define ESC_PIN1  PB6   // TIM1_CH1 
#define ESC_PIN2  PB7
#define ESC_PIN3  PB8
#define ESC_PIN4  PB9

unsigned long lastPacketTime = 0;
const unsigned long failsafeTimeout = 200; // ms

void setup() {
  Serial.begin(115200);

  // Initialize radio
  myRadio.begin();
  myRadio.setChannel(115);
  myRadio.setPALevel(RF24_PA_MAX);
  myRadio.setDataRate(RF24_250KBPS);
  myRadio.setAutoAck(false);
  myRadio.openReadingPipe(0, addresses[0]);
  myRadio.startListening();

  Serial.println("STM32 Receiver Ready...");

  esc1.attach(ESC_PIN1);
  esc2.attach(ESC_PIN2);
  esc3.attach(ESC_PIN3);
  esc4.attach(ESC_PIN4); 

  Serial.println("Starting ESC calibration...");
  esc1.writeMicroseconds(2000); 
  esc2.writeMicroseconds(2000); 
  esc3.writeMicroseconds(2000); 
  esc4.writeMicroseconds(2000); 
  delay(2000);                  
  esc1.writeMicroseconds(1000);              
  esc2.writeMicroseconds(1000);              
  esc3.writeMicroseconds(1000);                
  esc4.writeMicroseconds(1000);
  delay(2000);                  

  Serial.println("Calibration done. Starting test...");

  delay(2000);  // Wait 2 sec for ESC arming
}

void loop() {
  
    if (myRadio.available()) {
        myRadio.read(&dataReceive, sizeof(dataReceive));
        int throttlePWM = dataReceive.pwm1;

        esc1.writeMicroseconds(throttlePWM);
        esc2.writeMicroseconds(throttlePWM);
        esc3.writeMicroseconds(throttlePWM);
        esc4.writeMicroseconds(throttlePWM);

        lastPacketTime = millis();
    }

    // FAILSAFE
    if (millis() - lastPacketTime > failsafeTimeout) {
        esc1.writeMicroseconds(0);
        esc2.writeMicroseconds(0);
        esc3.writeMicroseconds(0);
        esc4.writeMicroseconds(0);
    }
    
  Serial.print("pwm: ");
    Serial.println(dataReceive.pwm1);
}