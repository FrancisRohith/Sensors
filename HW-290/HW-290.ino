#include <Wire.h>
#include "MPU6050.h"
#include "BMP180.h"
#include "QMC5883L.h"

/*
PIN CONNECTION
VCC_IN - 3v3
SCL - PB6
SDA - PB7
*/ 

#define MPU_ADDR 0x68
#define QMC_ADDR 0x0D

int16_t AccX, AccY, AccZ;
int16_t GyroX, GyroY, GyroZ;

BMP180 bmp;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  // MPU6050
  // values are stored in Tx buffer
  Wire.beginTransmission(MPU_ADDR);   // (0x68 << 1) + W/R -> 0/1
  Wire.write(PWR_MGMT_1); // target register address is PWR_MNGT_1
  Wire.write(0x00); // value to be written in the target register (wakeup)
  // end transmission is called to transmit values in the Tx buffer
  // if false values in buffer is sent but wire is still listenning for read data
  Wire.endTransmission(true); 

  Wire.beginTransmission(0x68); // MPU6050
  Wire.write(INT_PIN_CFG);      
  Wire.write(0x02);             // BYPASS_EN = 1
  Wire.endTransmission();

  // Set accelerometer range to ±2g (most stable)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_CONFIG);           // ACCEL_CONFIG register
  Wire.write(0x00);           // ±2g range
  Wire.endTransmission(true);

  // Set sample rate (optional)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(SMPLRT_DIV);          
  Wire.write(0x07);           // Sample rate = 1kHz / (7+1) = 125Hz every 8ms new reading is ready
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG);     // GYRO_CONFIG register
  Wire.write(0x00);     // ±250dps
  Wire.endTransmission(true);

  // BMP180
  if (!bmp.begin()) {
    Serial.println("BMP180 not found!");
    while (1);
  }
  
  // QMC5883L
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(SRST);
  Wire.write(0x01);
  Wire.endTransmission(true);

  Wire.beginTransmission(QMC_ADDR);
  Wire.write(CR1);
  Wire.write(0x1D);
  Wire.endTransmission(true);
}

void loop() {
  // MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);   
  Wire.endTransmission(false);  // keeps the bus active for reading
  Wire.requestFrom(MPU_ADDR, 14, true); // reads 14 bytes and end transmission

  // Read raw data (16-bit signed)
  AccX = (Wire.read() << 8) | Wire.read();
  AccY = (Wire.read() << 8) | Wire.read();
  AccZ = (Wire.read() << 8) | Wire.read();

  Wire.read(); Wire.read(); // Temparature data unused

  GyroX = (Wire.read() << 8) | Wire.read();
  GyroY = (Wire.read() << 8) | Wire.read();
  GyroZ = (Wire.read() << 8) | Wire.read();

  float ax = AccX / 16384.0;
  float ay = AccY / 16384.0;
  float az = AccZ / 16384.0;

  // Convert raw values to degrees per second
  float gx = GyroX / 131.0;
  float gy = GyroY / 131.0;
  float gz = GyroZ / 131.0;

  // BMP180
  float temp = bmp.readTemperature();
  int32_t pressure = bmp.readPressure();
  float altitude = bmp.readAltitude();

  // QMC5883L
  if (dataReady()) {

  Wire.beginTransmission(0x0D);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.requestFrom(0x0D, 6);

  int16_t MAG_X = (int16_t)(Wire.read() | (Wire.read()<<8));
  int16_t MAG_Y = (int16_t)(Wire.read() | (Wire.read()<<8));
  int16_t MAG_Z = (int16_t)(Wire.read() | (Wire.read()<<8));


  float mx = MAG_X/3000.0;
  float my = MAG_Y/3000.0;
  float mz = MAG_Z/3000.0;
  Serial.print(mx); Serial.print(" ");
  Serial.print(my); Serial.print(" ");
  Serial.print(mz); Serial.println(" ");
  delay(200);
  }
}

bool dataReady() {
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(STR); // STATUS register
  Wire.endTransmission();

  Wire.requestFrom(QMC_ADDR, 1);
  uint8_t status = Wire.read();

  return status & 0x01; // DRDY bit
}
