#ifndef BMP180_H
#define BMP180_H

#define BMP_ID 0xD0
#define CTRL_MEAS 0xF4
// Output
#define OUT_H 0xF6
#define OUT_L 0xF7
#define OUT_X 0xF8

#include <Arduino.h>

/*
////////////////////////////// DATASHEET DETAILS //////////////////////////////////////////////////
// BMP180 //
- Register Map -> 0xAA-0xF8 (8 bit registers )
- ID - Sensor Chip ID -> 0x55
- CTRL_MEAs - {OSS[1:0], SCO, MEAS[4:0]}
  OSS - Oversampling ratio of pressure measurement
  00 - 1 time
  01 - 2 time
  10 - 4 time
  11 - 8 time
  SCO - Start of conversion
  - Set to 1 during conversion
*/

class BMP180
{
public:
    BMP180(uint8_t address = 0x77);

    bool begin(); // initialize sensor

    float readTemperature(); // °C
    int32_t readPressure();  // Pa
    float readAltitude(float seaLevelPressure = 101325.0);

private:
    uint8_t BMP_ADDR;

    // Calibration coefficients
    int16_t AC1, AC2, AC3, bmp_B1, B2, MB, MC, MD;
    uint16_t AC4, AC5, AC6;

    // Internal helpers
    int16_t readInt(uint8_t reg);
    void readCalibration();

    int32_t readRawTemp();
    int32_t readRawPressure();

    int32_t computeB5(int32_t UT);
};

#endif