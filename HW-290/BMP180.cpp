#include "BMP180.h"
#include <Arduino.h>
#include <Wire.h>

BMP180::BMP180(uint8_t address)
{
    BMP_ADDR = address;
}

bool BMP180::begin()
{
    Wire.begin();

    Wire.beginTransmission(BMP_ADDR);
    Wire.write(BMP_ID);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP_ADDR, (uint8_t)1, true); // typecasting to 1 unsigned 8 bit

    if (Wire.available())
    { // Wire > 0
        uint8_t id = Wire.read();
        if (id != 0x55)
            return false;
    }
    else
    {
        return false;
    }
    readCalibration();
    return true;
}

int16_t BMP180::readInt(uint8_t reg)
{
    Wire.beginTransmission(BMP_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP_ADDR, (uint8_t)2, true);
    return (Wire.read() << 8) | Wire.read();
}

void BMP180::readCalibration()
{
    AC1 = readInt(0xAA);
    AC2 = readInt(0xAC);
    AC3 = readInt(0xAE);
    AC4 = readInt(0xB0);
    AC5 = readInt(0xB2);
    AC6 = readInt(0xB4);
    bmp_B1 = readInt(0xB6);
    B2 = readInt(0xB8);
    MB = readInt(0xBA);
    MC = readInt(0xBC);
    MD = readInt(0xBE);
}

int32_t BMP180::readRawTemp()
{
    Wire.beginTransmission(BMP_ADDR);
    Wire.write(CTRL_MEAS);
    Wire.write(0x2E);
    Wire.endTransmission();
    delay(5); // conversion time

    Wire.beginTransmission(BMP_ADDR);
    Wire.write(OUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP_ADDR, (uint8_t)2, true);

    return (Wire.read() << 8) | Wire.read();
}

int32_t BMP180::readRawPressure()
{
    Wire.beginTransmission(BMP_ADDR);
    Wire.write(CTRL_MEAS);
    Wire.write(0x34); // oss=0
    Wire.endTransmission();
    delay(5); // conversion time

    Wire.beginTransmission(BMP_ADDR);
    Wire.write(OUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP_ADDR, (uint8_t)3, true);

    int32_t msb = Wire.read();
    int32_t lsb = Wire.read();
    int32_t xlsb = Wire.read();
    return ((msb << 16) | (lsb << 8) | xlsb) >> 8; // xlsb for fraction precision
    // Shift (>> 8) depends on OSS
    // OSS=0 -> lowest precision
}

int32_t BMP180::computeB5(int32_t UT)
{
    int32_t X1 = ((UT - AC6) * AC5) >> 15;
    int32_t X2 = (MC << 11) / (X1 + MD);
    return X1 + X2;
}

float BMP180::readTemperature()
{
    int32_t UT = readRawTemp();
    int32_t B5 = computeB5(UT);
    return ((B5 + 8) >> 4) / 10.0;
}

int32_t BMP180::readPressure()
{
    int32_t UT = readRawTemp();
    int32_t UP = readRawPressure();
    int32_t B5 = computeB5(UT);
    int32_t B6 = B5 - 4000;
    int32_t X1 = (B2 * (B6 * B6 >> 12)) >> 11;
    int32_t X2 = (AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = (((int32_t)AC1 * 4 + X3) + 2) >> 2;
    X1 = (AC3 * B6) >> 13;
    X2 = (bmp_B1 * (B6 * B6 >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    uint32_t B4 = (AC4 * (uint32_t)(X3 + 32768)) >> 15;
    uint32_t B7 = ((uint32_t)UP - B3) * 50000;
    int32_t pressure;

    if (B7 < 0x80000000)
        pressure = (B7 * 2) / B4;
    else
        pressure = (B7 / B4) * 2;

    X1 = (pressure >> 8) * (pressure >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * pressure) >> 16;

    return pressure + ((X1 + X2 + 3791) >> 4);
}

float BMP180::readAltitude(float seaLevelPressure)
{
    float pressure = readPressure();
    return 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
}
