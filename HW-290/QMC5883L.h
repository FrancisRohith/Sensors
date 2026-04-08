#define X_OUT_L 0x00
#define X_OUT_H 0x01
#define Y_OUT_L 0x02
#define Y_OUT_H 0x03
#define Z_OUT_L 0x04
#define Z_OUT_H 0x05
#define STR 0x06
#define T_OUT_L 0x07
#define T_OUT_H 0x08
#define CR1 0x09
#define CR2 0x0A
#define SRST 0x0A
#define QMC_ID 0x0D

/*
- Raw data is in magnetic field (Gauss/uT)
- Raw=1000 → 0.1 Gauss

////////////////////////////// DATASHEET DETAILS //////////////////////////////////////////////////
// QMC5883L //
- Register Map -> 0x00-0x0D (8 bit registers )
- 0x00-0x05 -> Magnetometer Output Data register in 2's compliment
- 0x06 - Status register -> {--, DOR, OVL, DRDY}
  DRDY - Data Ready set to 1 when all 3 axis data is ready, reset to 0 by reading any data register
  OVL  - Overflow flag is set to 1 if any data out of range, reset to 0 if next data is in range
  DOR  - Data Skip is set to 1 if data is skipped for reading, reset to 0 if all data is red
- 0x07-0x08 Temparature reading
- CR1 -> {OSR[1:0], RNG[1:0], ODR[1:0], MODE[1:0]}
  OSR - Over sample ratio
    00 - 512
    01 - 256
    10 - 128
    11 - 64
  RNG - Full Scale
    00 - 2g
    01 - 8g
    AFS_SEL | Full Scale Range | LSB Sensitivity
      00         ±2g              12000 LSB/g
      01         ±8g               3000 LSB/g
  ODR - Output Data Rate
    00 - 10Hz
    01 - 50Hz
    10 - 100Hz
    11 - 200Hz
  Mode Control
    00 -standby
    01 -Continous

- SRST -> Set/Reset period. It determines how often sensor should recalibrate
  - Set to 0x01 for more frequent calibration
*/
