
#define PWR_MGMT_1  0x6B
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define SMPLRT_DIV  0x19
#define INT_PIN_CFG 0x37
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

/*
////////////////////////////// DATASHEET DETAILS //////////////////////////////////////////////////
// MPU050 //
- Register Map -> 0x0D-0x75 (8 bit registers )
- Reset value is zero for all reg except 0x40 and 0x68
- PWR_MGMT_1 - Power Management register -> set sleep=0 for wakeup
- GYRO_CONFIG - Gyroscope Configuration register -> {XG_ST, YG_ST, ZG_ST, FS_SEL[1:0], -}
  - XG_ST -> set to 1 to perform X axis gyroscope perform self test and same for YG_ST, ZG_ST
  FS_SEL | Full Scale Range | LSB Sensitivity
    0       ± 250 °/s           131 LSB/°/s
    1       ± 500 °/s           65.5 LSB/°/s
    2       ± 1000 °/s          32.8 LSB/°/s
    3       ± 2000 °/s          16.4 LSB/°/s
  - 16 bit ADC so output ranges from -32768 to +32767
  - for ± 250, 32768/250 = 131

- ACCEL_CONFIG - Accelerometer Configuration register -> {XA_ST, YA_ST, ZA_ST, AFS_SEL[1:0], -}
  - XA_ST -> set to 1 to perform X axis accelerometer self test and same for YA_ST, ZA_ST
  AFS_SEL | Full Scale Range | LSB Sensitivity
    0         ±2g              16384 LSB/g
    1         ±4g               8192 LSB/g
    2         ±8g               4096 LSB/g
    3         ±16g              2048 LSB/g
  - 16 bit ADC so output ranges from -32768 to +32767
  - for 2g, 32768/2 = 16384

- SMPLRT_DIV - Sample Rate Divider register
  Sample Rate = Gyro Output Rate / (1 + SMPLRT_DIV)
  - Gyro Output Rate is 1KHz if DLPF(Digital Low Pass Filter) is 1 else 8KHz.
  - usually DLPF=1

- INT_PIN_CFG - Interrupt Pin/Bypass enable configuration
  -> {INT_LEVEL, INT_OPEN, LATCH_INT_EN, INT_RD_CLEAR, FSYNC_INT_LEVEL, FSYNC_INT_EN, I2C_BYPASS_EN, -}
  - I2C_BYPASS_EN: Set to access auxillary I2C bus by the master (for QMC)

- TEMP_OUT
  - 16 bit signed register
  - Temperature in degrees C = (TEMP_OUT_signed)/340 + Current temparature
  - TEMP_OUT change 340 for every 1deg change

*/

