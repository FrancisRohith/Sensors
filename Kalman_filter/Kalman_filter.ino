#include <Wire.h>
#include <QMC5883LCompass.h>

#define PI 3.141592653589793

QMC5883LCompass compass;

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};
float heading;

void kalman_1d(
  float &KalmanState,
  float &KalmanUncertainty, 
  float KalmanInput,
  float KalmanMeasurement) {
    KalmanState = KalmanState + 0.01 * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + 0.01 * 0.01 * 5 * 5;
    float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 4 * 4);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission(); 

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4096 - 0.05;
  AccY = (float)AccYLSB / 4096 + 0.01;
  AccZ = (float)AccZLSB / 4096 - 0.11;

  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / PI);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / PI);
}

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  compass.init();
  compass.setMagneticDeclination(14, 3);  // Chennai declination
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }

  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

  LoopTimer = micros();
}

void loop() {
  gyro_signals();

  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  compass.read();
  float rollRad = KalmanAngleRoll * (PI / 180.0);
  float pitchRad = KalmanAnglePitch * (PI / 180.0);

  float magX = compass.getX();    
  float magY = compass.getY();    
  float magZ = compass.getZ();    
  float cosPitch = cos(rollRad);
  float sinPitch = sin(rollRad);
  float cosRoll = cos(pitchRad);
  float sinRoll = sin(pitchRad);

  float Xh = magX * cosRoll + magZ * sinPitch;
  float Yh = magX * sinRoll * sinPitch + magY * cosRoll - magZ * sinRoll * cosPitch;

  float heading = atan2(-Yh, Xh);
  if (heading < 0)
      heading += 2 * PI;
  heading = heading * (180.0 / PI);

  Serial.print("Roll: ");
  Serial.print(KalmanAngleRoll);
  Serial.print(" Pitch: ");
  Serial.print(KalmanAnglePitch);
  Serial.print(" Xh: ");
  Serial.print(Xh);
  Serial.print(" Yh: ");
  Serial.print(Yh);
  Serial.print(" Heading: ");
  Serial.println(heading);

  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
