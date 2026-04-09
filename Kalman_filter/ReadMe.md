# 📡 IMU + Magnetometer Sensor Fusion using Kalman Filter (Arduino)

## 📌 Overview

This project implements a **sensor fusion system** using:

* **MPU6050 (Accelerometer + Gyroscope)**
* **QMC5883L (Magnetometer / Compass)**

The system estimates:

* **Roll and Pitch** using a **Kalman Filter**
* **Yaw (Heading)** using a **tilt-compensated magnetometer**

---

## ⚙️ Hardware Requirements

* Arduino (Uno / Nano / compatible)
* MPU6050 IMU sensor
* QMC5883L Magnetometer
* Jumper wires

---

## 🔌 Connections

### MPU6050 (I2C)

| MPU6050 | Arduino   |
| ------- | --------- |
| VCC     | 3.3V      |
| GND     | GND       |
| SDA     | A4        |
| SCL     | A5        |

### QMC5883L (I2C - shared bus)

| QMC5883L | Arduino   |
| -------- | --------- |
| VCC      | 3.3V      |
| GND      | GND       |
| SDA      | A4        |
| SCL      | A5        |

---

## 🧠 Core Concepts

### 1. Sensor Fusion

* Combines **gyro (fast but drifting)** and **accelerometer (stable but noisy)**
* Uses a **Kalman filter** to estimate accurate roll and pitch

---

## 📂 Code Structure

### 🔹 `gyro_signals()`

* Reads:

  * Accelerometer data
  * Gyroscope data
* Converts raw values into:

  * Angular rates (°/s)
  * Tilt angles (Roll, Pitch)

---

### 🔹 `kalman_1d()`

Implements a **1D Kalman Filter**:

* Predicts angle using gyro rate
* Corrects using accelerometer measurement

---

### 🔹 `setup()`

* Initializes:

  * Serial communication
  * I2C bus
  * MPU6050 and QMC5883L
* Performs **gyro calibration** (2000 samples)

---

### 🔹 `loop()`

1. Read IMU data
2. Remove gyro bias
3. Apply Kalman filter → Roll & Pitch
4. Read magnetometer
5. Perform tilt compensation
6. Compute heading
7. Print all values

---

### Magnetometer Declination

```
compass.setMagneticDeclination(14, 3);
```

* Set for **Chennai (~14° 3')**
* Adjust based on your location

---

## ⏱ Timing

* Loop runs at **250 Hz (4000 µs per cycle)**
* Ensures stable Kalman filtering

---

## ⚠️ Known Limitations

* No magnetometer calibration (hard/soft iron errors not compensated)
* Kalman filter is simplified (1D per axis)
* Yaw is not filtered (only magnetometer-based)

---
