#include "sensors.h"
#include "config.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP5xx.h>
#include <BMI088.h>
#include <Adafruit_LIS2MDL.h>
#include <Arduino.h>

Adafruit_BMP5xx bmp;
Bmi088Accel accel(SPI, BMI_ACC_CS_PIN);
Bmi088Gyro gyro(SPI, BMI_GYR_CS_PIN);
Adafruit_LIS2MDL mag(2);

static void deselectSpiDevices() {
  pinMode(LIS_CS_PIN, OUTPUT);
  pinMode(BMI_ACC_CS_PIN, OUTPUT);
  pinMode(BMI_GYR_CS_PIN, OUTPUT);
  pinMode(RFM_CS_PIN, OUTPUT);

  digitalWrite(LIS_CS_PIN, HIGH);
  digitalWrite(BMI_ACC_CS_PIN, HIGH);
  digitalWrite(BMI_GYR_CS_PIN, HIGH);
  digitalWrite(RFM_CS_PIN, HIGH);
}

void sensorsInit() {
  Serial.println("Initializing sensors...");

#if BMP_USE_WIRE1
  Wire1.begin();
#endif

  deselectSpiDevices();
  SPI.begin();
  delay(100);

#if BMP_USE_WIRE1
  if (bmp.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire1)) {
    Serial.println("BMP I2C (Wire1) ok");
  } else {
    Serial.println("BMP I2C (Wire1) NOT found");
  }
#endif

  if (accel.begin()) {
    Serial.println("BMI088 Accel SPI ok");
  } else {
    Serial.println("BMI088 Accel SPI NOT found");
  }

  if (gyro.begin()) {
    Serial.println("BMI088 Gyro SPI ok");
  } else {
    Serial.println("BMI088 Gyro SPI NOT found");
  }

  mag.enableAutoRange(true);
  if (mag.begin_SPI(LIS_CS_PIN)) {
    Serial.println("LIS2MDL Mag SPI ok");
  } else {
    Serial.println("LIS2MDL Mag SPI NOT found");
  }

  Serial.println("Sensor init done.");
}

float calibrateBaroBase() {
  float sum = 0.0f;
  int count = 0;
  SensorData temp = {};

  for (int i = 0; i < BARO_CALIB_SAMPLES; i++) {
    readBaro(temp);
    if (temp.baroUpdated) {
      sum += temp.hpa;
      count++;
    }
    delay(20);
  }

  if (count == 0) {
    Serial.println("Baro calibration failed, using sea level.");
    return SEALEVEL_HPA;
  }

  Serial.println("Baro calibrated to local altitude");
  return sum / count;
}

void readIMU(SensorData& data) {
  accel.readSensor();
  gyro.readSensor();

  data.ax = accel.getAccelX_mss();
  data.ay = accel.getAccelY_mss();
  data.az = accel.getAccelZ_mss();

  data.gx = gyro.getGyroX_rads();
  data.gy = gyro.getGyroY_rads();
  data.gz = gyro.getGyroZ_rads();

  data.imuTimeUs = micros();
  data.imuUpdated = true;
}

void readBaro(SensorData& data) {
  if (bmp.performReading()) {
    data.hpa = bmp.pressure;
    data.tempC = bmp.temperature;
    data.baroTimeUs = micros();
    data.baroUpdated = true;
  } else {
    data.baroUpdated = false;
  }
}

void readMag(SensorData& data) {
  sensors_event_t event;
  if (mag.getEvent(&event)) {
    data.mx = event.magnetic.x;
    data.my = event.magnetic.y;
    data.mz = event.magnetic.z;
    data.magTimeUs = micros();
    data.magUpdated = true;
  } else {
    data.magUpdated = false;
  }
}
