#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

#include <Adafruit_BMP5xx.h>
#include <Adafruit_LIS2MDL.h>
#include <BMI088.h>

// Pin defines - TODO: get actual pins
#define BMP_CS     0
#define BMI_GYR_CS 0
#define BMI_ACC_CS 0
#define LIS_CS     0
#define RFM_CS     0

#define MOSI 0
#define MISO 0
#define CLK  0

Adafruit_BMP5xx bmp;
Adafruit_LIS2MDL lis = Adafruit_LIS2MDL(2);
Bmi088Accel accel(SPI, BMI_ACC_CS);
Bmi088Gyro gyro(SPI,BMI_GYR_CS);

void setup(){
  Serial.begin(115200);
  while(!Serial) delay(10);

  // BMP setup, see https://github.com/adafruit/Adafruit_BMP5xx/blob/main/examples/bmp5xx_test/bmp5xx_test.ino
  // for more config options.
  
  if (!bmp.begin(BMP_CS, &SPI)){
    Serial.println("NO BMP FOUND");
    while(1) delay(10);
  }

  // LIS setup
  /*
  lis.enableAutoRange(true);
  if(!lis.begin_SPI(LIS_CS)){
    Serial.println("NO LIS FOUND");
    while(1) delay(10);
  }

  int status;
  status = accel.begin();
  */
  


}

void loop(){
  if(bmp.dataReady()){
    bmp.performReading();
    Serial.print("Pressure: ");
    Serial.print(bmp.pressure);
    Serial.println(" hPa");
  }

  /*
  sensors_event_t event;
  lis.getEvent(&event);


  Serial.print("Mag X: ");
  Serial.print(event.magnetic.x, 2); 
  Serial.println(" uT");

  accel.readSensor();
  gyro.readSensor();
  Serial.print(accel.getAccelX_mss());
  Serial.print("\t");
  Serial.println(gyro.getGyroX_rads());
  */
}