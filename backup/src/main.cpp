#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP5xx.h"


Adafruit_BMP5xx bmp;

// Pin defines
#define BMP_CS_PIN 0
#define BMI_CS_PIN 0
#define LIS_CS_PIN 0
#define RFM_CS_PIN 0

#define MOSI 0
#define MISO 0

void setup(){
  Serial.begin(115200);
  while(!Serial) delay(10);


  // BMP Setup, see https://github.com/adafruit/Adafruit_BMP5xx/blob/main/examples/bmp5xx_test/bmp5xx_test.ino
  // for more config options.
  
  if (!bmp.begin(BMP_CS_PIN, &SPI)){
    Serial.println("NO BMP FOUND");
    while(1) delay(10);
  }




}

void loop(){
  if(!bmp.dataReady()){
    return;
  }

  if(!bmp.performReading()){
    return;
  }

  Serial.print("Pressure: ");
  Serial.print(bmp.pressure);
  Serial.println(" hPa");
}