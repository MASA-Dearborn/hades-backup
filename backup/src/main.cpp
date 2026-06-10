#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

#include <Adafruit_BMP5xx.h>
#include <Adafruit_LIS2MDL.h>
#include <BMI088.h>
#include <LoRa.h>

#define BMI_GYR_CS 22
#define BMI_ACC_CS 22
// TODO: Label correct pins
#define BMI_INT1   222
#define BMI_INT2   222

#define LIS_CS     19
#define RFM_CS     23
#define RFM_EN     2
#define RFM_INT0   1
#define RFM_RST    0
#define FREQ       433E6


#define MOSI 11
#define MISO 12
#define CLK  13

Adafruit_BMP5xx bmp;
Adafruit_LIS2MDL lis = Adafruit_LIS2MDL(2);
Bmi088Accel accel(SPI, BMI_ACC_CS);
Bmi088Gyro gyro(SPI,BMI_GYR_CS);


/*
TODO:


IMPORTANT:
- [ ] Integrate Airbrakes code
- [ ] Trigger data record to the flash after the rocket launches using a bmi interrupt
  - [ ] BMI interrupt programmed to trigger when G threshold is reached for launch
  - [ ] Testing to make sure flash memory doesn't fill up when recording over the time our rocket is in flight
  - [ ] Way to move the data off the flash after flight
- [ ] Work on integration of radio code into above loop. Make sure it's properly received by the ground station and read by dashboard
  - [ ] Make sure you can't hang the loop via sending data and not having it received

Extra feature requests:
- [ ] Longer term temperature logging to SD card through sensors. NOT ACTIVE DURING FLIGHT. (Writing to SD card might(?) hang the execution loop)

*/

void setup(){
  Serial.begin(9600);
  while(!Serial) delay(1000);
  Wire1.begin();

  pinMode(RFM_RST, OUTPUT);
  digitalWrite(RFM_RST, HIGH);

  digitalWrite(RFM_RST, LOW);
  delay(10);
  digitalWrite(RFM_RST, HIGH);
  delay(10);

  LoRa.setPins(RFM_CS, RFM_RST, RFM_INT0);


  if(!LoRa.begin(FREQ)){
    Serial.println("LoRa radio init failed.");
    while(1);
  }

  Serial.println("LoRa init success!");
  Serial.print("Freq is: "); Serial.println(FREQ);

  LoRa.setTxPower(20);

  // BMP setup, see https://github.com/adafruit/Adafruit_BMP5xx/blob/main/examples/bmp5xx_test/bmp5xx_test.ino
  // for more config options.
  if (!bmp.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire1)){
    Serial.println("NO BMP FOUND");
    while(1) delay(100);
  }
  // LIS setup
  lis.enableAutoRange(true);
  if(!lis.begin_SPI(LIS_CS)){
    Serial.println("NO LIS FOUND");
    while(1) delay(10);
  }


  int status;
  status = accel.begin();
  
  // LoRa Setup

  pinMode(RFM_EN, OUTPUT);
  

  LoRa.setPins(RFM_CS, RFM_RST, RFM_INT0);
  if (!LoRa.begin(433E6)){
    Serial.println("Starting LoRa failed!");
    while (100);
  }

}

void loop(){

  Serial.println("Initialized successfully");
  bmp.performReading();
  Serial.print("Pressure: ");
  Serial.print(bmp.pressure);
  Serial.println(" hPa");

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

  // Send telemetry data to ground
  /*
  // Declare array of data to send to ground
  int numOfThingsBeingSent;
  float data[numOfThingsBeingSent]; // Will have to fill array with what you're sending

  // send data packet to ground
  LoRa.beginPacket();
  LoRa.write((byte*)&data, sizeof(data));
  LoRa.endPacket();
  */

  delay(1000);
}