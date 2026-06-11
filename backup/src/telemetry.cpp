#include "telemetry.h"
#include "config.h"

#include <LoRa.h>
#include <Arduino.h>
#include <math.h>

static uint32_t s_cycles = 0;
static bool s_ready = false;

static float magneticHeadingDeg(float mx, float my) {
  float heading = atan2f(my, mx) * 180.0f / M_PI;
  if (heading < 0.0f) heading += 360.0f;
  return heading;
}

static int32_t gyroToInt32(float radPerS) {
  return (int32_t)lroundf(radPerS * 57.2957795f);
}

bool telemetryInit() {
  pinMode(RFM_RST_PIN, OUTPUT);
  digitalWrite(RFM_RST_PIN, HIGH);
  digitalWrite(RFM_RST_PIN, LOW);
  delay(10);
  digitalWrite(RFM_RST_PIN, HIGH);
  delay(10);

  pinMode(RFM_EN_PIN, OUTPUT);
  digitalWrite(RFM_EN_PIN, HIGH);

  LoRa.setPins(RFM_CS_PIN, RFM_RST_PIN, RFM_INT0_PIN);
  if (!LoRa.begin(LORA_FREQ_HZ)) {
    Serial.println("LoRa init failed");
    return false;
  }

  LoRa.setTxPower(LORA_TX_POWER_DBM);
  Serial.println("LoRa telemetry ready");
  s_ready = true;
  return true;
}

bool telemetrySend(const Payload& packet) {
  if (!s_ready) return false;

  LoRa.beginPacket();
  size_t written = LoRa.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
  bool sent = LoRa.endPacket();
  if (sent && written == sizeof(packet)) {
    s_cycles++;
    return true;
  }
  return false;
}

uint32_t telemetryCycleCount() {
  return s_cycles;
}

Payload buildTelemetryPacket(const StateEstimate& state,
                             const SensorData& sensors,
                             const GuidanceState& guidance,
                             FlightPhase phase) {
  Payload pkt = {};

  pkt.airbrakesData.timestamp = millis();
  pkt.airbrakesData.flight_state = static_cast<uint8_t>(phase);
  pkt.airbrakesData.altitude_est = state.vertical.h;
  pkt.airbrakesData.vel_est = state.vertical.v;
  pkt.airbrakesData.apogee_pred = guidance.estimatedApogeeM;

  pkt.insData.longitude = 0.0;
  pkt.insData.latitude = 0.0;
  pkt.insData.acceleration = sqrtf(
      sensors.ax * sensors.ax + sensors.ay * sensors.ay + sensors.az * sensors.az);
  pkt.insData.magnetic_heading = magneticHeadingDeg(sensors.mx, sensors.my);
  pkt.insData.barometric_pressure = sensors.hpa;

  pkt.piezoData.cycles = telemetryCycleCount() + 1;
  pkt.piezoData.voltage = 0.0f;
  pkt.piezoData.gyro_x = gyroToInt32(sensors.gx);
  pkt.piezoData.gyro_y = gyroToInt32(sensors.gy);
  pkt.piezoData.gyro_z = gyroToInt32(sensors.gz);

  return pkt;
}
