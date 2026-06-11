#include <Arduino.h>

#include "config.h"
#include "sensors.h"
#include "estimation.h"
#include "statemachine.h"
#include "guidance.h"
#include "telemetry.h"

static float basePressure_hPa;
static SensorData data = {};
static StateEstimate state = {};
static GuidanceState guidanceState = {};
static Estimator estimator;
static StateMachine sm;
static Guidance guidance;

static FlightPhase phase = FlightPhase::IDLE;

static uint32_t lastImuUs = 0;
static uint32_t lastBaroUs = 0;
static uint32_t lastMagUs = 0;
static uint32_t lastOuterUs = 0;
static uint32_t lastTelemUs = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  sensorsInit();
  basePressure_hPa = calibrateBaroBase();

  estimator.begin(basePressure_hPa);
  sm.begin();
  guidance.begin();

  if (!telemetryInit()) {
    Serial.println("LoRa init failed — telemetry disabled");
  }

  uint32_t now = micros();
  lastImuUs = now;
  lastBaroUs = now + 3000;
  lastMagUs = now + 1000;
  lastOuterUs = now + 7000;
  lastTelemUs = now + 13000;

  Serial.println("LoRa telemetry transmitter ready");
}

void loop() {
  uint32_t now = micros();

  if ((uint32_t)(now - lastImuUs) >= IMU_PERIOD_US) {
    lastImuUs += IMU_PERIOD_US;
    data.imuUpdated = false;
    readIMU(data);
    estimator.update(data);
    state = estimator.getState();
  }

  if ((uint32_t)(now - lastBaroUs) >= BARO_PERIOD_US) {
    lastBaroUs += BARO_PERIOD_US;
    data.baroUpdated = false;
    readBaro(data);
    estimator.update(data);
    state = estimator.getState();
  }

  if ((uint32_t)(now - lastMagUs) >= MAG_PERIOD_US) {
    lastMagUs += MAG_PERIOD_US;
    data.magUpdated = false;
    readMag(data);
  }

  if ((uint32_t)(now - lastOuterUs) >= OUTER_CTRL_PERIOD_US) {
    lastOuterUs += OUTER_CTRL_PERIOD_US;
    sm.update(state);
    phase = sm.getPhase();
    guidanceState = guidance.update(state, data, 0.0f);
  }

  // if ((uint32_t)(now - lastTelemUs) >= TELEMETRY_PERIOD_US) {
  //   lastTelemUs += TELEMETRY_PERIOD_US;
  //   Payload pkt = buildTelemetryPacket(state, data, guidanceState, phase);
  //   if (!telemetrySend(pkt)) {
  //     Serial.println("LoRa telemetry send failed");
  //   }
  // }
}
