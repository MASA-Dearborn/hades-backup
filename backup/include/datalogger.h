#pragma once

#include "types.h"
#include <stdint.h>

// Fixed-size binary flight record. Layout MUST stay byte-identical to the main
// flight computer's LogRecord so a single ground decoder reads both boards'
// .BIN files. The backup board has no actuator, so targetPositionCm /
// actualPositionCm are always written as 0.
#pragma pack(push, 1)
struct LogRecord {
  uint32_t timeUs;
  uint8_t  phase;
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float hpa, tempC;
  float h, v, a;
  float q0, q1, q2, q3, tiltDeg;
  float estimatedApogeeM, apogeeErrorM, targetPositionCm;
  float actualPositionCm;
};
#pragma pack(pop)
static_assert(sizeof(LogRecord) == 97, "LogRecord must be 97 bytes");

bool     datalogInit();
void     datalogWrite(const StateEstimate& state, const SensorData& sensors,
                      const GuidanceState& guidance, FlightPhase phase);
void     datalogFlush();
void     datalogClose();
bool     datalogDump();
uint32_t datalogEraseAll();
uint32_t datalogRecordCount();
bool     datalogReady();
bool     datalogSessionActive();
