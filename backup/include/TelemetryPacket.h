#pragma once

#include <stdint.h>

// Ground receiver binary layout. Must match decoder on ground station.
#pragma pack(push, 1)
struct AirbrakesData {
  uint32_t timestamp;
  uint8_t flight_state;
  float altitude_est;
  float vel_est;
  float apogee_pred;
};

struct INSData {
  double longitude;
  double latitude;
  float acceleration;
  float magnetic_heading;
  float barometric_pressure;
};

struct PiezoData {
  uint32_t cycles;
  float voltage;
  int32_t gyro_x;
  int32_t gyro_y;
  int32_t gyro_z;
};

struct Payload {
  AirbrakesData airbrakesData;
  INSData insData;
  PiezoData piezoData;
};
#pragma pack(pop)

static_assert(sizeof(Payload) == 65, "Payload must be 65 bytes");
