#pragma once

#include "TelemetryPacket.h"
#include "types.h"

bool telemetryInit();
bool telemetrySend(const Payload& packet);
uint32_t telemetryCycleCount();

Payload buildTelemetryPacket(const StateEstimate& state,
                             const SensorData& sensors,
                             const GuidanceState& guidance,
                             FlightPhase phase);
