#include "statemachine.h"
#include "config.h"
#include <Arduino.h>

void StateMachine::begin() {
    phase        = FlightPhase::IDLE;
    launchTimeUs = 0;
    confirmCount = 0;
}

void StateMachine::update(const StateEstimate& state) {
    switch (phase) {
        case FlightPhase::IDLE:
            if (state.vertical.a >= SM_LAUNCH_ACCEL_THRESHOLD_MS2) {
                if (++confirmCount >= SM_LAUNCH_CONFIRM_COUNT) {
                    phase        = FlightPhase::LAUNCHED;
                    launchTimeUs = micros();
                    confirmCount = 0;
                }
            } else {
                confirmCount = 0;
            }
            break;

        case FlightPhase::LAUNCHED:
            if ((uint32_t)(micros() - launchTimeUs) >= SM_COAST_DELAY_US) {
                phase = FlightPhase::COASTING;
            }
            break;

        case FlightPhase::COASTING:
            if (state.vertical.v <= 0.0f) {
                phase = FlightPhase::APOGEE;
            }
            break;

        case FlightPhase::APOGEE:
            phase = FlightPhase::DESCENT;
            break;

        case FlightPhase::DESCENT:
            break;
    }
}

FlightPhase StateMachine::getPhase() const {
    return phase;
}

const char* phaseName(FlightPhase phase) {
    switch (phase) {
        case FlightPhase::IDLE:     return "IDLE";
        case FlightPhase::LAUNCHED: return "LAUNCHED";
        case FlightPhase::COASTING: return "COASTING";
        case FlightPhase::APOGEE:   return "APOGEE";
        case FlightPhase::DESCENT:  return "DESCENT";
    }
    return "IDLE";
}
