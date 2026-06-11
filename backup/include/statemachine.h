#pragma once
#include "types.h"

class StateMachine {
public:
    void begin();
    void update(const StateEstimate& state);
    FlightPhase getPhase() const;

private:
    FlightPhase phase        = FlightPhase::IDLE;
    uint32_t    launchTimeUs = 0;
    int         confirmCount = 0;
};

const char* phaseName(FlightPhase phase);
