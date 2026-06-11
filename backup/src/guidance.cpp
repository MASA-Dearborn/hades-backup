#include "guidance.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

static const float MACH_AXIS[8] = {
    0.25f, 0.50f, 0.75f, 0.80f, 0.85f, 0.90f, 0.95f, 1.00f
};
static const float OPEN_AXIS[4] = { 0.0f, 0.3333f, 0.6667f, 1.0f };
static const float CD_TABLE[8][4] = {
    { 0.336f, 0.594f, 1.012f, 1.298f },
    { 0.304f, 0.581f, 1.051f, 1.352f },
    { 0.289f, 0.624f, 1.202f, 1.529f },
    { 0.287f, 0.647f, 1.254f, 1.583f },
    { 0.286f, 0.686f, 1.318f, 1.649f },
    { 0.288f, 0.754f, 1.415f, 1.740f },
    { 0.338f, 0.854f, 1.596f, 1.908f },
    { 0.450f, 1.013f, 1.805f, 2.177f },
};

static constexpr int N_MACH = 8;
static constexpr int N_OPEN = 4;

float Guidance::lookupCd(float mach, float opening) const {
    mach    = constrain(mach, MACH_AXIS[0], MACH_AXIS[N_MACH - 1]);
    opening = constrain(opening, OPEN_AXIS[0], OPEN_AXIS[N_OPEN - 1]);

    int mi = N_MACH - 2;
    for (int i = 0; i < N_MACH - 1; i++) {
        if (mach <= MACH_AXIS[i + 1]) { mi = i; break; }
    }

    int oi = N_OPEN - 2;
    for (int i = 0; i < N_OPEN - 1; i++) {
        if (opening <= OPEN_AXIS[i + 1]) { oi = i; break; }
    }

    float tm = (mach - MACH_AXIS[mi]) / (MACH_AXIS[mi + 1] - MACH_AXIS[mi]);
    float to = (opening - OPEN_AXIS[oi]) / (OPEN_AXIS[oi + 1] - OPEN_AXIS[oi]);

    float c00 = CD_TABLE[mi][oi];
    float c10 = CD_TABLE[mi + 1][oi];
    float c01 = CD_TABLE[mi][oi + 1];
    float c11 = CD_TABLE[mi + 1][oi + 1];

    return c00 * (1.0f - tm) * (1.0f - to)
         + c10 * tm * (1.0f - to)
         + c01 * (1.0f - tm) * to
         + c11 * tm * to;
}

float Guidance::predictApogee(float h, float v, float Cd, float rho) const {
    if (v <= 0.0f) return h;
    float k = (rho * Cd * ROCKET_REF_AREA_M2) / (4.0f * ROCKET_MASS_KG);
    float dh = (v * v) / (2.0f * GRAVITY + k * v * v);
    return h + dh;
}

void Guidance::begin() {}

GuidanceState Guidance::update(const StateEstimate& state,
                                const SensorData& sensors,
                                float currentOpening) {
    GuidanceState out = {};
    out.timeUs = state.vertical.timeUs;

    float v = state.vertical.v;
    float h = state.vertical.h;

    float T_K = sensors.tempC + 273.15f;
    float P_Pa = sensors.hpa * 100.0f;
    float rho = P_Pa / (287.05f * T_K);
    float soundSpeed = 20.05f * sqrtf(T_K);
    float mach = (v > 0.0f && soundSpeed > 0.0f) ? (v / soundSpeed) : 0.0f;

    float Cd = lookupCd(mach, currentOpening);
    out.estimatedApogeeM = predictApogee(h, v, Cd, rho);
    out.apogeeErrorM = out.estimatedApogeeM - ROCKET_TARGET_APOGEE_M;

    return out;
}
