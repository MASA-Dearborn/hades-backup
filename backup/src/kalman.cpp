#include "kalman.h"
#include "config.h"
#include <Arduino.h>

void VerticalKalman::begin(float h0, float v0) {
    h = h0;
    v = v0;
    a = 0.0f;
    P00 = 1.0f;
    P01 = 0.0f;
    P10 = 0.0f;
    P11 = 1.0f;
}

void VerticalKalman::predict(float aVert, float dt) {
    h = h + v * dt + 0.5f * aVert * dt * dt;
    v = v + aVert * dt;

    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt2 * dt2;

    float Q00 = KF_SIGMA_A2 * dt4 / 4.0f;
    float Q01 = KF_SIGMA_A2 * dt3 / 2.0f;
    float Q10 = Q01;
    float Q11 = KF_SIGMA_A2 * dt2;

    float P00_new = P00 + dt * (P10 + P01) + dt2 * P11 + Q00;
    float P01_new = P01 + dt * P11 + Q01;
    float P10_new = P10 + dt * P11 + Q10;
    float P11_new = P11 + Q11;

    P00 = P00_new;
    P01 = P01_new;
    P10 = P10_new;
    P11 = P11_new;
    a = aVert;
}

void VerticalKalman::updateBaro(float hBaro) {
    float S = P00 + KF_R;
    if (S < 1e-6f) return;

    float K0 = P00 / S;
    float K1 = P10 / S;
    float innovation = hBaro - h;

    h = h + K0 * innovation;
    v = v + K1 * innovation;

    float P00_old = P00;
    float P01_old = P01;
    float P10_old = P10;
    float P11_old = P11;

    P00 = (1.0f - K0) * P00_old;
    P01 = (1.0f - K0) * P01_old;
    P10 = P10_old - K1 * P00_old;
    P11 = P11_old - K1 * P01_old;
}

VerticalState VerticalKalman::getState() const {
    VerticalState out;
    out.h = h;
    out.v = v;
    out.a = a;
    out.timeUs = micros();
    return out;
}
