#include <Arduino.h>
#include "estimation.h"
#include <math.h>

void Estimator::begin(float basePressure) {
    basePressure_hPa = basePressure;
    attitude.begin();
    kalman.begin(0.0f, 0.0f);
    lastImuTimeUs = 0;
}

void Estimator::update(SensorData& data) {
    bool verticalUpdated = false;

    if (data.imuUpdated) {
        if (lastImuTimeUs == 0) {
            lastImuTimeUs = data.imuTimeUs;
        } else {
            float dt = (data.imuTimeUs - lastImuTimeUs) * 1e-6f;
            if (dt > 0.0f && dt < 0.1f) {
                attitude.updateIMU(
                    data.gx, data.gy, data.gz,
                    -data.ax, -data.ay, -data.az,
                    dt
                );
                state.attitude = attitude.getState();
                state.attitude.tiltDeg = computeTiltDeg(state.attitude);

                float aVert = verticalAccel(data, state.attitude);
                kalman.predict(aVert, dt);
                verticalUpdated = true;
            }
            lastImuTimeUs = data.imuTimeUs;
        }
        data.imuUpdated = false;
    }

    if (data.baroUpdated) {
        float hBaro = pressureToAlt(data.hpa, basePressure_hPa);
        kalman.updateBaro(hBaro);
        verticalUpdated = true;
        data.baroUpdated = false;
    }

    if (verticalUpdated) {
        state.vertical = kalman.getState();
    }
}

StateEstimate Estimator::getState() const {
    return state;
}

float pressureToAlt(float p_hPa, float baseP_hPa) {
    return 44330.0f * (1.0f - powf(p_hPa / baseP_hPa, 0.1903f));
}

float verticalAccel(const SensorData& data, const AttitudeState& q) {
    float ax = data.ax;
    float ay = data.ay;
    float az = data.az;

    float aWorldZ =
        2.0f * (q.q1 * q.q3 - q.q0 * q.q2) * ax +
        2.0f * (q.q0 * q.q1 + q.q2 * q.q3) * ay +
        (q.q0 * q.q0 - q.q1 * q.q1 - q.q2 * q.q2 + q.q3 * q.q3) * az;

    return aWorldZ + GRAVITY;
}

float computeTiltDeg(const AttitudeState& q) {
    float zw = 1.0f - 2.0f * (q.q1 * q.q1 + q.q2 * q.q2);
    zw = constrain(zw, -1.0f, 1.0f);
    return acosf(zw) * 180.0f / M_PI;
}
