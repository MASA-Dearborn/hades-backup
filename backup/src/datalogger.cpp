#include "datalogger.h"
#include "config.h"
#include <Arduino.h>
#include <SdFat.h>

static SdFs   sd;
static FsFile file;
static bool   s_ready = false;
static bool   s_sessionActive = false;
static uint32_t s_writeCount = 0;

bool datalogInit() {
    if (!sd.begin(SdioConfig(FIFO_SDIO))) {
        Serial.println("SD: init failed — logging disabled");
        return false;
    }

    // Find the next unused filename FLT00.BIN … FLT99.BIN
    char fname[13];
    int slot = -1;

    for (int i = 0; i < 100; i++) {
        snprintf(fname, sizeof(fname), "FLT%02d.BIN", i);
        if (!sd.exists(fname)) {
            slot = i;
            break;
        }
    }

    if (slot < 0) {
        Serial.println("SD: no free log slots (FLT00–FLT99 all exist)");
        return false;
    }

    if (!file.open(fname, O_RDWR | O_CREAT | O_TRUNC)) {
        Serial.println("SD: file open failed");
        return false;
    }

    // Pre-allocate contiguous space to eliminate cluster-allocation delays
    // during flight. Non-fatal if the card doesn't support it.
    if (!file.preAllocate(LOG_PREALLOC_BYTES)) {
        Serial.println("SD: preAllocate skipped");
    }

    // Rewind to the start of the pre-allocated region so writes begin at byte 0
    file.rewind();

    Serial.print("SD: logging to ");
    Serial.println(fname);
    Serial.print("SD: record size = ");
    Serial.print((int)sizeof(LogRecord));
    Serial.println(" bytes");

    s_ready = true;
    s_sessionActive = true;
    s_writeCount = 0;

    return true;
}

void datalogWrite(const StateEstimate& state,
                  const SensorData& sensors,
                  const GuidanceState& guidance,
                  FlightPhase phase) {
    if (!s_ready || !s_sessionActive) return;

    LogRecord rec;

    rec.timeUs = state.vertical.timeUs;
    rec.phase = static_cast<uint8_t>(phase);

    rec.ax = sensors.ax;
    rec.ay = sensors.ay;
    rec.az = sensors.az;

    rec.gx = sensors.gx;
    rec.gy = sensors.gy;
    rec.gz = sensors.gz;

    rec.mx = sensors.mx;
    rec.my = sensors.my;
    rec.mz = sensors.mz;

    rec.hpa = sensors.hpa;
    rec.tempC = sensors.tempC;

    rec.h = state.vertical.h;
    rec.v = state.vertical.v;
    rec.a = state.vertical.a;

    rec.q0 = state.attitude.q0;
    rec.q1 = state.attitude.q1;
    rec.q2 = state.attitude.q2;
    rec.q3 = state.attitude.q3;
    rec.tiltDeg = state.attitude.tiltDeg;

    rec.estimatedApogeeM = guidance.estimatedApogeeM;
    rec.apogeeErrorM = guidance.apogeeErrorM;

    // Backup board has no actuator.
    rec.targetPositionCm = 0.0f;
    rec.actualPositionCm = 0.0f;

    size_t written = file.write(&rec, sizeof(rec));
    if (written != sizeof(rec)) {
        Serial.println("SD: write failed");
        s_ready = false;
        return;
    }

    s_writeCount++;

    // Periodic flush every 100 records.
    // At 50 Hz this is about every 2 seconds.
    if (s_writeCount % 100 == 0) {
        file.flush();
    }
}

void datalogFlush() {
    if (!s_ready) return;
    file.flush();
}

void datalogClose() {
    if (!s_ready) return;

    file.flush();
    file.truncate();
    file.close();

    s_ready = false;
    s_sessionActive = false;
}

bool datalogReady() {
    return s_ready;
}

bool datalogSessionActive() {
    return s_sessionActive;
}

uint32_t datalogRecordCount() {
    return s_writeCount;
}