#include "datalogger.h"
#include "config.h"

#include <Arduino.h>
#include <SdFat.h>

static SdFs sd;
static FsFile file;
static bool s_ready = false;
static uint32_t s_writeCount = 0;
static char s_fname[13] = {};

bool datalogInit() {
  if (!sd.begin(SdioConfig(FIFO_SDIO))) {
    Serial.println("SD: init failed — logging disabled");
    return false;
  }

  int slot = -1;
  for (int i = 0; i < 100; i++) {
    snprintf(s_fname, sizeof(s_fname), "FLT%02d.BIN", i);
    if (!sd.exists(s_fname)) {
      slot = i;
      break;
    }
  }
  if (slot < 0) {
    Serial.println("SD: no free log slots (FLT00–FLT99 all exist)");
    return false;
  }

  if (!file.open(s_fname, O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println("SD: file open failed");
    return false;
  }

  if (!file.preAllocate(LOG_PREALLOC_BYTES)) {
    Serial.println("SD: preAllocate skipped");
  }
  file.rewind();

  Serial.print("SD: logging to ");
  Serial.println(s_fname);
  Serial.print("SD: record size = ");
  Serial.print((int)sizeof(LogRecord));
  Serial.println(" bytes");

  s_ready = true;
  s_writeCount = 0;
  return true;
}

void datalogWrite(const StateEstimate& state, const SensorData& sensors,
                  const GuidanceState& guidance, FlightPhase phase) {
  if (!s_ready) return;

  LogRecord rec = {};
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
  rec.targetPositionCm = 0.0f;
  rec.actualPositionCm = 0.0f;

  if (file.write(&rec, sizeof(rec)) == sizeof(rec)) {
    s_writeCount++;
  }

  if (s_writeCount % DATALOG_FLUSH_INTERVAL == 0) {
    file.flush();
  }
}

void datalogFlush() {
  if (!s_ready) return;
  file.flush();
}

// Clean shutdown ('s' command): truncate away the unused pre-allocated tail so
// the file's real size is committed (without this it shows on the card as a
// fixed LOG_PREALLOC_BYTES blob), then close so the directory entry flushes to
// media. Call before removing the SD card or powering off.
void datalogClose() {
  if (!s_ready) return;
  file.truncate(file.fileSize());
  file.close();
  s_ready = false;
  Serial.print("SD: closed ");
  Serial.println(s_fname);
  Serial.println("Logging stopped — safe to remove SD card");
}

uint32_t datalogRecordCount() {
  return s_writeCount;
}

bool datalogReady() {
  return s_ready;
}

bool datalogSessionActive() {
  return s_ready;
}

bool datalogDump() {
  Serial.println("dump: pull the SD card and read FLTnn.BIN on a computer");
  return false;
}

uint32_t datalogEraseAll() {
  if (!s_ready) {
    if (!sd.begin(SdioConfig(FIFO_SDIO))) {
      Serial.println("erase: SD not mounted");
      return 0;
    }
  } else {
    file.close();
    s_ready = false;
    s_writeCount = 0;
  }

  uint32_t removed = 0;
  char fname[13];
  for (int i = 0; i < 100; i++) {
    snprintf(fname, sizeof(fname), "FLT%02d.BIN", i);
    if (sd.exists(fname) && sd.remove(fname)) removed++;
  }

  Serial.print("Erased ");
  Serial.print(removed);
  Serial.println(" log file(s) — reboot to start a new log");

  return removed;
}
