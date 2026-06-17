#pragma once

// LoRa telemetry + sensors (backup payload board). No actuator.

#define GRAVITY 9.80665f

#define BARO_CALIB_SAMPLES 50
#define SEALEVEL_HPA 1013.25f

#define BMP_USE_WIRE1 1

#define LIS_CS_PIN 19
#define BMI_ACC_CS_PIN 22
#define BMI_GYR_CS_PIN 22

#define RFM_CS_PIN 23
#define RFM_EN_PIN 2
#define RFM_INT0_PIN 1
#define RFM_RST_PIN 0
#define LORA_FREQ_HZ 433E6
#define LORA_TX_POWER_DBM 20

#define TELEMETRY_PERIOD_US 100000UL  // 10 Hz
#define LOG_PERIOD_US 20000UL                   // 50 Hz binary flight log
#define DATALOG_FLUSH_INTERVAL 100              // records between fsync (~2 s at 50 Hz)
#define LOG_PREALLOC_BYTES (8UL * 1024 * 1024)  // contiguous reserve on SD

#define IMU_PERIOD_US 5000UL
#define BARO_PERIOD_US 20000UL
#define MAG_PERIOD_US 10000UL
#define OUTER_CTRL_PERIOD_US 50000UL

#define KF_SIGMA_A2 2.73e-3f
#define KF_R 5.71e-3f
#define MADGWICK_BETA 0.1f

#define SM_LAUNCH_ACCEL_THRESHOLD_MS2 30.0f
#define SM_LAUNCH_CONFIRM_COUNT 5
#define SM_COAST_DELAY_US 4400000UL

#define ROCKET_MASS_KG 33.135f
#define ROCKET_REF_AREA_M2 0.01928f
#define ROCKET_TARGET_APOGEE_M 3048.0f
