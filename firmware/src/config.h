#pragma once
#include <Arduino.h>

#define FW_VERSION "0.1.0"
constexpr uint8_t PROTO_VERSION = 1;

// ---- Sensor buses (each ADXL345 on its own I2C controller, both at 0x53) ----
constexpr int S1_SDA = 21, S1_SCL = 22, S1_INT = 13;   // S1: top floor, Wire
constexpr int S2_SDA = 14, S2_SCL = 27, S2_INT = 26;   // S2: base,      Wire1
constexpr uint32_t I2C_HZ = 400000;
constexpr uint8_t ADXL_ADDR = 0x53;
constexpr int NUM_SENSORS = 2;

// ---- Alerts and UI pins (none are boot strapping pins except GPIO 0/2, used only after boot) ----
constexpr int PIN_LED_GREEN = 25;
constexpr int PIN_LED_YELLOW = 32;
constexpr int PIN_LED_RED = 33;
constexpr int PIN_BUZZER = 23;     // NPN base through 1 k
constexpr int PIN_HEARTBEAT = 2;   // on-board LED on most DevKits
constexpr int PIN_BUTTON = 0;      // on-board BOOT button: short = silence, hold 2 s = re-zero
constexpr int PIN_VBAT = 35;       // optional 100k/100k divider from the 5 V pack

// ---- Sampling ----
constexpr float ODR_NOMINAL = 800.0f;     // ADXL345 BW_RATE 0x0D
constexpr int DECIM = 4;                  // 800 -> 200 Hz
constexpr float FS_DEC_NOMINAL = ODR_NOMINAL / DECIM;
constexpr int FIFO_WATERMARK = 16;
constexpr float LSB_G = 1.0f / 256.0f;    // full resolution: 3.9 mg/LSB at every range
constexpr int FIR_TAPS = 31;
constexpr int FIR_DELAY = (FIR_TAPS - 1) / 2;  // raw samples of group delay
constexpr float HP_HZ = 0.5f;             // removes gravity and tilt
constexpr float LP_HZ = 25.0f;            // removes high-frequency junk
constexpr float OUT_SCALE = 2000.0f;      // counts per g in DATA frames (0.5 mg resolution, +/-16 g)
constexpr int DATA_PER_FRAME = 10;        // decimated samples per DATA frame (20 frames/s/sensor)
constexpr int RAW_PER_FRAME = 16;         // raw samples per RAW frame
constexpr int ZERO_SAMPLES = 400;         // 2 s at 200 Hz for gravity alignment
constexpr int WARMUP_SAMPLES = 200;       // 1 s before features are trusted

// ---- Serial ----
constexpr uint32_t SERIAL_BAUD = 921600;
