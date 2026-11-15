#pragma once
#include <Arduino.h>
#include "hal.h"

#define FW_VERSION "0.1.0"
constexpr uint8_t PROTO_VERSION = 1;

#if HAL_RP2040
// ================= YD-RP2040 (RP2040, GPIO numbers) =================
// ---- Sensor buses (each ADXL345 on its own I2C controller, both at 0x53) ----
// I2C0 takes SDA on GPIO 0/4/8/12/16/20 and SCL on 1/5/9/13/17/21; I2C1 takes SDA on 2/6/10/14/18/26
// and SCL on 3/7/11/15/19/27.
constexpr int S1_SDA = 4, S1_SCL = 5, S1_INT = 6;      // S1: top floor, Wire  (I2C0)
constexpr int S2_SDA = 14, S2_SCL = 15, S2_INT = 13;   // S2: base,      Wire1 (I2C1)
constexpr uint32_t I2C_HZ = 400000;
constexpr uint8_t ADXL_ADDR = 0x53;
constexpr int NUM_SENSORS = 2;

// ---- Alerts and UI pins ----
// Two status LEDs: green = quiet, red = shaking. PWM slices are GPIO/2 mod 8: green (GP8) is slice 4,
// red (GP11) is slice 5, and the buzzer (GP17 = slice 0, channel B) is alone on its slice, so a
// tone change never disturbs the LED PWM. There is no yellow LED on this build (-1 = not fitted).
constexpr int PIN_LED_GREEN = 8;
constexpr int PIN_LED_YELLOW = -1;
constexpr int PIN_LED_RED = 11;
constexpr int PIN_BUZZER = 17;     // passive buzzer: NPN (2N2222) base, ideally through 1 k
constexpr int PIN_HEARTBEAT = 25;  // on-board blue LED
constexpr int PIN_BUTTON = 24;     // on-board KEY button (to GND): short = silence, hold 2 s = re-zero
constexpr int PIN_VBAT = 26;       // ADC0, 100k/100k divider from the 5 V pack (3.3 V reference)

// ---- Round GC9A01 display on SPI0 (write only, no backlight pin on this module) ----
// SPI0 takes SCK on GPIO 2/6/18 and MOSI on 3/7/19.
constexpr int PIN_TFT_DC = 0, PIN_TFT_CS = 1, PIN_TFT_SCK = 2, PIN_TFT_MOSI = 3, PIN_TFT_RST = 7;

// ---- Tasks: the RP2040 FreeRTOS SMP build only has priorities 0-7 (the USB task is 6) ----
// arduino-pico's I2C busy-waits, so each sensor task keeps its core busy for a large share of the
// time (two of them on one core starved everything else and left the buzzer stuck on). Sensors go
// on separate cores and sit BELOW the short, periodic detector and alert tasks, which can then
// never be starved. Core 0 also runs USB and comms; core 1 runs detector and alerts.
constexpr int PRIO_SENSOR = 3, PRIO_COMMS = 4, PRIO_DETECTOR = 5, PRIO_ALERTS = 6, PRIO_BOOT = 3;
// Core 0: USB, comms and the SPI display. Core 1: the sensor task and its interrupt, detector, alerts.
constexpr int CORE_COMMS = 0, CORE_DSP = 1, CORE_BOOT = 0;
constexpr int CORE_SENSOR[2] = {1, 1};
// S1 (top floor) is removed on this build: S2 on the base is the only sensor. The detector already
// falls back to whichever sensor is valid; S1 just shows as missing in the health frame.
constexpr bool SENSOR_FITTED[2] = {false, true};
#else
// ================= ESP32 DevKit (GPIO numbers) =================
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

// ---- Tasks ----
constexpr int PRIO_COMMS = 5, PRIO_ALERTS = 10, PRIO_DETECTOR = 18, PRIO_SENSOR = 20, PRIO_BOOT = 1;
constexpr int CORE_COMMS = 0, CORE_DSP = 1, CORE_BOOT = 1;
constexpr int CORE_SENSOR[2] = {1, 1};
constexpr bool SENSOR_FITTED[2] = {true, true};
#endif

// ---- Sampling ----
constexpr float ODR_NOMINAL = 800.0f;     // ADXL345 BW_RATE 0x0D
constexpr int DECIM = 4;                  // 800 -> 200 Hz
constexpr float FS_DEC_NOMINAL = ODR_NOMINAL / DECIM;
constexpr int FIFO_WATERMARK = 16;
constexpr float LSB_G = 1.0f / 256.0f;    // full resolution: 3.9 mg/LSB at every range
constexpr float MEDIAN_GLITCH_G = 1.5f;   // only samples this far from the 3-sample median are treated as glitches
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
