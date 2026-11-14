#pragma once
#include <Arduino.h>

// Tunable detector settings, stored in flash (NVS) and settable from the dashboard.
enum ParamId : uint8_t {
  P_ON_THR = 0,       // g RMS that starts a candidate
  P_OFF_THR,          // g RMS that ends it (hysteresis)
  P_CONFIRM_S,        // seconds above threshold before an alarm can fire
  P_FMIN,             // Hz, shake band
  P_FMAX,
  P_CREST_MAX,        // peak/RMS limit (bumps are spiky)
  P_PERIODIC_MIN,     // autocorrelation at one period
  P_DECAY_MIN,        // newest/previous RMS ratio (bumps decay)
  P_BASE_RATIO_MIN,   // base RMS / top RMS (top-only motion = someone touched the building)
  P_LATCH_S,          // minimum alarm time
  P_QUIET_S,          // quiet time before re-arming
  P_BUZZER_EN,        // 1 = buzzer on
  P_LVL1, P_LVL2, P_LVL3, P_LVL4, P_LVL5,  // g RMS for severity levels 1..5
  P_BUZZER_MODE,      // 0 = passive (tone), 1 = active (on/off)
  P_COUNT
};

extern float g_params[P_COUNT];

void paramsLoad();
void paramsSet(uint8_t id, float v);
void paramsReset();

struct Cal {
  float off[3];   // g
  float gain[3];
};
extern Cal g_cal[2];
void calSet(int sensor, const Cal& c);
