#include "params.h"
#include <Preferences.h>

static const float kDefaults[P_COUNT] = {
    0.040f,  // P_ON_THR
    0.025f,  // P_OFF_THR
    0.80f,   // P_CONFIRM_S
    1.0f,    // P_FMIN
    15.0f,   // P_FMAX
    3.0f,    // P_CREST_MAX
    0.50f,   // P_PERIODIC_MIN
    0.60f,   // P_DECAY_MIN
    0.20f,   // P_BASE_RATIO_MIN
    3.0f,    // P_LATCH_S
    1.5f,    // P_QUIET_S
    1.0f,    // P_BUZZER_EN
    0.05f, 0.10f, 0.20f, 0.35f, 0.50f,  // levels
    0.0f,    // P_BUZZER_MODE
};

float g_params[P_COUNT];
Cal g_cal[2];
static Preferences prefs;

static void key(char* out, uint8_t id) { snprintf(out, 8, "p%u", id); }

void paramsLoad() {
  prefs.begin("quake", false);
  for (uint8_t i = 0; i < P_COUNT; i++) {
    char k[8];
    key(k, i);
    g_params[i] = prefs.getFloat(k, kDefaults[i]);
  }
  for (int s = 0; s < 2; s++) {
    Cal c = {{0, 0, 0}, {1, 1, 1}};
    char k[8];
    snprintf(k, sizeof(k), "cal%d", s);
    if (prefs.getBytesLength(k) == sizeof(Cal)) prefs.getBytes(k, &c, sizeof(Cal));
    g_cal[s] = c;
  }
}

void paramsSet(uint8_t id, float v) {
  if (id >= P_COUNT || !isfinite(v)) return;
  g_params[id] = v;
  char k[8];
  key(k, id);
  prefs.putFloat(k, v);
}

void paramsReset() {
  for (uint8_t i = 0; i < P_COUNT; i++) {
    g_params[i] = kDefaults[i];
    char k[8];
    key(k, i);
    prefs.remove(k);
  }
}

void calSet(int sensor, const Cal& c) {
  if (sensor < 0 || sensor > 1) return;
  g_cal[sensor] = c;
  char k[8];
  snprintf(k, sizeof(k), "cal%d", sensor);
  prefs.putBytes(k, &c, sizeof(Cal));
}
