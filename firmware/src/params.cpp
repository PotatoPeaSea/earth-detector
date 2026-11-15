#include "params.h"
#include "hal.h"

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

#if HAL_RP2040
// ---------------------------------------------------------------------------------------------
// RP2040 has no NVS. Settings live in one checksummed block in the emulated-EEPROM flash page. A
// change updates RAM at once and is written back by paramsPoll() after it has settled, because a
// flash erase halts both cores for tens of milliseconds (which would overrun the sensor FIFOs).
// ---------------------------------------------------------------------------------------------
#include <EEPROM.h>

namespace {
constexpr uint32_t kMagic = 0x51554B00u | P_COUNT;  // "QUK" + parameter count: a layout change resets to defaults
constexpr uint32_t kSettleMs = 1500;

struct Store {
  uint32_t magic;
  float params[P_COUNT];
  Cal cal[2];
  uint8_t calValid[2];
  uint8_t pad[2];
  uint32_t crc;  // FNV-1a over everything above
};

Store s_store;
SpinMux s_mux;
bool s_dirty = false;
uint32_t s_dirtyAt = 0;

uint32_t checksum(const Store& s) {
  const uint8_t* p = (const uint8_t*)&s;
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < offsetof(Store, crc); i++) h = (h ^ p[i]) * 16777619u;
  return h;
}

void markDirty() {
  CritGuard g(s_mux);
  s_dirty = true;
  s_dirtyAt = millis();
}

void storeDefaults() {
  memset(&s_store, 0, sizeof(s_store));
  s_store.magic = kMagic;
  for (int i = 0; i < P_COUNT; i++) s_store.params[i] = kDefaults[i];
}
}  // namespace

void paramsLoad() {
  EEPROM.begin(sizeof(Store));
  Store tmp;
  EEPROM.get(0, tmp);
  if (tmp.magic == kMagic && tmp.crc == checksum(tmp)) {
    s_store = tmp;
  } else {
    storeDefaults();
  }
  for (int i = 0; i < P_COUNT; i++) g_params[i] = isfinite(s_store.params[i]) ? s_store.params[i] : kDefaults[i];
  for (int s = 0; s < 2; s++) {
    Cal c = {{0, 0, 0}, {1, 1, 1}};
    if (s_store.calValid[s]) c = s_store.cal[s];
    g_cal[s] = c;
  }
}

void paramsSet(uint8_t id, float v) {
  if (id >= P_COUNT || !isfinite(v)) return;
  g_params[id] = v;
  {
    CritGuard g(s_mux);
    s_store.params[id] = v;
  }
  markDirty();
}

void paramsReset() {
  for (uint8_t i = 0; i < P_COUNT; i++) g_params[i] = kDefaults[i];
  {
    CritGuard g(s_mux);
    for (uint8_t i = 0; i < P_COUNT; i++) s_store.params[i] = kDefaults[i];
  }
  markDirty();
}

void calSet(int sensor, const Cal& c) {
  if (sensor < 0 || sensor > 1) return;
  g_cal[sensor] = c;
  {
    CritGuard g(s_mux);
    s_store.cal[sensor] = c;
    s_store.calValid[sensor] = 1;
  }
  markDirty();
}

void paramsPoll(bool quiet) {
  Store snap;
  {
    CritGuard g(s_mux);
    if (!s_dirty || !quiet || millis() - s_dirtyAt < kSettleMs) return;
    snap = s_store;
    s_dirty = false;
  }
  snap.crc = checksum(snap);
  EEPROM.put(0, snap);
  EEPROM.commit();
}

#else
// ---------------------------------------------------------------------------------------------
// ESP32: NVS through Preferences, written immediately.
// ---------------------------------------------------------------------------------------------
#include <Preferences.h>

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

void paramsPoll(bool) {}
#endif
