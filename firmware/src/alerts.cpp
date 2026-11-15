#include "alerts.h"
#include "config.h"
#include "detector.h"
#include "params.h"
#include "protocol.h"
#include "sensor.h"

// PWM channels (LEDC on ESP32, pin-bound slices on RP2040): LEDs on 0-2; buzzer alone on channel 6
// (its own LEDC timer / its own RP2040 slice), so changing the buzzer tone never disturbs the LEDs.
constexpr int CH_GREEN = 0, CH_YELLOW = 1, CH_RED = 2, CH_BUZZ = 6;
static volatile uint32_t s_testUntil = 0;
static int s_buzzMode = -1;

void alertsInit() {
  halPwmSetup(CH_GREEN, 5000, 8);
  halPwmSetup(CH_YELLOW, 5000, 8);
  halPwmSetup(CH_RED, 5000, 8);
  halPwmAttachPin(PIN_LED_GREEN, CH_GREEN);
  if (PIN_LED_YELLOW >= 0) halPwmAttachPin(PIN_LED_YELLOW, CH_YELLOW);  // not fitted on RP2040
  halPwmAttachPin(PIN_LED_RED, CH_RED);
  halPwmSetup(CH_BUZZ, 2700, 10);
  halPwmAttachPin(PIN_BUZZER, CH_BUZZ);
  halPwmWrite(CH_BUZZ, 0);
  pinMode(PIN_HEARTBEAT, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
}

void alertsTest(uint32_t ms) { s_testUntil = millis() + ms; }
static volatile uint8_t s_melReq = 0;
constexpr uint32_t SELF_NOISE_HOLD_MS = 1000;  // longer than the detector's 0.5 s window plus filter settling
static volatile uint32_t s_selfNoiseUntil = 0;
bool alertsSelfNoise() { return (int32_t)(s_selfNoiseUntil - millis()) > 0; }
void alertsTestMelody(uint8_t which) { s_melReq = which; }

#if HAL_RP2040
// Two LEDs only: green when quiet, solid red only while the alarm is going off (and the test
// alarm). A candidate that has not been confirmed yet stays green. Start-up blinks green, a sensor
// fault blinks red. Called once per tick.
static void ledsTwoColor(uint8_t state, bool test, float t) {
  uint8_t g = 0, r = 0;
  if (test || state == ST_ALARM) {
    r = 255;
  } else if (state == ST_FAULT) {
    r = fmodf(t, 1.0f) < 0.5f ? 255 : 0;
  } else if (state == ST_INIT) {
    g = fmodf(t * 2.0f, 1.0f) < 0.5f ? 255 : 0;
  } else {
    g = 255;
  }
  halPwmWrite(CH_GREEN, g);
  halPwmWrite(CH_RED, r);
}
static void leds(uint8_t, uint8_t, uint8_t) {}  // the three-colour patterns below are ESP32 only
#else
static void leds(uint8_t g, uint8_t y, uint8_t r) {
  halPwmWrite(CH_GREEN, g);
  halPwmWrite(CH_YELLOW, y);
  halPwmWrite(CH_RED, r);
}
#endif

static void buzzer(bool on, float hz) {
  int mode = g_params[P_BUZZER_MODE] > 0.5f ? 1 : 0;
  if (mode != s_buzzMode) {
    s_buzzMode = mode;
    halPwmSetup(CH_BUZZ, mode ? 1000 : 2700, 10);
    halPwmWrite(CH_BUZZ, 0);
  }
  if (g_params[P_BUZZER_EN] < 0.5f) on = false;
  if (mode == 1) {
    halPwmWrite(CH_BUZZ, on ? 1023 : 0);  // active buzzer: plain on/off
  } else if (on) {
    halPwmTone(CH_BUZZ, (uint32_t)hz);  // passive buzzer: siren tone
  } else {
    halPwmWrite(CH_BUZZ, 0);
  }
}

// ---------------------------------------------------------------- melodies
// The sensor samples at ~788 Hz, so a note folds down to |f - k*ODR| in the sensor data. A note within
// 100 Hz of a multiple of the sample rate would land inside the detector's band and read as shaking
// (see the siren below), so every note is nudged out of those zones. The tunes below already sit
// clear of them at 788 Hz; the nudge only matters if the measured rate drifts.
static float sensorOdr() {
  for (int i = 0; i < NUM_SENSORS; i++)
    if (g_sensors[i].present) return g_sensors[i].rate.rate;
  return ODR_NOMINAL;
}

static float safeHz(float f) {
#if HAL_RP2040
  const float odr = sensorOdr(), margin = 100.0f;
  float r = fmodf(f, odr);
  if (r < margin) f += margin - r;
  else if (r > odr - margin) f -= r - (odr - margin);
#endif
  return f;
}

struct Note {
  uint16_t hz;  // 0 = rest
  uint16_t ms;
};
// Bump: a soft descending two-note "ding-dong".
static const Note kChime[] = {{1319, 120}, {0, 30}, {1047, 260}};
// Shaking has stopped: a bright rising arpeggio.
static const Note kJingle[] = {{1047, 110}, {1319, 110}, {1760, 110}, {2093, 110}, {0, 40}, {1760, 110}, {2093, 340}};
static const Note* s_mel = nullptr;
static int s_melN = 0, s_melI = 0;
static uint32_t s_noteEnd = 0;

static void melodyStart(const Note* m, int n, uint32_t now) {
  s_mel = m;
  s_melN = n;
  s_melI = 0;
  s_noteEnd = now + m[0].ms;
}
// True while a note is sounding; hz receives its (alias-safe) pitch.
static bool melodyTick(uint32_t now, float& hz) {
  if (!s_mel) return false;
  while ((int32_t)(now - s_noteEnd) >= 0) {
    if (++s_melI >= s_melN) {
      s_mel = nullptr;
      return false;
    }
    s_noteEnd += s_mel[s_melI].ms;
  }
  if (s_mel[s_melI].hz == 0) return false;
  hz = safeHz((float)s_mel[s_melI].hz);
  return true;
}

void alertTask(void*) {
  TickType_t last = xTaskGetTickCount();
  uint32_t pressStart = 0;
  bool pressed = false, longDone = false;
  uint32_t lastRaw[NUM_SENSORS] = {0, 0};
  uint32_t hbToggle = 0;
  bool hb = false;
  bool prevAlarm = false, prevSilenced = false;
  uint32_t seenBumps = detectorInfo().bumpCount;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(20));
    halWatchdogFeed();
    uint32_t now = millis();
    DetectorView v = detectorView();
    bool test = (int32_t)(s_testUntil - now) > 0;
    float t = now / 1000.0f;
#if HAL_RP2040
    ledsTwoColor(v.state, test, t);
#endif

    // Melodies: a chime for a rejected bump, a jingle when a real alarm ends (not if it was
    // silenced by the button, and never over a live alarm).
    bool alarmNow = test || v.state == ST_ALARM;
    uint32_t bumps = detectorInfo().bumpCount;
    if (alarmNow) {
      s_mel = nullptr;
    } else if (prevAlarm && !prevSilenced) {
      melodyStart(kJingle, sizeof(kJingle) / sizeof(kJingle[0]), now);
    } else if (bumps != seenBumps) {
      melodyStart(kChime, sizeof(kChime) / sizeof(kChime[0]), now);
    }
    seenBumps = bumps;
    if (s_melReq) {  // bench request (monitor --test-melody)
      if (!alarmNow) {
        if (s_melReq == 1) melodyStart(kChime, sizeof(kChime) / sizeof(kChime[0]), now);
        else melodyStart(kJingle, sizeof(kJingle) / sizeof(kJingle[0]), now);
      }
      s_melReq = 0;
    }
    prevAlarm = v.state == ST_ALARM && !test;
    prevSilenced = v.silenced;
    float melHz = 0;
    bool melOn = !alarmNow && melodyTick(now, melHz);
    if (alarmNow || s_mel) s_selfNoiseUntil = now + SELF_NOISE_HOLD_MS;

    if (test || v.state == ST_ALARM) {
      uint8_t level = test ? 3 : (v.level < 1 ? 1 : v.level);
      float blinkHz = 2.0f + level;                     // faster with severity
      bool on = fmodf(t * blinkHz, 1.0f) < 0.5f;
      leds(0, level >= 3 ? 255 : 0, on ? 255 : 0);
      float sweep = 0.5f + 0.5f * sinf(2.0f * (float)M_PI * t * (0.8f + 0.4f * level));
#if HAL_RP2040
      // The sensor samples at ~788 Hz, so a buzzer tone folds down to |f - k*ODR|. The old 1.8-3.2 kHz
      // sweep crossed 3x and 4x ODR, where the fold lands at 0 Hz, inside the detector's 0.5-25 Hz
      // band, and the sensor read the buzzer as shaking. Sweep only between 3.17x and 3.8x ODR
      // (~2.5-3.0 kHz): the fold stays 130+ Hz away from DC, where the 25 Hz low-pass removes it.
      float hz = (3.17f + 0.63f * sweep) * sensorOdr();
#else
      float hz = 1800.0f + 1400.0f * sweep;             // siren 1.8-3.2 kHz
#endif
      bool active = (g_params[P_BUZZER_MODE] > 0.5f) ? on : true;
      buzzer(!v.silenced || test ? active : false, hz);
    } else if (v.state == ST_CANDIDATE) {
      leds(0, 255, 0);
      buzzer(melOn, melHz);
    } else if (v.state == ST_FAULT) {
      bool a = fmodf(t, 1.0f) < 0.5f;
      leds(0, a ? 255 : 0, a ? 0 : 255);
      buzzer(melOn, melHz);
    } else if (v.state == ST_INIT) {
      bool a = fmodf(t * 2.0f, 1.0f) < 0.5f;
      leds(a ? 255 : 0, a ? 0 : 255, 0);
      buzzer(melOn, melHz);
    } else {  // IDLE: slow breathing green
      float b = 0.2f + 0.8f * (0.5f + 0.5f * sinf(2.0f * (float)M_PI * t / 3.0f));
      leds((uint8_t)(255 * b), 0, 0);
      buzzer(melOn, melHz);
    }

    // heartbeat: blinks only while samples are arriving
    bool flowing = false;
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (g_sensors[i].rawCount != lastRaw[i]) flowing = true;
      lastRaw[i] = g_sensors[i].rawCount;
    }
    if (flowing && now - hbToggle > 250) {
      hb = !hb;
      hbToggle = now;
    } else if (!flowing) {
      hb = false;
    }
    digitalWrite(PIN_HEARTBEAT, hb);

    // BOOT button: short press silences, 2 s hold re-zeroes both sensors
    bool down = digitalRead(PIN_BUTTON) == LOW;
    if (down && !pressed) {
      pressed = true;
      longDone = false;
      pressStart = now;
    } else if (down && pressed && !longDone && now - pressStart > 2000) {
      longDone = true;
      sensorsRequestZero(0x03);
      logf("Button: re-zero requested");
    } else if (!down && pressed) {
      pressed = false;
      if (!longDone && now - pressStart > 30) {
        detectorSilence();
        logf("Button: alarm silenced");
      }
    }
  }
}
