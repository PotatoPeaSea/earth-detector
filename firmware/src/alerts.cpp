#include "alerts.h"
#include "config.h"
#include "detector.h"
#include "params.h"
#include "protocol.h"
#include "sensor.h"

// LEDC channels: LEDs on 0-2 (timers 0 and 1); buzzer alone on channel 6 (timer 3),
// so changing the buzzer tone never disturbs the LED PWM.
constexpr int CH_GREEN = 0, CH_YELLOW = 1, CH_RED = 2, CH_BUZZ = 6;
static volatile uint32_t s_testUntil = 0;
static int s_buzzMode = -1;

void alertsInit() {
  ledcSetup(CH_GREEN, 5000, 8);
  ledcSetup(CH_YELLOW, 5000, 8);
  ledcSetup(CH_RED, 5000, 8);
  ledcAttachPin(PIN_LED_GREEN, CH_GREEN);
  ledcAttachPin(PIN_LED_YELLOW, CH_YELLOW);
  ledcAttachPin(PIN_LED_RED, CH_RED);
  ledcSetup(CH_BUZZ, 2700, 10);
  ledcAttachPin(PIN_BUZZER, CH_BUZZ);
  ledcWrite(CH_BUZZ, 0);
  pinMode(PIN_HEARTBEAT, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
}

void alertsTest(uint32_t ms) { s_testUntil = millis() + ms; }

static void leds(uint8_t g, uint8_t y, uint8_t r) {
  ledcWrite(CH_GREEN, g);
  ledcWrite(CH_YELLOW, y);
  ledcWrite(CH_RED, r);
}

static void buzzer(bool on, float hz) {
  int mode = g_params[P_BUZZER_MODE] > 0.5f ? 1 : 0;
  if (mode != s_buzzMode) {
    s_buzzMode = mode;
    ledcSetup(CH_BUZZ, mode ? 1000 : 2700, 10);
    ledcWrite(CH_BUZZ, 0);
  }
  if (g_params[P_BUZZER_EN] < 0.5f) on = false;
  if (mode == 1) {
    ledcWrite(CH_BUZZ, on ? 1023 : 0);  // active buzzer: plain on/off
  } else if (on) {
    ledcWriteTone(CH_BUZZ, (uint32_t)hz);  // passive buzzer: siren tone
  } else {
    ledcWrite(CH_BUZZ, 0);
  }
}

void alertTask(void*) {
  TickType_t last = xTaskGetTickCount();
  uint32_t pressStart = 0;
  bool pressed = false, longDone = false;
  uint32_t lastRaw[NUM_SENSORS] = {0, 0};
  uint32_t hbToggle = 0;
  bool hb = false;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(20));
    uint32_t now = millis();
    DetectorView v = detectorView();
    bool test = (int32_t)(s_testUntil - now) > 0;
    float t = now / 1000.0f;

    if (test || v.state == ST_ALARM) {
      uint8_t level = test ? 3 : (v.level < 1 ? 1 : v.level);
      float blinkHz = 2.0f + level;                     // faster with severity
      bool on = fmodf(t * blinkHz, 1.0f) < 0.5f;
      leds(0, level >= 3 ? 255 : 0, on ? 255 : 0);
      float sweep = 0.5f + 0.5f * sinf(2.0f * (float)M_PI * t * (0.8f + 0.4f * level));
      float hz = 1800.0f + 1400.0f * sweep;             // siren 1.8-3.2 kHz
      bool active = (g_params[P_BUZZER_MODE] > 0.5f) ? on : true;
      buzzer(!v.silenced || test ? active : false, hz);
    } else if (v.state == ST_CANDIDATE) {
      leds(0, 255, 0);
      buzzer(false, 0);
    } else if (v.state == ST_FAULT) {
      bool a = fmodf(t, 1.0f) < 0.5f;
      leds(0, a ? 255 : 0, a ? 0 : 255);
      buzzer(false, 0);
    } else if (v.state == ST_INIT) {
      bool a = fmodf(t * 2.0f, 1.0f) < 0.5f;
      leds(a ? 255 : 0, a ? 0 : 255, 0);
      buzzer(false, 0);
    } else {  // IDLE: slow breathing green
      float b = 0.2f + 0.8f * (0.5f + 0.5f * sinf(2.0f * (float)M_PI * t / 3.0f));
      leds((uint8_t)(255 * b), 0, 0);
      buzzer(false, 0);
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
