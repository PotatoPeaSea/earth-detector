#include "detector.h"
#include <esp_timer.h>
#include "params.h"
#include "protocol.h"
#include "sensor.h"

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint8_t s_state = ST_INIT;
static volatile uint8_t s_level = 0;
static volatile bool s_silenced = false;

uint8_t detectorState() { return s_state; }
DetectorView detectorView() {
  portENTER_CRITICAL(&s_mux);
  DetectorView v = {s_state, s_level, s_silenced};
  portEXIT_CRITICAL(&s_mux);
  return v;
}
void detectorSilence() { s_silenced = true; }

// ---------------------------------------------------------------- event bookkeeping
struct EventAcc {
  uint32_t id = 0;
  int64_t tStart = 0;
  float duration = 0;
  float peak = 0, rmsMax = 0, pgaBase = 0;
  float freqSum = 0;
  int freqN = 0;
  uint8_t level = 0;
  // checks captured at the strongest moment of the event
  uint8_t pass = 0, eval = 0;
  float crest = 0, periodicity = 0, decay = 0, baseRatio = 0, freqAtMax = 0;
};

static EventAcc ev;
static uint32_t s_eventCount = 0;

static uint8_t levelFor(float rms) {
  uint8_t l = 0;
  for (int i = 0; i < 5; i++)
    if (rms >= g_params[P_LVL1 + i]) l = i + 1;
  return l;
}

static float mmiFromPga(float pgaG) {
  float cms2 = pgaG * 980.665f;
  if (cms2 <= 0.01f) return 1.0f;
  float m = 3.66f * log10f(cms2) - 1.66f;  // Wald et al. 1999
  return fmaxf(1.0f, fminf(10.0f, m));
}

static void emitEvent(uint8_t verdict) {
  float freq = ev.freqN ? ev.freqSum / ev.freqN : ev.freqAtMax;
  float pga = ev.pgaBase > 0 ? ev.pgaBase : ev.peak;
  float disp = 0;
  if (freq > 0.2f) {
    float w = 2.0f * (float)M_PI * freq;
    disp = pga * 9.80665f / (w * w) * 1000.0f;  // mm, peak
  }
  PayloadWriter w;
  w.u32(ev.id);
  w.u8(verdict);
  w.u8(ev.pass);
  w.u8(ev.eval);
  w.u8(ev.level);
  w.f32(ev.duration);
  w.f32(ev.peak);
  w.f32(ev.rmsMax);
  w.f32(freq);
  w.f32(ev.crest);
  w.f32(ev.periodicity);
  w.f32(ev.decay);
  w.f32(ev.baseRatio);
  w.f32(pga);
  w.f32(mmiFromPga(pga));
  w.f32(disp);
  w.u32((uint32_t)ev.tStart);
  sendFrame(F_EVENT, w);
}

// ---------------------------------------------------------------- state machine
static void detectorStep(float dt, uint8_t& passOut, uint8_t& evalOut, float& durOut, Features& P,
                         Features& B, bool& haveBase) {
  bool ok0 = sensorOk(0), ok1 = sensorOk(1);
  bool pres0 = g_sensors[0].present, pres1 = g_sensors[1].present;
  int prim = pres0 ? 0 : (pres1 ? 1 : -1);
  haveBase = false;
  passOut = evalOut = 0;
  durOut = 0;
  if (prim < 0) {
    s_state = ST_FAULT;
    return;
  }
  P = sensorFeatures(prim);
  if (prim == 0 && ok1) {
    B = sensorFeatures(1);
    haveBase = B.valid;
  }
  if (!(prim == 0 ? ok0 : ok1)) {
    if (s_state != ST_ALARM) s_state = ST_INIT;  // warming up / re-zeroing
    return;
  }

  static float candDur = 0, quiet = 0, alarmT = 0;
  const float on = g_params[P_ON_THR], off = g_params[P_OFF_THR];

  // evaluate every check on the current window
  uint8_t eval = 0, pass = 0;
  auto chk = [&](int bit, bool okc) {
    eval |= 1 << bit;
    if (okc) pass |= 1 << bit;
  };
  chk(CK_DURATION, candDur >= g_params[P_CONFIRM_S]);
  chk(CK_STEADY, P.decay >= g_params[P_DECAY_MIN]);
  chk(CK_FREQ, P.freq >= g_params[P_FMIN] && P.freq <= g_params[P_FMAX] && P.freqQ >= 0.5f);
  chk(CK_CREST, P.crest > 0 && P.crest <= g_params[P_CREST_MAX]);
  chk(CK_PERIODIC, P.periodicity >= g_params[P_PERIODIC_MIN]);
  float baseRatio = 0;
  if (haveBase) {
    baseRatio = P.rms > 1e-6f ? B.rms / P.rms : 0;
    chk(CK_BASE, baseRatio >= g_params[P_BASE_RATIO_MIN]);
  }
  passOut = pass;
  evalOut = eval;

  auto track = [&]() {
    if (P.peak > ev.peak) ev.peak = P.peak;
    if (haveBase && B.peak > ev.pgaBase) ev.pgaBase = B.peak;
    if (P.rms >= ev.rmsMax) {
      ev.rmsMax = P.rms;
      ev.pass = pass;
      ev.eval = eval;
      ev.crest = P.crest;
      ev.periodicity = P.periodicity;
      ev.decay = P.decay;
      ev.baseRatio = baseRatio;
      ev.freqAtMax = P.freq;
    }
    uint8_t l = levelFor(P.rms);
    if (l > ev.level) ev.level = l;
  };

  switch (s_state) {
    case ST_INIT:
    case ST_FAULT:
    case ST_IDLE:
      s_state = ST_IDLE;
      s_level = 0;
      if (P.rms >= on) {
        ev = EventAcc();
        ev.id = ++s_eventCount;
        ev.tStart = esp_timer_get_time();
        candDur = 0;
        s_state = ST_CANDIDATE;
      }
      break;

    case ST_CANDIDATE:
      candDur += dt;
      ev.duration = candDur;
      track();
      if (P.rms < off) {
        // ended without every check passing: rejected (bump or noise)
        ev.pass &= ~(1 << CK_DURATION);
        if (candDur >= g_params[P_CONFIRM_S]) ev.pass |= 1 << CK_DURATION;
        ev.eval |= 1 << CK_DURATION;
        emitEvent(0);
        s_state = ST_IDLE;
      } else if (pass == eval) {
        s_state = ST_ALARM;
        s_silenced = false;
        alarmT = 0;
        quiet = 0;
        ev.pass = pass;  // the confirming moment is the one to explain
        ev.eval = eval;
        logf("ALARM: %.2f Hz, RMS %.3f g, crest %.2f, periodicity %.2f", P.freq, P.rms, P.crest,
             P.periodicity);
      }
      break;

    case ST_ALARM:
      alarmT += dt;
      candDur += dt;
      ev.duration = candDur;
      if (P.rms > ev.rmsMax) {
        ev.rmsMax = P.rms;
        ev.freqAtMax = P.freq;
      }
      if (P.peak > ev.peak) ev.peak = P.peak;
      if (haveBase && B.peak > ev.pgaBase) ev.pgaBase = B.peak;
      if (P.freq > 0 && P.freqQ >= 0.5f) {
        ev.freqSum += P.freq;
        ev.freqN++;
      }
      {
        uint8_t l = levelFor(P.rms);
        if (l > ev.level) ev.level = l;
        s_level = ev.level;
      }
      quiet = (P.rms < off) ? quiet + dt : 0;
      if (alarmT >= g_params[P_LATCH_S] && quiet >= g_params[P_QUIET_S]) {
        ev.duration = candDur - quiet;
        emitEvent(1);
        s_state = ST_IDLE;
        s_level = 0;
        s_silenced = false;
      }
      break;
  }
  durOut = (s_state == ST_CANDIDATE || s_state == ST_ALARM) ? candDur : 0;
}

void detectorTask(void*) {
  TickType_t last = xTaskGetTickCount();
  uint32_t tick = 0;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    uint8_t pass, eval;
    float dur;
    Features P, B;
    bool haveBase;
    detectorStep(0.01f, pass, eval, dur, P, B, haveBase);
    tick++;

    if (tick % 10 == 0) {  // STATUS at 10 Hz
      PayloadWriter w;
      uint8_t flags = (s_silenced ? 0x01 : 0) | (sensorOk(0) ? 0x02 : 0) | (sensorOk(1) ? 0x04 : 0) |
                      (g_rawMode ? 0x08 : 0);
      w.u8(s_state);
      w.u8(s_state == ST_ALARM ? s_level : levelFor(P.rms));
      w.u8(flags);
      w.u8(pass);
      w.u8(eval);
      w.f32(P.rms);
      w.f32(P.peak);
      w.f32(P.crest);
      w.f32(P.freq);
      w.f32(P.freqQ);
      w.f32(P.periodicity);
      w.f32(P.decay);
      w.f32(haveBase ? B.rms : 0);
      w.f32(haveBase ? B.peak : 0);
      w.f32(dur);
      sendFrame(F_STATUS, w);
    }
    if (tick % 100 == 0) {  // HEALTH at 1 Hz
      PayloadWriter w;
      w.u32(millis());
      w.u32(txDropped());
      w.u32(ESP.getFreeHeap());
      uint32_t mv = analogReadMilliVolts(PIN_VBAT) * 2;
      w.u16(mv < 500 ? 0 : (uint16_t)mv);
      sensorsWriteHealth(w);
      sendFrame(F_HEALTH, w);
    }
    if (tick % 500 == 0) sendHello();
  }
}
