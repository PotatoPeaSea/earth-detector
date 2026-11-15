#include "sensor.h"
#include "detector.h"
#include "params.h"

static ADXL345 dev0(Wire, ADXL_ADDR);
static ADXL345 dev1(Wire1, ADXL_ADDR);
SensorCtx g_sensors[NUM_SENSORS];
volatile bool g_rawMode = false;

static void HAL_ISR_ATTR onInt(void* arg) {
  SensorCtx* s = (SensorCtx*)arg;
  {
    CritIsrGuard g(s->mux);
    s->isrTime = halNowUs();
  }
  BaseType_t hp = pdFALSE;
  if (s->task) vTaskNotifyGiveFromISR(s->task, &hp);
  HAL_YIELD_FROM_ISR(hp);
}

static inline int16_t toCounts(float g) {
  float v = g * OUT_SCALE;
  if (v > 32767.f) v = 32767.f;
  if (v < -32768.f) v = -32768.f;
  return (int16_t)lroundf(v);
}

// ---------------------------------------------------------------- frames
static void flushData(SensorCtx& s) {
  if (s.dataN == 0) return;
  PayloadWriter w;
  uint8_t flags = (s.gapPending ? 0x01 : 0) | (s.zeroed ? 0x02 : 0);
  s.gapPending = false;
  w.u8((uint8_t)s.id);
  w.u8(flags);
  w.u8((uint8_t)s.dataN);
  w.u32(s.dataFirstIdx);
  w.u32(s.dataFirstT);
  w.f32(s.fsDec());
  for (int i = 0; i < s.dataN; i++)
    for (int k = 0; k < 6; k++) w.i16(s.dataBuf[i][k]);
  sendFrame(F_DATA, w);
  s.dataN = 0;
}

static void flushRaw(SensorCtx& s) {
  if (s.rawN == 0) return;
  PayloadWriter w;
  w.u8((uint8_t)s.id);
  w.u8((uint8_t)s.rawN);
  w.u32(s.rawFirstIdx);
  for (int i = 0; i < s.rawN; i++)
    for (int k = 0; k < 3; k++) w.i16(s.rawBuf[i][k]);
  sendFrame(F_RAW, w);
  s.rawN = 0;
}

// ---------------------------------------------------------------- gravity alignment
static void computeRotation(SensorCtx& s, const double m[3]) {
  double norm = sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
  if (norm < 0.5) return;  // not plausible gravity
  double u[3] = {m[0] / norm, m[1] / norm, m[2] / norm};
  // rotate u onto +Z: v = u x z, c = u . z
  double v[3] = {u[1], -u[0], 0.0};
  double c = u[2];
  double s2 = v[0] * v[0] + v[1] * v[1];
  float R[3][3];
  if (s2 < 1e-12) {
    float sign = c > 0 ? 1.f : -1.f;
    float I[3][3] = {{1, 0, 0}, {0, sign, 0}, {0, 0, sign}};
    memcpy(R, I, sizeof(R));
  } else {
    double k = (1.0 - c) / s2;
    double vx[3][3] = {{0, -v[2], v[1]}, {v[2], 0, -v[0]}, {-v[1], v[0], 0}};
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        double vx2 = 0;
        for (int q = 0; q < 3; q++) vx2 += vx[i][q] * vx[q][j];
        R[i][j] = (float)((i == j ? 1.0 : 0.0) + vx[i][j] + k * vx2);
      }
  }
  memcpy(s.R, R, sizeof(R));
  s.tiltDeg = (float)(acos(fmax(-1.0, fmin(1.0, c))) * 180.0 / M_PI);
}

static void finishZero(SensorCtx& s) {
  double N = s.zeroN;
  double m[3] = {s.zSum[0] / N, s.zSum[1] / N, s.zSum[2] / N};
  double magMean = s.zMagSum / N;
  double magStd = sqrt(fmax(0.0, s.zMagSum2 / N - magMean * magMean));
  s.zeroActive = false;
  if (magStd > 0.03 && s.zeroAttempts < 3) {
    s.zeroAttempts++;
    s.zeroRequest = true;  // moving: try again
    logf("S%d zero retry: moving (std %.1f mg)", s.id + 1, magStd * 1000);
    return;
  }
  computeRotation(s, m);
  s.zeroed = true;
  s.zeroAttempts = 0;
  s.needPrime = true;
  logf("S%d zeroed: |g| %.4f g, tilt %.1f deg, std %.1f mg", s.id + 1, magMean, s.tiltDeg, magStd * 1000);
}

// ---------------------------------------------------------------- per-sample processing
static void processDecimated(SensorCtx& s, const float a[3], uint32_t rawIdx) {
  // gravity alignment window (on calibrated, unrotated data)
  if (s.zeroRequest && !s.zeroActive) {
    uint8_t st = detectorState();
    if (st != ST_CANDIDATE && st != ST_ALARM) {
      s.zeroRequest = false;
      s.zeroActive = true;
      s.zeroN = 0;
      s.zSum[0] = s.zSum[1] = s.zSum[2] = s.zMagSum = s.zMagSum2 = 0;
    }
  }
  if (s.zeroActive) {
    double mag = sqrt((double)a[0] * a[0] + (double)a[1] * a[1] + (double)a[2] * a[2]);
    for (int k = 0; k < 3; k++) s.zSum[k] += a[k];
    s.zMagSum += mag;
    s.zMagSum2 += mag * mag;
    if (++s.zeroN >= ZERO_SAMPLES) finishZero(s);
  }

  float r[3];
  for (int i = 0; i < 3; i++) r[i] = s.R[i][0] * a[0] + s.R[i][1] * a[1] + s.R[i][2] * a[2];

  if (s.needPrime) {
    for (int k = 0; k < 3; k++) {
      s.hp[k].prime(r[k]);
      s.lp[k].reset();
    }
    s.ft.reset();
    s.warm = 0;
    s.needPrime = false;
  }
  float f[3];
  for (int k = 0; k < 3; k++) f[k] = s.lp[k].step(s.hp[k].step(r[k]));

  float fs = s.fsDec();
  s.ft.push(f[0], f[1], f[2], fs);
  if (s.warm < WARMUP_SAMPLES) s.warm++;

  Features ft = s.ft.features();
  ft.valid = s.warm >= WARMUP_SAMPLES;
  if (ft.valid && detectorState() == ST_IDLE) {
    float mg = ft.rms * 1000.f;
    s.noiseMg = (s.noiseMg == 0) ? mg : s.noiseMg + 0.005f * (mg - s.noiseMg);
  }
  {
    CritGuard g(s.mux);
    s.feat = ft;
  }

  if (s.dataN == 0) {
    s.dataFirstIdx = s.decCount;
    s.dataFirstT = (uint32_t)s.rate.timeOf(rawIdx - FIR_DELAY);
  }
  int16_t* d = s.dataBuf[s.dataN];
  for (int k = 0; k < 3; k++) {
    d[k] = toCounts(r[k]);
    d[3 + k] = toCounts(f[k]);
  }
  if (++s.dataN == DATA_PER_FRAME) flushData(s);
  s.decCount++;
}

static void processRaw(SensorCtx& s, const RawSample& raw) {
  const Cal& c = g_cal[s.id];
  float in[3] = {(float)raw.x, (float)raw.y, (float)raw.z};
  float a[3];
  for (int k = 0; k < 3; k++) a[k] = s.med[k].step((in[k] * LSB_G - c.off[k]) * c.gain[k]);

  if (g_rawMode) {
    if (s.rawN == 0) s.rawFirstIdx = s.rawCount;
    s.rawBuf[s.rawN][0] = raw.x;
    s.rawBuf[s.rawN][1] = raw.y;
    s.rawBuf[s.rawN][2] = raw.z;
    if (++s.rawN == RAW_PER_FRAME) flushRaw(s);
  } else {
    s.rawN = 0;
  }

  float out[3];
  if (s.fir.push(a, out)) processDecimated(s, out, s.rawCount);
}

// ---------------------------------------------------------------- hardware
static bool hwStart(SensorCtx& s, bool selfTest) {
  s.wire->end();
  i2cBusRecover(s.sda, s.scl);
#if HAL_RP2040
  s.wire->setSDA(s.sda);
  s.wire->setSCL(s.scl);
  s.wire->setClock(I2C_HZ);
  s.wire->begin();
  s.wire->setTimeout(10);
#else
  s.wire->begin(s.sda, s.scl, I2C_HZ);
  s.wire->setTimeOut(10);
#endif
  if (!s.dev->probe()) {
    s.present = false;
    return false;
  }
  if (selfTest) s.selfTest = s.dev->selfTest(s.stDelta);
  bool ok = s.dev->startStream();
  s.present = ok;
  s.consecErr = 0;
  s.rate.clearHistory();
  s.gapPending = true;
  return ok;
}

static void sensorTask(void* arg) {
  SensorCtx& s = *(SensorCtx*)arg;
  uint32_t lastRetry = 0;
#if HAL_RP2040
  // RP2040 GPIO interrupts are handled by the core that attached them. Attaching from here puts the
  // sensor's interrupt on the same core as its task, away from the display and USB on core 0.
  halAttachRisingIsr(s.intPin, onInt, &s);
#endif
  for (;;) {
    if (!s.present) {
      vTaskDelay(pdMS_TO_TICKS(100));
      if (millis() - lastRetry > 1000) {
        lastRetry = millis();
        if (hwStart(s, false)) {
          s.reinits++;
          s.needPrime = true;
          logf("S%d back online", s.id + 1);
        }
      }
      continue;
    }

    uint32_t got = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(30));
    int64_t tIsr;
    {
      CritGuard g(s.mux);
      tIsr = s.isrTime;
    }

    bool ovr = false;
    int n = s.dev->fifoEntries(&ovr);
    if (n < 0) {
      if (++s.consecErr >= 3) {
        logf("S%d I2C failing, re-initializing", s.id + 1);
        s.reinits++;
        if (!hwStart(s, false)) {
          logf("S%d offline", s.id + 1);
        } else {
          s.needPrime = true;
        }
      }
      continue;
    }
    // One clean edge means the FIFO held exactly FIFO_WATERMARK unread entries at tIsr.
    if (got == 1 && n >= FIFO_WATERMARK && n < 32) s.rate.add(tIsr, s.rawCount + FIFO_WATERMARK - 1);
    // The first drain after (re)start finds a full FIFO by design; only count overruns after that.
    if ((ovr || n >= 32) && s.rawCount > 0) {
      s.overruns++;
      s.gapPending = true;
      s.rate.clearHistory();
    }
    for (int i = 0; i < n; i++) {
      RawSample r;
      if (!s.dev->readSample(r)) {
        s.consecErr++;
        break;
      }
      s.consecErr = 0;
      processRaw(s, r);
      s.rawCount++;
    }
  }
}

void sensorsInit() {
  const int sda[2] = {S1_SDA, S2_SDA}, scl[2] = {S1_SCL, S2_SCL}, ip[2] = {S1_INT, S2_INT};
  TwoWire* wires[2] = {&Wire, &Wire1};
  ADXL345* devs[2] = {&dev0, &dev1};
  for (int i = 0; i < NUM_SENSORS; i++) {
    SensorCtx& s = g_sensors[i];
    s.id = i;
    s.sda = sda[i];
    s.scl = scl[i];
    s.intPin = ip[i];
    s.wire = wires[i];
    s.dev = devs[i];
    s.fir.design(50.0f, ODR_NOMINAL);
    for (int k = 0; k < 3; k++) {
      s.hp[k].designHighpass(HP_HZ, FS_DEC_NOMINAL);
      s.lp[k].designLowpass(LP_HZ, FS_DEC_NOMINAL);
    }
    s.ft.reset();
    if (!SENSOR_FITTED[i]) continue;  // not installed: never started, no task, no interrupt
    pinMode(s.intPin, INPUT_PULLDOWN);
    bool ok = hwStart(s, true);
    logf("S%d: %s, DEVID first 0x%02X last 0x%02X, self-test %s (%d, %d, %d LSB)", i + 1,
         ok ? "streaming" : "NOT FOUND", s.dev->firstDevId, s.dev->devId,
         s.selfTest == ST_PASS ? "pass" : (s.selfTest == ST_FAIL ? "FAIL" : "n/a"), s.stDelta[0],
         s.stDelta[1], s.stDelta[2]);
  }
  // Restart both streams together, right before the tasks start, so neither FIFO overflows
  // while the other sensor runs its self-test.
  for (int i = 0; i < NUM_SENSORS; i++) {
    SensorCtx& s = g_sensors[i];
    if (s.present) s.present = s.dev->startStream();
    s.overruns = 0;
    s.gapPending = false;
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    SensorCtx& s = g_sensors[i];
    if (!SENSOR_FITTED[i]) continue;
    char name[12];
    snprintf(name, sizeof(name), "sensor%d", i + 1);
    halTaskCreate(sensorTask, name, 6144, &s, PRIO_SENSOR, CORE_SENSOR[i], &s.task);
#if !HAL_RP2040
    halAttachRisingIsr(s.intPin, onInt, &s);
#endif  // RP2040: the task attaches its own interrupt, see sensorTask
  }
}

bool sensorOk(int i) {
  SensorCtx& s = g_sensors[i];
  if (!s.present) return false;
  CritGuard g(s.mux);
  return s.feat.valid;
}

Features sensorFeatures(int i) {
  SensorCtx& s = g_sensors[i];
  Features f;
  {
    CritGuard g(s.mux);
    f = s.feat;
  }
  if (!s.present) f.valid = false;
  return f;
}

void sensorsRequestZero(uint8_t mask) {
  for (int i = 0; i < NUM_SENSORS; i++)
    if (mask & (1 << i)) {
      g_sensors[i].zeroAttempts = 0;
      g_sensors[i].zeroRequest = true;
    }
}

void sensorsWriteHealth(PayloadWriter& w) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    SensorCtx& s = g_sensors[i];
    w.u8(s.present ? 1 : 0);
    w.u8(s.dev ? s.dev->firstDevId : 0);
    w.u8(s.dev ? s.dev->devId : 0);
    w.u8(s.selfTest);
    for (int k = 0; k < 3; k++) w.i16(s.stDelta[k]);
    w.f32(s.rate.rate);
    w.u32(s.rawCount);
    w.u32(s.overruns);
    w.u32(s.dev ? s.dev->i2cErrors : 0);
    w.u32(s.reinits);
    w.f32(s.noiseMg);
    w.f32(s.tiltDeg);
    w.u8(s.zeroed ? 1 : 0);
  }
}
