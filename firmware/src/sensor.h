#pragma once
#include <Arduino.h>
#include <esp_timer.h>
#include "adxl345.h"
#include "dsp.h"
#include "protocol.h"

// Maps each sensor's sample count to ESP32 crystal time and measures the sensor's true output rate.
struct RateEstimator {
  static constexpr int N = 64;  // one point per second, 64 s baseline
  uint32_t n[N];
  int64_t t[N];
  int count = 0, head = 0;
  int64_t lastAdd = 0;
  float rate = ODR_NOMINAL;
  int64_t anchorT = 0;
  uint32_t anchorN = 0;
  bool hasAnchor = false;

  void clearHistory() { count = 0; head = 0; lastAdd = 0; }
  void add(int64_t tUs, uint32_t idx) {
    anchorT = tUs;
    anchorN = idx;
    hasAnchor = true;
    if (count > 0 && tUs - lastAdd < 1000000) return;
    lastAdd = tUs;
    n[head] = idx;
    t[head] = tUs;
    head = (head + 1) % N;
    if (count < N) count++;
    if (count >= 3) {
      int oldest = (head - count + N) % N, newest = (head - 1 + N) % N;
      double dt = (t[newest] - t[oldest]) * 1e-6;
      double dn = (double)(n[newest] - n[oldest]);
      if (dt > 1.5) {
        double r = dn / dt;
        if (r > 0.8 * ODR_NOMINAL && r < 1.2 * ODR_NOMINAL) rate = (float)r;
      }
    }
  }
  int64_t timeOf(uint32_t idx) const {
    if (!hasAnchor) return esp_timer_get_time();
    return anchorT + (int64_t)((double)(int32_t)(idx - anchorN) * 1e6 / rate);
  }
};

struct SensorCtx {
  int id;
  int sda, scl, intPin;
  TwoWire* wire;
  ADXL345* dev;
  TaskHandle_t task = nullptr;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  volatile int64_t isrTime = 0;

  // health
  bool present = false;
  uint8_t selfTest = ST_UNKNOWN;
  int16_t stDelta[3] = {0, 0, 0};
  uint32_t rawCount = 0, decCount = 0, overruns = 0, reinits = 0, consecErr = 0;
  float noiseMg = 0, tiltDeg = 0;
  RateEstimator rate;

  // signal chain
  Median3 med[3];
  FirDecimator fir;
  Biquad hp[3], lp[3];
  float R[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  bool zeroed = false, needPrime = true;
  volatile bool zeroRequest = true;
  bool zeroActive = false;
  int zeroN = 0, zeroAttempts = 0;
  double zSum[3], zMagSum, zMagSum2;
  FeatureTracker ft;
  int warm = 0;

  // published features
  Features feat;

  // frame building
  int16_t dataBuf[DATA_PER_FRAME][6];
  int dataN = 0;
  uint32_t dataFirstIdx = 0, dataFirstT = 0;
  bool gapPending = false;
  int16_t rawBuf[RAW_PER_FRAME][3];
  int rawN = 0;
  uint32_t rawFirstIdx = 0;

  float fsDec() const { return rate.rate / DECIM; }
};

extern SensorCtx g_sensors[NUM_SENSORS];
extern volatile bool g_rawMode;

void sensorsInit();                        // hardware + tasks
bool sensorOk(int i);                      // present and features valid
Features sensorFeatures(int i);
void sensorsRequestZero(uint8_t mask);
void sensorsWriteHealth(PayloadWriter& w);
