#pragma once
#include <Arduino.h>

enum DetState : uint8_t { ST_INIT = 0, ST_IDLE = 1, ST_CANDIDATE = 2, ST_ALARM = 3, ST_FAULT = 4 };

// Check bits (pass/eval masks in STATUS and EVENT frames)
enum CheckBit : uint8_t {
  CK_DURATION = 0,
  CK_STEADY = 1,
  CK_FREQ = 2,
  CK_CREST = 3,
  CK_PERIODIC = 4,
  CK_BASE = 5,
};

struct DetectorView {
  uint8_t state;
  uint8_t level;
  bool silenced;
};

// Live numbers for on-device UIs (the round display); refreshed every detector tick (10 ms).
struct DetectorInfo {
  uint8_t state;     // DetState
  uint8_t level;     // magnitude level 0-5 (the alarm's level while in ALARM, else from current RMS)
  float rms, peak;   // g, primary sensor, 0.5 s window
  float freq;        // Hz, 0 = none
  bool ok0, ok1;     // sensors delivering valid data
  uint32_t bumpCount;  // increments each time a disturbance ends rejected (a bump, not a quake)
  float bumpPeak;      // g, peak of the latest bump
};
DetectorInfo detectorInfo();

uint8_t detectorState();
DetectorView detectorView();
void detectorSilence();
void detectorTask(void*);
