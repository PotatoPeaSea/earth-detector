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

uint8_t detectorState();
DetectorView detectorView();
void detectorSilence();
void detectorTask(void*);
