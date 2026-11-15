#pragma once
#include <Arduino.h>

void alertsInit();
void alertsTest(uint32_t ms);  // run the alarm pattern without a quake
void alertsTestMelody(uint8_t which);  // 1 = bump chime, 2 = end-of-shake jingle (bench diagnostics)
// True while the buzzer sounds (alarm, test or melody) and for ~1 s after: the sensor picks up the
// buzzer's own vibration, so the detector must not start a new candidate from it. Without this a
// bump chime shook the sensor, was judged a bump, and chimed again, over and over.
bool alertsSelfNoise();
void alertTask(void*);         // LEDs, buzzer, heartbeat, BOOT button
