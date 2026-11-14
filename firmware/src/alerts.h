#pragma once
#include <Arduino.h>

void alertsInit();
void alertsTest(uint32_t ms);  // run the alarm pattern without a quake
void alertTask(void*);         // LEDs, buzzer, heartbeat, BOOT button
