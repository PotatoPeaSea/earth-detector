#pragma once
// Round 1.28" GC9A01 status display (RP2040 build only): solid green while the alarm is not going
// off, solid red during an alarm, with the shake's magnitude (level 0-5 + name, peak g, frequency).
void displayTask(void*);
