# Incident log, 2026-11-14: alarm kept sounding (RP2040 build)

Two separate problems showed up as "the alarm doesn't stop". Both were seen on the YD-RP2040 build (branch `rp2040-port`), never on the ESP32 build.

## 1. Buzzer stuck on after a 2-second test alarm (fixed)

**Symptom.** After `monitor.py --test-alarm` the buzzer kept sounding and never stopped; the user had to unplug the board.

**Evidence.**
- Frame rate in the monitor was ~40 frames/s (40 DATA frames, no STATUS) instead of ~50/s. The detector task had stopped.
- It only appeared once S1 started working, i.e. with both ADXL345s streaming at 800 Hz.

**Cause.** arduino-pico's `Wire` busy-waits during every I2C transfer, so each sensor task keeps a core busy for a large share of the time. Both sensor tasks were on core 1 at a higher priority than the detector and alert tasks, so those two were starved. The alert task never ran again, so the last PWM setting (the test siren) stayed latched; the PWM hardware keeps running with no CPU involvement.

**Fix (in the code).**
- S1's sensor task on core 0, S2's on core 1 (`CORE_SENSOR` in `config.h`).
- Detector and alert tasks now have higher priority than the sensor tasks (`PRIO_*` in `config.h`).
- Hardware watchdog (2 s) fed by the alert task, so a wedged firmware resets the board and silences the buzzer instead of leaving it on.
- Crash recorder (`hal.cpp`): panic, assert, stack overflow and watchdog resets are logged on the next boot.

**Verified.** 25 s run: both sensors ~200 samples/s, STATUS back at 10 Hz (~52 frames/s), 0 lost frames; alarm test started and stopped on its own.

## 2. Alarm kept sounding ~11-25 s after a real shake stopped (firmware fixed; S1 hardware still open)

**Symptom.** After a hard shake the alarm kept sounding with nobody touching the table. User's first description: "the passive noise from the accelerometers is continuing the alarm".

**Recording** (100 s, `monitor.py`, shake from t=12 s to t=21 s):
- Shake: ~1.3 g RMS, 2.6 g peak. Alarm confirmed at t=13.7 s (EARTHQUAKE, level 5, all checks passed).
- Table at rest again by t=22 s: base sensor (S2) back to 10-16 mg, idle level is ~7 mg, end threshold 25 mg.
- Alarm kept sounding until about t=33 s, ~11 s after the shaking ended.

**Cause (from the recording, not yet fixed).**
- S1 (the top sensor, the one the detector uses for the end-of-alarm decision) became glitchy after the shake: RMS jumped between 16 and 64 mg with peaks of 76-116 mg and no steady frequency, so it crossed the 25 mg end threshold repeatedly and reset the 1.5 s quiet timer each time.
- S1 logged 22 "I2C failing, re-initializing" events in the run (98 I2C errors, 29 re-inits, 50 FIFO overruns). S2 had 4 errors and no re-inits. A re-init leaves S1's features invalid for about 1 s, and the detector holds the alarm in place while its primary sensor is invalid.
- Most likely physical: the S1 connection was shaken loose by the violent shake (S1 wiring had been intermittent all session).
- The buzzer itself couples into the sensors (RMS about doubled during a test alarm: base 7 to 16 mg, top 9 to 14 mg) but stayed below the end threshold, so it is a minor contributor.

**Second recording (after rewiring interrupts: S1 INT on GP6, S2 INT on GP13; interrupts verified working, measured rates 820 / 786 Hz).**
- Shake t=52-61 s (S1 up to 0.7 g RMS, base up to 1.4 g); alarm confirmed t=54 s as EARTHQUAKE.
- Table at rest by t=62 s (base 8-17 mg). Alarm kept sounding until about t=81-87 s, i.e. 20-25 s after the shake stopped.
- Same pattern: S1 RMS 14-60 mg with peaks to 210 mg and no steady frequency, readings of 0.0 mg while S1 was invalid mid re-init, S1 rate down to 160-190 samples/s. 31 "I2C failing" log lines, 108 I2C errors, 33 re-inits on S1; S2 had 1 error and 0 re-inits.
- S1 had 0 errors in quiet periods (12 s idle check, and the idle parts of both recordings), so it fails only under vibration: the S1 connection is mechanically weak. The interrupt-pin change did not fix or cause this.

**Changes made (`detector.cpp`, user approved) and result.**
1. "Shaking has stopped" is now judged by the base sensor (S2, the table itself) when it is valid; the top sensor is used only if S2 is missing.
2. The primary sensor is now the first *valid* one (S1, else S2) instead of the first *present* one, so an S1 re-init no longer freezes the alarm; S2 takes over until S1 is valid again.
3. Verified with a third shake recording on the same hardware: shake 35.7-43.8 s, table at rest by 44.8 s, alarm ended about 46-47 s, i.e. 2-3 s after the table stopped (was 20-25 s). The status line shows `base 0.0 mg` while S1 was invalid, which is the fallback to S2 at work. S1 still logged 9 re-inits / 31 I2C errors during the shake (S2: none), so the S1 connection is still weak under vibration.

**Still open.**
- Hardware: strain relief and solid connections for the S1 harness (solder or locking connector, twisted pairs).
- Optional: replace the 1.8-3.2 kHz sweep with a two-tone siren whose aliases at the 800 Hz sample rate fall outside the 0-25 Hz passband (the sweep passes through 2400 Hz, a multiple of 800 Hz).

**Requirement behind this** (user, same day): the alarm must sound continuously while the building is shaking, and stop once it isn't.

## 3. Silent USB stream after boot (fixed, for completeness)

The monitor and dashboard drop DTR after opening the port. arduino-pico USB CDC only transmits with DTR high (fixed with `ignoreFlowControl`) and, once its 256-byte FIFO fills with nobody reading, never restarts the transfer on its own. `usbWrite` in `protocol.cpp` now chunks writes and calls `Serial.flush()` on the skipped-frame path to kick it. Symptom was zero frames from a board that was otherwise running.
