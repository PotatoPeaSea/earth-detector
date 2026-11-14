# Unshakeable Earth Detector: Wiring and Firmware Plan

Draft for review, Nov 14 2026. Builds on the electronics handoff and the judging criteria. Nothing here is built yet except what the board already runs.

## 0. What the bench shows today

| Check | Result |
| --- | --- |
| Board | ESP32-D0WD-V3 rev 3.1 (classic dual-core ESP32), 40 MHz crystal, 4 MB flash, CH340 USB on COM10 |
| Flash voltage | Set by strapping pin GPIO 12 (eFuse not burned), so GPIO 12 must be low at every reset |
| Current firmware | Polls one ADXL345 at 0x53 and prints x/y/z text |
| First DEVID read | Returned 0xE6, the retry returned 0xE5. Either a glitch on the first transaction or a clone part; bring-up repeats it 1000 times to find out |
| Gravity at rest | x 0.048, y -0.993, z 0.240 g, so the sensor sits on its edge (Y vertical) and is tilted about 14 degrees. Fine, firmware will rotate it (section 4.3) |
| Toolchain | PlatformIO with espressif32 6.12 / Arduino-ESP32 2.0.17 installed, Python and Node present |

## 1. Rule risks to clear first

- **Microcontroller.** The rubric says "Arduino or Raspberry Pi Pico" and the kit hands out a Pico (RP2040). Ask the organizers whether an ESP32 is allowed. The firmware below keeps sensor, DSP and protocol code behind a thin hardware layer so it can move to the Pico (two I2C controllers, Arduino-Pico core) if the answer is no.
- **Floors.** The intro slides say "max 2 floors", the rubric says "2 to 3". Tell the structure lead to build 2.
- **Tests.** The slides list a design analysis, a bump test and a shake test. The detector, the dashboard and the explanation of why a bump was rejected should all be demo-ready for the bump test, not only the shake.

## 2. System architecture options (pick one)

All three share the same firmware, the same binary frames and the same dashboard code. They differ in how the page gets the data.

| | A. Laptop backend (handoff plan) | B. Browser-direct (recommended) | C. ESP32 hosts the page |
| --- | --- | --- | --- |
| Link | USB serial or BLE into Python (FastAPI, pyserial, bleak), WebSocket to the page | Chrome talks to the ESP32 directly: Web Serial over USB, Web Bluetooth over BLE | ESP32 runs a Wi-Fi access point and serves the page plus a WebSocket |
| Install at demo | Python env and packages | None, one HTML file (served from localhost or GitHub Pages) | None, connect to the ESP32 Wi-Fi |
| Tether to building | Optional (BLE) | Optional (BLE) | None |
| Strengths | Python tooling for logging and offline analysis | Fewest moving parts on demo day; DSP in JS is easily fast enough for 2 x 200 Hz | Judges can open the live view on their own phone by scanning a QR code |
| Weaknesses | Most moving parts; bleak behaves differently per OS | Chrome or Edge only | 2.4 GHz crowding at the venue, more firmware, higher current draw; Wi-Fi and BLE share one radio |

**Recommendation: B**, with BLE as the wireless link and USB as the fallback, and a small Python script only for offline analysis of recorded CSVs. A wireless link matters for scoring: a USB cable pulling on a 40 cm building changes its sway and costs cable-management points. C is a good stretch goal for the "wow" factor if time remains.

## 3. Wiring

### 3.1 Pin map

| Function | GPIO | Notes |
| --- | --- | --- |
| S1 SDA / SCL (Wire, bus 0) | 21 / 22 | 400 kHz, as wired now |
| S1 INT1 | 13 | Keep |
| S2 SDA / SCL (Wire1, bus 1) | 14 / 27 | Keep. GPIO 14 toggles briefly at boot, so firmware runs an I2C bus recovery before init |
| **S2 INT1** | **26** (moved off 12) | See 3.2. No strapping role; the ADXL345 INT pin is push-pull, so no pull-up is needed |
| Green / yellow / red LED | 25 / 32 / 33 | LEDC PWM so they can blink, breathe or chase |
| Buzzer (via NPN) | 23 | LEDC PWM for siren tones if the buzzer is passive |
| Heartbeat | 2 (on-board LED) | Blinks only while samples arrive |
| Silence / re-zero button | 0 (on-board BOOT) | Free to use after boot |
| Battery voltage (optional) | 35 | ADC1, 100 k / 100 k divider from the pack's 5 V |
| Optional OLED (SSD1306, 0x3C) | Bus 0 | See 3.5 |
| Avoid | 6-11 (flash), 0/2/5/12/15 as outputs or inputs that can be high at reset | |

### 3.2 Must fix: S2 interrupt on GPIO 12

GPIO 12 is a strapping pin. If it is high when the ESP32 resets, the chip sets its flash supply to 1.8 V and does not boot (the bench readout confirms this board takes its flash voltage from that pin).

At power-on the ADXL345 keeps its interrupts off, so the first boot works. But once firmware has configured S2 for FIFO streaming, the sensor stays powered and streaming through any ESP32 reset (upload, watchdog, brownout, pressing EN). The watermark interrupt goes high within 20 ms, holds GPIO 12 high, and the next boot fails until someone unplugs the battery. At the competition that means a dead alarm after any glitch, and the rubric re-tests the alarm on a second shake.

Done: the S2 INT1 wire now goes to GPIO 26. (Burning the flash-voltage eFuse would also have worked, but it is permanent and was not needed.)

### 3.3 Sensors

- Recommended placement: **S1 on the top floor next to the TMD** (the rubric asks for this) and **S2 on the base plate**. The base sensor measures what the table actually does, which gives the bump-rejection check, the amplification ratio and the magnitude.
- Each module: VCC to 3.3 V, GND, SDA, SCL, **CS tied to 3.3 V** (I2C mode), **SDO to GND** (address 0x53; both sensors can use 0x53 because they are on separate buses), INT1 to its GPIO.
- GY-291-style modules usually have 4.7 k pull-ups on board. Check with a meter (SDA to 3.3 V); add 4.7 k only if missing. The ESP32 internal pull-ups are too weak for 400 kHz.
- 10 uF plus 100 nF across VCC and GND at each module.
- The top sensor's harness is about 40 cm long. Use a 6-wire ribbon or two twisted pairs in this order: SCL, GND, SDA, 3V3, INT1, GND, so each clock and data line sits next to a ground. Firmware counts I2C errors; if the count is not zero over a 10 minute test, drop to 200 kHz before anything else.
- Mount both sensors with two screws through the module holes plus a thin layer of epoxy, never foam tape (foam acts as a spring and filters the signal). Align one sensor axis with the table's shake direction where possible; firmware handles any remaining tilt.

### 3.4 Alerts

```
3V3 GPIO 25 --[220 R]--|>|-- GND   green  (also yellow on 26, red on 32)

5V ----+-----------+
       |           |
    [buzzer]    1N4148 (cathode to 5V, only for a magnetic/passive buzzer)
       |           |
       +-----------+
       |
       C
GPIO 33 --[1 k]-- B   NPN (kit part, e.g. S8050 or 2N2222)
       E
       |
      GND
```

- The kit's 470 ohm resistors give only about 3 mA from 3.3 V, which is dim under hall lighting. Use 150 to 220 ohm (6 to 9 mA), or use the 470 ohm parts on bright 10 mm LEDs. Put the LEDs on the outside of the top floor where judges look.
- Drive the buzzer from 5 V through the NPN for volume. If it is passive, LEDC can play a rising siren that changes with severity (an easy "dynamic alert" point). If it is active, switch it on and off in patterns.

### 3.5 Optional on-building display

A 0.96 inch SSD1306 is optional (the laptop display counts). If added, put it on bus 0 and send one 128-byte page per transaction under the bus mutex: a full-frame write holds the bus for about 25 ms, which would delay draining S1's FIFO. The FIFO holds 32 samples (40 ms at 800 Hz), so page-by-page writes keep plenty of margin.

### 3.6 Power and harness

- Building runs from a USB power bank into the ESP32 USB port (or 5 V/VIN). Do not connect the battery to VIN while the laptop USB is also plugged in unless the board has a diode between them; check the board before trying it.
- 470 uF electrolytic across 5 V and GND near the ESP32 for BLE current bursts.
- One ground star point at the ESP32. The 12 V motor supply stays completely separate.
- ESP32, battery and perfboard mount low in the building; flexible harness up to S1 with strain relief at both ends; JST-XH or screw terminals instead of loose jumpers; zip ties to the frame, no free-hanging loops. If USB is used, anchor a slack loop to the table so it cannot pull on the building.

## 4. Firmware

### 4.1 Sensor configuration (each sensor)

| Register | Value | Why |
| --- | --- | --- |
| DEVID 0x00 | Expect 0xE5, read 3 times | Retry before failing; log the first value |
| Self-test | DATA_FORMAT bit 7, average 100 samples on and off | Confirms the sensing element moves; result shown on the dashboard health panel |
| BW_RATE 0x2C | 0x0D | 800 Hz, normal power (low-power mode adds noise) |
| DATA_FORMAT 0x31 | **0x0B** | Full resolution, +/-16 g. In full-resolution mode the scale is 3.9 mg/LSB at every range, so the widest range costs no resolution and the top floor can never clip |
| FIFO_CTL 0x38 | 0x90 | Stream mode, watermark 16 |
| INT_MAP 0x2F / INT_ENABLE 0x2E | 0x00 / 0x02 | Watermark on INT1 |
| POWER_CTL 0x2D | 0x08 | Measure |

Set everything with POWER_CTL in standby first, then switch to measure last.

### 4.2 Getting clean, evenly spaced samples

1. **Sensor clock sets the sample spacing.** The ESP32 never polls on its own timer. The INT1 interrupt only notifies the sensor task.
2. **Drain correctly.** Read FIFO_STATUS for the entry count, then read each entry as its own 6-byte burst from 0x32 (the FIFO pops one entry per read and needs at least 5 us between reads). About 3.6 ms for 16 entries at 400 kHz.
3. **Never stall.** The task waits on the notification with a 30 ms timeout and drains anyway on timeout, so a missed edge cannot freeze the stream. It also checks FIFO_STATUS again after draining.
4. **Detect gaps, never hide them.** Every sample gets a 32-bit sample index. If FIFO_STATUS shows 32 entries or the overrun bit (INT_SOURCE 0x30 bit 0) is set, a gap flag goes into the stream and the counter shows on the dashboard.
5. **Measure the true sample rate.** The ADXL345's internal oscillator is not precise, and its output rate can be off by a few percent from 800 Hz. That error goes straight into the measured shaking frequency. Each watermark interrupt is timestamped with the ESP32 crystal clock (esp_timer, tens of ppm), and a running linear fit of sample index against time gives each sensor's real rate (for example 793.6 Hz). All frequency math uses the measured rate.
6. **One timeline for both sensors.** The same fit maps each sensor's sample index to ESP32 time, so S2 is resampled onto S1's time grid before any comparison, ratio or phase calculation.
7. **I2C health.** Count every failed transaction. Retry once; after three failures in a row, run bus recovery (9 clock pulses and a STOP) and re-initialize that sensor while the other keeps running. The LEDs show a distinct fault pattern so a dead sensor is never mistaken for "no earthquake".

### 4.3 Calibration

- **Six-position calibration (once, stored in flash).** Hold each module still with each axis pointing up and then down. This gives per-axis offset and gain from gravity, so reported g values are accurate rather than raw counts.
- **Gravity alignment (at every boot and on the re-zero button).** Average 2 s at rest, find the gravity direction and build a rotation so the output is vertical plus two horizontal axes. This removes the 14 degree tilt seen on the bench and makes "horizontal shaking" a clean quantity.
- **Shake direction (learned from the first shake).** A principal-component fit of the horizontal motion finds the table's axis; the frequency and magnitude estimates use that projection, which has the best signal-to-noise ratio.

### 4.4 Signal chain (per sensor, on the ESP32)

1. Calibrate and rotate (above).
2. Median of 3 at 800 Hz to remove single-sample glitches.
3. Decimate 800 to 200 Hz with a 31-tap FIR low-pass. This is free noise reduction: averaging 4 samples cuts white noise by about 2x, and it removes anything that could alias.
4. Band-pass: 2nd-order high-pass at 0.5 Hz (removes gravity and slow tilt) and 2nd-order low-pass at 25 Hz, as two biquads.
5. Features over a sliding window: RMS (0.5 s), peak, crest factor (peak / RMS), dominant frequency (zero-crossing period averaged over the window, cross-checked with a small Goertzel bank), and periodicity (autocorrelation at one period).

### 4.5 Detection: explainable bump vs earthquake

The state machine goes IDLE, then CANDIDATE when RMS crosses the on-threshold, then ALARM only if every check passes. Each check produces a reason code that goes out in the event frame, so the dashboard can show a verdict card.

| Check | Earthquake | Bump |
| --- | --- | --- |
| Duration above threshold | at least 0.5 s | decays in about 0.2 s |
| Periodicity (autocorrelation at one period) | high, above 0.5 | low |
| Dominant frequency | inside the table's band (set after measuring the motor, roughly 1 to 15 Hz) | building ring-down, usually higher |
| Crest factor | about 1.4 to 2 (sine-like) | high, above 3 (spiky) |
| Base sensor (S2) | also shaking periodically | quiet when someone knocks the building or its cable; a table knock shows on both, so the first four checks handle that case |

Alarm latches for at least 3 s, uses separate on and off thresholds, and re-arms after 1.5 s of quiet, so it fires again on the second shake. The alarm runs entirely on the ESP32 and never waits on the radio or the dashboard.

### 4.6 Magnitude (5-point bonus)

Show three numbers, each explained on the dashboard:

1. **Table level**, from a lookup built by recording the base RMS at each table setting during practice. This is what the rubric checks, if the station has settings.
2. **Instrumental intensity (Modified Mercalli)** from peak ground acceleration on the base sensor, using the Wald et al. (1999) relation, MMI = 3.66 log10(PGA in cm/s^2) - 1.66. Real seismology vocabulary that judges recognize.
3. **Table stroke in mm**, displacement = acceleration / (2 pi f)^2 at the measured frequency. Judges can check this against the visible motion of the table, which shows the numbers are real.

### 4.7 Tasks (FreeRTOS on the ESP32)

| Task | Core | Priority | Job |
| --- | --- | --- | --- |
| Sensor S1, Sensor S2 | 1 | 20 | One per I2C bus, so the two buses drain in parallel |
| DSP and detector | 1 | 15 | Gets 16-sample blocks from a queue, runs 4.4 and 4.5 |
| Alert | 1 | 18 | LEDs and buzzer patterns from the detector state |
| Comms | 0 | 5 | Frames out over USB (921600 baud) and BLE (NimBLE), commands in |
| Housekeeping | 0 | 2 | Battery voltage, health frame, saving settings |

Settings (thresholds, durations, calibration) live in NVS and are exchanged as atomic snapshots. The task watchdog covers the sensor and DSP tasks.

### 4.8 Frames (same on USB and BLE)

COBS-encoded on USB (one zero byte ends each frame, which resynchronizes automatically after a dropped byte); one frame per notification on BLE. Every frame has type, sequence number and CRC16.

| Frame | Contents | Rate |
| --- | --- | --- |
| DATA | Sensor id, first sample index, count, int16 x/y/z. Stream mode is 200 Hz decimated; raw mode is full 800 Hz (USB only) for recordings | 10 to 50 Hz |
| STATUS | State, level, RMS, peak, dominant frequency, amplification ratio | 10 Hz |
| EVENT | Start/end sample index, verdict, each check's value and pass/fail, peak, frequency, magnitude numbers | Per event |
| HEALTH | Measured sample rate per sensor, overruns, I2C errors, dropped frames, self-test result, noise floor, battery voltage, uptime | 1 Hz |
| CMD (to ESP32) | Thresholds, durations, re-zero, start calibration, raw mode on/off, test alarm, silence | On demand |

Bandwidth: both sensors in raw mode are about 11 kB/s, well inside 921600 baud. BLE carries the 200 Hz stream, about 3 kB/s.

## 5. Dashboard

**Chosen design: `design/concept-e-sidebar.html`** (minimalist, one matplotlib-style graph with Time / Spectrum views, text sidebar, toggleable event log). Concepts A-D are kept for reference only. Every concept has: status banner matching the LEDs, raw and filtered graphs, measured frequency with FFT and confidence, peak and RMS g, magnitude, both sensors with amplification ratio, event log, a data-quality strip and simulator buttons.

Frequency on the laptop: 1024-point FFT (about 5 s at 200 Hz), Hann window, 4x zero padding and parabolic peak interpolation, using the measured sample rate. A clean table sine should read to about 0.01 Hz.

## 6. Extra ideas

- **Turn bumps into data.** Every rejected bump makes the building ring. Fit the decay to report the building's natural frequency and damping ratio: "Bump rejected. Building natural frequency 11.3 Hz, damping 4.2 %." This shows the detector understands the structure, and comparing damping with and without the TMD makes a strong design-analysis slide.
- **TMD effectiveness plot.** Amplification ratio (top / base) over time, and against frequency if the table speed can change.
- **Record everything.** Every practice run is saved as CSV with its verdict and can be replayed through the dashboard, which is how thresholds get tuned on the real building.
- **Judge replay.** One button replays the last event in slow motion with the verdict card.
- **Known-frequency check.** Tape a sensor to a speaker playing a 10 Hz tone (or any motor of known speed) to prove the frequency readout is accurate, and quote the error in the presentation.
- **Honest fault display.** If a sensor drops out, the LEDs and dashboard say so instead of looking calm.

## 7. Acceptance tests (data quality)

| Test | Pass |
| --- | --- |
| DEVID read 1000 times on each bus | No wrong values, no I2C errors |
| Rest noise, 60 s, after decimation | Recorded per axis in mg rms; S1 and S2 within 30 % of each other |
| Sample rate, 60 s | Rate estimate stable to 0.01 %; both sensors' rates logged |
| 10 minutes streaming with BLE and dashboard open | 0 overruns, 0 gaps, 0 I2C errors |
| Press EN 20 times while streaming | Boots every time (checks the GPIO 34 move) |
| Speaker or known-motor frequency test | Reads within 0.05 Hz |
| 20 bumps (table knocks, building flicks, cable tugs) and 10 shakes on the real building | 0 false alarms, 10 of 10 detected, alarm within 1 s |
| Second shake right after the first | Alarm fires again |
| Wiggle and tug every harness | No dropouts |

## 8. Status and tuning once the sensors are mounted

Firmware 0.1.0 is flashed. As of Nov 14 2026 **both IMUs are only wired, sitting loose in a passive position, not screwed to the building.** Everything below depends on the final mount and must be redone after mounting:

| Item | Why it changes with mounting | How to set it |
| --- | --- | --- |
| Gravity alignment (tilt) | Bench values (S1 74 deg, S2 95 deg) are for the loose position | Automatic at boot; hold BOOT 2 s after mounting to redo it |
| Six-position calibration | S2 reads 1.095 g at rest (9.5 % scale/offset error); S1 0.993 g | Do it before mounting (needs each axis up and down), store with `C_SET_CAL` |
| Noise floor | Loose sensors pick up desk vibration differently from a screwed mount | Read from the HEALTH frame after mounting |
| `on_thr` / `off_thr` (40 / 25 mg) | Depends on noise floor and the building's response at the top floor | Record table shakes and bumps on the real building, set on_thr well above bump RMS and below shake RMS |
| `confirm_s` (0.8 s) | Trade-off between alarm latency and bump rejection | From the same recordings |
| `fmin` / `fmax` (1-15 Hz) | Must bracket the table's actual motor frequency | Measure the motor frequency, then set the band around it |
| `crest_max`, `periodic_min`, `decay_min` (3.0 / 0.5 / 0.6) | Bump ring-down depends on the building's stiffness and the TMD | Compare event values for recorded bumps vs shakes |
| `base_ratio_min` (0.2) | Top/base amplification depends on the structure and TMD | Set below the smallest base/top ratio seen in real shakes |
| Severity levels `lvl1-5` | Should map to the table's magnitude settings | One recording per table setting |
| Self-test limits | Fine as is (both pass with large margin) | None |

Bench facts that do not depend on mounting: S1 runs at 829.07 Hz and S2 at 788.12 Hz (measured and corrected automatically); both modules are clones reporting DEVID 0xE6; 0 overruns, 0 I2C errors, 0 lost frames over 60 s.

## 9. Build order

1. Move S2 INT1 to GPIO 34. Bring-up firmware: both sensors, DEVID loop, self-test, FIFO streaming, measured rates, noise floor printed.
2. Binary frames, plus the chosen dashboard reading them over Web Serial.
3. Signal chain, detector, LEDs and buzzer, event verdicts.
4. Calibration (six-position and gravity alignment), magnitude, recording and replay.
5. BLE link and auto-reconnect; fall back to USB with one click.
6. Hardening: harness, acceptance tests on the finished building, threshold tuning from recordings.
