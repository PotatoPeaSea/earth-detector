#include "hal.h"

#if HAL_RP2040
#include <Adafruit_GC9A01A.h>
#include <SPI.h>
#include "config.h"
#include "detector.h"
#include "display.h"

// Magnitude names, same as the dashboard (index = level 0-5).
static const char* const kLevelName[6] = {"Quiet", "Micro", "Weak", "Moderate", "Strong", "Severe"};

static Adafruit_GC9A01A tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

namespace {
constexpr int W = 240;

uint16_t kGreen, kRed, kOrange, kPurple, kWhite, kBlack;
constexpr uint32_t BUMP_SHOW_MS = 3000;  // how long the bump screen stays up

// One text line: clear its band, then draw it centred. Skipped when nothing changed, so the
// screen does not flicker and the SPI bus stays mostly idle.
struct Line {
  char shown[24] = {0};
  uint16_t shownBg = 0, shownFg = 0;
};
Line g_line[6];

void drawLine(int slot, int y, int size, const char* text, uint16_t fg, uint16_t bg, bool force) {
  Line& l = g_line[slot];
  if (!force && l.shownBg == bg && l.shownFg == fg && strcmp(l.shown, text) == 0) return;
  strncpy(l.shown, text, sizeof(l.shown) - 1);
  l.shownBg = bg;
  l.shownFg = fg;
  int h = 8 * size;
  tft.fillRect(0, y, W, h, bg);
  int w = (int)strlen(text) * 6 * size;
  tft.setTextSize(size);
  tft.setTextColor(fg, bg);
  tft.setCursor((W - w) / 2, y);
  tft.print(text);
}
}  // namespace

void displayTask(void*) {
  SPI.setSCK(PIN_TFT_SCK);
  SPI.setTX(PIN_TFT_MOSI);
  tft.begin(40000000);
  tft.setRotation(2);  // 180 degrees from the default orientation
  kGreen = tft.color565(0, 150, 50);
  kRed = tft.color565(210, 0, 0);
  kOrange = tft.color565(230, 120, 0);
  kPurple = tft.color565(110, 0, 160);
  kWhite = tft.color565(255, 255, 255);
  kBlack = tft.color565(0, 0, 0);
  tft.fillScreen(kBlack);

  uint16_t bgNow = 0;
  bool force = true;
  uint8_t prevState = ST_INIT;
  uint8_t evLevel = 0;     // strongest level seen in the current alarm
  float evPeak = 0;        // largest peak (g) seen in the current alarm
  bool haveLast = false;
  uint8_t lastLevel = 0;
  float lastPeak = 0;
  uint32_t seenBumps = 0;  // bump counter value already shown
  uint32_t bumpUntil = 0;  // millis() until which the bump screen stays up
  float bumpPeak = 0;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(150));
    DetectorInfo d = detectorInfo();
    bool alarm = d.state == ST_ALARM;

    // Hold the strongest values while an alarm runs, and keep the last event afterwards.
    if (alarm) {
      if (prevState != ST_ALARM) {
        evLevel = 0;
        evPeak = 0;
      }
      if (d.level > evLevel) evLevel = d.level;
      if (d.peak > evPeak) evPeak = d.peak;
    } else if (prevState == ST_ALARM) {
      haveLast = true;
      lastLevel = evLevel;
      lastPeak = evPeak;
    }
    prevState = d.state;

    // A bump is a disturbance the detector rejected (not an earthquake): orange for a few seconds.
    uint32_t now = millis();
    if (d.bumpCount != seenBumps) {
      seenBumps = d.bumpCount;
      bumpUntil = now + BUMP_SHOW_MS;
      bumpPeak = d.bumpPeak;
    }
    bool bump = !alarm && (int32_t)(bumpUntil - now) > 0;

    // Green whenever the alarm is not going off. A sensor fault is purple: with no sensor the
    // device cannot detect anything, so it must not look healthy.
    uint16_t bg = alarm ? kRed : bump ? kOrange : (d.state == ST_FAULT ? kPurple : kGreen);
    if (bg != bgNow) {
      tft.fillScreen(bg);
      bgNow = bg;
      force = true;
    }

    if (bump) {
      char pb[24];
      snprintf(pb, sizeof(pb), "%.2f g", bumpPeak);
      drawLine(0, 26, 2, "NOT A QUAKE", kWhite, bg, force);
      drawLine(1, 50, 2, "IGNORED", kWhite, bg, force);
      drawLine(2, 72, 8, "BUMP", kWhite, bg, force);
      drawLine(3, 142, 3, pb, kWhite, bg, force);
      drawLine(4, 176, 2, "", kWhite, bg, force);
      drawLine(5, 200, 2, "", kWhite, bg, force);
      force = false;
      continue;
    }

    const char* status = alarm ? "ALARM" : d.state == ST_FAULT ? "NO SENSOR" : d.state == ST_INIT ? "STARTING" : d.state == ST_CANDIDATE ? "CHECKING" : "QUIET";
    uint8_t lvl = alarm ? evLevel : d.level;
    float peak = alarm ? evPeak : d.peak;
    char buf[24];

    drawLine(0, 26, 2, status, kWhite, bg, force);
    drawLine(1, 50, 2, "MAGNITUDE", kWhite, bg, force);
    snprintf(buf, sizeof(buf), "%u", (unsigned)lvl);
    drawLine(2, 72, 8, buf, kWhite, bg, force);
    drawLine(3, 142, 3, kLevelName[lvl > 5 ? 5 : lvl], kWhite, bg, force);
    if (d.freq > 0.5f && (alarm || d.state == ST_CANDIDATE))
      snprintf(buf, sizeof(buf), "%.2f g %.1f Hz", peak, d.freq);
    else
      snprintf(buf, sizeof(buf), "%.2f g", peak);
    drawLine(4, 176, 2, buf, kWhite, bg, force);
    if (alarm) {
      snprintf(buf, sizeof(buf), "EARTHQUAKE");
    } else if (haveLast) {
      snprintf(buf, sizeof(buf), "last L%u %.1fg", (unsigned)lastLevel, lastPeak);
    } else {
      snprintf(buf, sizeof(buf), "no events");
    }
    drawLine(5, 200, 2, buf, kWhite, bg, force);
    force = false;
  }
}

#else
#include "display.h"
void displayTask(void*) {}
#endif
