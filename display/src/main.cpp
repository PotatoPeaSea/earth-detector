// Round status display for the Unshakeable Earth Detector (ESP32-2424S012: ESP32-C3 + GC9A01 240x240).
//
//   green  = no shake detected
//   red    = alarm; shows the magnitude exactly as the dashboard does (level 1-5 + its name) and peak g
//   BUMP   = a rejected event (the detector's "bump or noise" verdict), shown on green for a few seconds
//   grey   = no data from the computer (never shows green or red it cannot vouch for)
//
// Input: newline-terminated ASCII lines on the USB serial port, sent by tools/display_bridge.py:
//   S,<state>,<level>,<peak_g>,<rms_g>,<freq_hz>   detector STATUS (state 0 init, 1 idle, 2 candidate, 3 alarm, 4 fault)
//   B,<peak_g>,<duration_s>                        a bump (rejected event) just ended
#define LGFX_USE_V1
#include <Arduino.h>
#include <LovyanGFX.hpp>

// ---- board pins (ESP32-2424S012) ----
constexpr int PIN_SCLK = 6, PIN_MOSI = 7, PIN_CS = 10, PIN_DC = 2, PIN_BL = 3;
constexpr bool BL_ACTIVE_LOW = false;
constexpr bool PANEL_INVERT = true;   // GC9A01 panels are normally wired inverted; flip if colours look negative
constexpr bool PANEL_BGR = false;     // flip if red and blue are swapped

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 panel_;
  lgfx::Bus_SPI bus_;
  lgfx::Light_PWM light_;

 public:
  LGFX() {
    {
      auto c = bus_.config();
      c.spi_host = SPI2_HOST;
      c.spi_mode = 0;
      c.freq_write = 40000000;
      c.freq_read = 16000000;
      c.spi_3wire = true;
      c.use_lock = true;
      c.dma_channel = SPI_DMA_CH_AUTO;
      c.pin_sclk = PIN_SCLK;
      c.pin_mosi = PIN_MOSI;
      c.pin_miso = -1;
      c.pin_dc = PIN_DC;
      bus_.config(c);
      panel_.setBus(&bus_);
    }
    {
      auto c = panel_.config();
      c.pin_cs = PIN_CS;
      c.pin_rst = -1;  // no reset line on this board
      c.pin_busy = -1;
      c.panel_width = 240;
      c.panel_height = 240;
      c.offset_x = 0;
      c.offset_y = 0;
      c.readable = false;
      c.invert = PANEL_INVERT;
      c.rgb_order = PANEL_BGR;
      c.bus_shared = false;
      panel_.config(c);
    }
    {
      auto c = light_.config();
      c.pin_bl = PIN_BL;
      c.invert = BL_ACTIVE_LOW;
      c.freq = 44100;
      c.pwm_channel = 7;
      light_.config(c);
      panel_.setLight(&light_);
    }
    setPanel(&panel_);
  }
};

static LGFX lcd;

// ---- dashboard vocabulary (dashboard/index.html: const LV) ----
static const char* const LEVEL_NAME[6] = {"QUIET", "MICRO", "WEAK", "MODERATE", "STRONG", "SEVERE"};

constexpr uint32_t LINK_TIMEOUT_MS = 2000;  // no STATUS for this long -> NO DATA screen
constexpr uint32_t BUMP_SHOW_MS = 5000;

enum Screen : uint8_t { SC_NONE, SC_NODATA, SC_INIT, SC_IDLE, SC_BUMP, SC_ALARM, SC_FAULT };

// latest input
static uint8_t g_state = 0, g_level = 0;
static float g_peak = 0, g_rms = 0, g_freq = 0;
static uint32_t g_lastStatusMs = 0;
static bool g_everStatus = false;
static float g_bumpPeak = 0, g_bumpDur = 0;
static uint32_t g_bumpUntil = 0;

// alarm bookkeeping: the display holds the strongest values seen during the alarm
static float g_alarmPeak = 0;
static uint8_t g_alarmLevel = 1;

static Screen g_screen = SC_NONE;
static char g_drawnA[24] = "", g_drawnB[24] = "", g_drawnC[24] = "";  // text currently on screen

static uint16_t col(uint8_t r, uint8_t g, uint8_t b) { return lcd.color565(r, g, b); }

static void text(const char* s, int y, const lgfx::IFont* font, float size, uint16_t fg, uint16_t bg, int padW) {
  lcd.setFont(font);
  lcd.setTextSize(size);
  lcd.setTextColor(fg, bg);
  lcd.setTextDatum(lgfx::middle_center);
  lcd.setTextPadding(padW);
  lcd.drawString(s, 120, y);
}

static void applyScreen(Screen s) {
  g_screen = s;
  g_drawnA[0] = g_drawnB[0] = g_drawnC[0] = 0;
  const uint16_t white = TFT_WHITE;
  switch (s) {
    case SC_NODATA: {
      uint16_t bg = col(45, 45, 50);
      lcd.fillScreen(bg);
      text("NO DATA", 112, &fonts::FreeSansBold18pt7b, 1, white, bg, 0);
      text("check the bridge", 150, &fonts::FreeSansBold9pt7b, 1, col(170, 170, 175), bg, 0);
      break;
    }
    case SC_INIT: {
      uint16_t bg = col(30, 60, 95);
      lcd.fillScreen(bg);
      text("CALIBRATING", 112, &fonts::FreeSansBold12pt7b, 1, white, bg, 0);
      text("not armed yet", 146, &fonts::FreeSansBold9pt7b, 1, col(170, 190, 215), bg, 0);
      break;
    }
    case SC_IDLE: {
      uint16_t bg = col(0, 150, 60);
      lcd.fillScreen(bg);
      text("MONITORING", 108, &fonts::FreeSansBold12pt7b, 1, white, bg, 0);
      text("no shake detected", 142, &fonts::FreeSansBold9pt7b, 1, col(210, 245, 220), bg, 0);
      break;
    }
    case SC_BUMP: {
      uint16_t bg = col(0, 150, 60);
      lcd.fillScreen(bg);
      text("BUMP", 96, &fonts::FreeSansBold24pt7b, 1.5f, white, bg, 0);
      text("detected", 140, &fonts::FreeSansBold12pt7b, 1, white, bg, 0);
      break;
    }
    case SC_ALARM: {
      uint16_t bg = col(205, 0, 0);
      lcd.fillScreen(bg);
      text("ALARM", 36, &fonts::FreeSansBold12pt7b, 1, white, bg, 0);
      break;
    }
    case SC_FAULT: {
      uint16_t bg = col(235, 150, 0);
      lcd.fillScreen(bg);
      text("SENSOR FAULT", 112, &fonts::FreeSansBold12pt7b, 1, TFT_BLACK, bg, 0);
      text("not monitoring", 146, &fonts::FreeSansBold9pt7b, 1, col(60, 35, 0), bg, 0);
      break;
    }
    default:
      break;
  }
}

// Redraw only text that changed, padded over the old text, so the screen never flickers.
static void updateValues() {
  char a[24], b[24];
  if (g_screen == SC_ALARM) {
    uint16_t bg = col(205, 0, 0);
    snprintf(a, sizeof a, "%u", g_alarmLevel);
    snprintf(b, sizeof b, "%s", LEVEL_NAME[g_alarmLevel > 5 ? 5 : g_alarmLevel]);
    if (strcmp(a, g_drawnA) || strcmp(b, g_drawnB)) {
      text(a, 100, &fonts::FreeSansBold24pt7b, 2.4f, TFT_WHITE, bg, 90);
      text(b, 163, &fonts::FreeSansBold18pt7b, 1, TFT_WHITE, bg, 190);
      strcpy(g_drawnA, a);
      strcpy(g_drawnB, b);
    }
    char p[24];
    snprintf(p, sizeof p, "%.3f g", g_alarmPeak);
    if (strcmp(p, g_drawnC)) {
      text(p, 200, &fonts::FreeSansBold12pt7b, 1, TFT_WHITE, bg, 150);
      strcpy(g_drawnC, p);
    }
  } else if (g_screen == SC_BUMP) {
    uint16_t bg = col(0, 150, 60);
    snprintf(a, sizeof a, "peak %.3f g", g_bumpPeak);
    if (strcmp(a, g_drawnA)) {
      text(a, 178, &fonts::FreeSansBold9pt7b, 1, col(210, 245, 220), bg, 150);
      strcpy(g_drawnA, a);
    }
  }
}

static Screen wantedScreen(uint32_t now) {
  if (!g_everStatus || now - g_lastStatusMs > LINK_TIMEOUT_MS) return SC_NODATA;
  switch (g_state) {
    case 3: return SC_ALARM;
    case 4: return SC_FAULT;
    case 0: return SC_INIT;
    default: return (int32_t)(g_bumpUntil - now) > 0 ? SC_BUMP : SC_IDLE;
  }
}

static void handleLine(char* line, uint32_t now) {
  char* save = nullptr;
  char* tag = strtok_r(line, ",", &save);
  if (!tag) return;
  float v[5] = {0};
  int n = 0;
  for (char* t = strtok_r(nullptr, ",", &save); t && n < 5; t = strtok_r(nullptr, ",", &save)) v[n++] = atof(t);
  if (tag[0] == 'S' && n >= 5) {
    g_state = (uint8_t)v[0];
    g_level = (uint8_t)v[1];
    g_peak = v[2];
    g_rms = v[3];
    g_freq = v[4];
    g_lastStatusMs = now;
    g_everStatus = true;
    if (g_state == 3) {
      if (g_screen != SC_ALARM) {  // alarm starts: begin tracking afresh
        g_alarmPeak = 0;
        g_alarmLevel = 1;
      }
      if (g_peak > g_alarmPeak) g_alarmPeak = g_peak;
      uint8_t l = g_level < 1 ? 1 : g_level;
      if (l > g_alarmLevel) g_alarmLevel = l;
    }
  } else if (tag[0] == 'B' && n >= 2) {
    g_bumpPeak = v[0];
    g_bumpDur = v[1];
    g_bumpUntil = now + BUMP_SHOW_MS;
    g_screen = g_screen == SC_BUMP ? SC_NONE : g_screen;  // restart the banner redraw
  }
}

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.setBrightness(255);
  applyScreen(SC_NODATA);
  Serial.println("EQ-DISPLAY ready");
}

void loop() {
  static char buf[96];
  static size_t len = 0;
  uint32_t now = millis();
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) {
        buf[len] = 0;
        handleLine(buf, now);
      }
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;  // overlong garbage: drop it
    }
  }
  Screen want = wantedScreen(now);
  if (want != g_screen) applyScreen(want);
  updateValues();
  delay(10);
}
