#include "protocol.h"
#include <stdarg.h>
#include "alerts.h"
#include "config.h"
#include "detector.h"
#include "params.h"
#include "sensor.h"

// Producers queue unframed (type + payload). The comms task, the only consumer, stamps the
// sequence number, CRC and COBS in send order, so sequence numbers on the wire never swap.
struct TxFrame {
  uint16_t len;  // type + payload
  uint8_t data[1 + 280];
};

static QueueHandle_t s_txq = nullptr;
static volatile uint32_t s_dropped = 0;
static uint16_t s_seq = 0;  // touched only by commsTask

uint32_t txDropped() { return s_dropped; }

static uint16_t crc16(const uint8_t* p, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)p[i] << 8;
    for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

static size_t cobsEncode(const uint8_t* in, size_t len, uint8_t* out) {
  size_t r = 0, w = 1, codeIdx = 0;
  uint8_t code = 1;
  while (r < len) {
    if (in[r] == 0) {
      out[codeIdx] = code;
      code = 1;
      codeIdx = w++;
      r++;
    } else {
      out[w++] = in[r++];
      if (++code == 0xFF) {
        out[codeIdx] = code;
        code = 1;
        codeIdx = w++;
      }
    }
  }
  out[codeIdx] = code;
  return w;
}

static size_t cobsDecode(const uint8_t* in, size_t len, uint8_t* out, size_t cap) {
  size_t r = 0, w = 0;
  while (r < len) {
    uint8_t code = in[r++];
    if (code == 0) return 0;
    for (uint8_t i = 1; i < code; i++) {
      if (r >= len || w >= cap) return 0;
      out[w++] = in[r++];
    }
    if (code != 0xFF && r < len) {
      if (w >= cap) return 0;
      out[w++] = 0;
    }
  }
  return w;
}

void protocolInit() { s_txq = xQueueCreate(64, sizeof(TxFrame)); }

bool sendFrame(uint8_t type, const uint8_t* payload, size_t len) {
  if (!s_txq || len > 280) return false;
  TxFrame f;
  f.data[0] = type;
  memcpy(f.data + 1, payload, len);
  f.len = (uint16_t)(len + 1);
  if (xQueueSend(s_txq, &f, 0) != pdTRUE) {
    s_dropped++;
    return false;
  }
  return true;
}

void logf(const char* fmt, ...) {
  char buf[200];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
  sendFrame(F_LOG, (const uint8_t*)buf, n);
}

void sendParams() {
  PayloadWriter w;
  w.u8(P_COUNT);
  for (int i = 0; i < P_COUNT; i++) w.f32(g_params[i]);
  sendFrame(F_PARAMS, w);
}

void sendHello() {
  PayloadWriter w;
  w.u8(PROTO_VERSION);
  w.u8(NUM_SENSORS);
  w.f32(ODR_NOMINAL);
  w.u8(DECIM);
  w.f32(OUT_SCALE);
  const char* v = FW_VERSION;
  w.put(v, strlen(v));
  sendFrame(F_HELLO, w);
}

static float rdF32(const uint8_t* p) {
  float v;
  memcpy(&v, p, 4);
  return v;
}

static void handleCommand(const uint8_t* f, size_t n) {
  if (n < 5) return;
  uint16_t crc = f[n - 2] | (f[n - 1] << 8);
  if (crc16(f, n - 2) != crc) return;
  uint8_t type = f[0];
  const uint8_t* p = f + 3;
  size_t pl = n - 5;
  switch (type) {
    case C_SET_PARAM:
      if (pl >= 5) {
        paramsSet(p[0], rdF32(p + 1));
        sendParams();
      }
      break;
    case C_REZERO:
      sensorsRequestZero(pl >= 1 ? p[0] : 0x03);
      logf("Re-zero requested");
      break;
    case C_RAW_MODE:
      g_rawMode = pl >= 1 && p[0];
      logf("Raw 800 Hz stream %s", g_rawMode ? "on" : "off");
      break;
    case C_TEST_ALARM:
      if (pl >= 1 && p[0]) alertsTestMelody(p[0]);  // payload 1/2: play the chime / jingle only
      else alertsTest(2000);
      break;
    case C_SILENCE:
      detectorSilence();
      break;
    case C_SET_CAL:
      if (pl >= 25) {
        Cal c;
        for (int k = 0; k < 3; k++) {
          c.off[k] = rdF32(p + 1 + 4 * k);
          c.gain[k] = rdF32(p + 13 + 4 * k);
        }
        calSet(p[0], c);
        logf("S%d calibration stored", p[0] + 1);
      }
      break;
    case C_GET_PARAMS:
      sendHello();
      sendParams();
      break;
    case C_RESET_PARAMS:
      paramsReset();
      sendParams();
      break;
  }
}

#if HAL_RP2040
// The dashboard and monitor.py drop DTR right after opening the port (to stop the ESP32 resetting).
// arduino-pico's USB CDC stays silent while DTR is low unless ignoreFlowControl is set (done in
// startup()), and then still truncates any write bigger than its 256-byte FIFO. So write in
// chunks that fit, wait briefly for the host to drain, and drop whole frames when nobody is
// reading (battery operation) instead of blocking the comms task.
//
// A full FIFO with nobody reading also has to be re-kicked: TinyUSB only starts an IN transfer on
// a flush, so after the host opens the port (or reopens it) nothing would ever drain the stale
// bytes. Flushing on every skipped frame is cheap and non-blocking.
static bool s_usbStalled = false;
static void usbWrite(const uint8_t* p, size_t n) {
  if (s_usbStalled && Serial.availableForWrite() < 64) {
    Serial.flush();
    return;
  }
  uint32_t t0 = millis();
  while (n) {
    int avail = Serial.availableForWrite();
    if (avail > 0) {
      size_t k = n < (size_t)avail ? n : (size_t)avail;
      Serial.write(p, k);
      p += k;
      n -= k;
      s_usbStalled = false;
      t0 = millis();
    } else if (millis() - t0 > 20) {
      s_usbStalled = true;
      Serial.flush();
      return;
    } else {
      vTaskDelay(1);
    }
  }
}
#define SERIAL_WRITE(p, n) usbWrite((p), (n))
#else
#define SERIAL_WRITE(p, n) Serial.write((p), (n))
#endif

static void writeFrame(const TxFrame& f) {
  static uint8_t raw[1 + 2 + 280 + 2], enc[300];
  size_t plen = f.len - 1;
  raw[0] = f.data[0];
  raw[1] = s_seq & 0xFF;
  raw[2] = s_seq >> 8;
  s_seq++;
  memcpy(raw + 3, f.data + 1, plen);
  uint16_t crc = crc16(raw, plen + 3);
  raw[plen + 3] = crc & 0xFF;
  raw[plen + 4] = crc >> 8;
  size_t n = cobsEncode(raw, plen + 5, enc);
  enc[n++] = 0x00;
  SERIAL_WRITE(enc, n);
}

void commsTask(void*) {
  static TxFrame f;
  static uint8_t rx[300], dec[300];
  size_t rxn = 0;
  bool rxOverflow = false;
  for (;;) {
    if (xQueueReceive(s_txq, &f, pdMS_TO_TICKS(5)) == pdTRUE) {
      writeFrame(f);
      // drain whatever else is queued without waiting
      while (xQueueReceive(s_txq, &f, 0) == pdTRUE) writeFrame(f);
    }
    while (Serial.available()) {
      uint8_t b = Serial.read();
      if (b == 0) {
        if (!rxOverflow && rxn > 0) {
          size_t n = cobsDecode(rx, rxn, dec, sizeof(dec));
          if (n) handleCommand(dec, n);
        }
        rxn = 0;
        rxOverflow = false;
      } else if (rxn < sizeof(rx)) {
        rx[rxn++] = b;
      } else {
        rxOverflow = true;
      }
    }
  }
}
