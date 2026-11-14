#include "protocol.h"
#include <stdarg.h>
#include "alerts.h"
#include "config.h"
#include "detector.h"
#include "params.h"
#include "sensor.h"

struct TxFrame {
  uint16_t len;
  uint8_t data[300];
};

static QueueHandle_t s_txq = nullptr;
static volatile uint32_t s_dropped = 0;
static portMUX_TYPE s_seqMux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t s_seq = 0;

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
  uint8_t raw[288];
  portENTER_CRITICAL(&s_seqMux);
  uint16_t seq = s_seq++;
  portEXIT_CRITICAL(&s_seqMux);
  raw[0] = type;
  raw[1] = seq & 0xFF;
  raw[2] = seq >> 8;
  memcpy(raw + 3, payload, len);
  uint16_t crc = crc16(raw, len + 3);
  raw[len + 3] = crc & 0xFF;
  raw[len + 4] = crc >> 8;
  TxFrame f;
  f.len = cobsEncode(raw, len + 5, f.data);
  f.data[f.len++] = 0x00;
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
      alertsTest(2000);
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

void commsTask(void*) {
  static TxFrame f;
  static uint8_t rx[300], dec[300];
  size_t rxn = 0;
  bool rxOverflow = false;
  for (;;) {
    if (xQueueReceive(s_txq, &f, pdMS_TO_TICKS(5)) == pdTRUE) {
      Serial.write(f.data, f.len);
      // drain whatever else is queued without waiting
      while (xQueueReceive(s_txq, &f, 0) == pdTRUE) Serial.write(f.data, f.len);
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
