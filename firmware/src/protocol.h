#pragma once
#include <Arduino.h>

// Wire format: COBS(type u8, seq u16, payload..., crc16 u16) followed by 0x00.
// All multi-byte fields little-endian. CRC16-CCITT (0xFFFF, poly 0x1021) over type..payload.
enum FrameType : uint8_t {
  F_DATA = 0x01,
  F_STATUS = 0x02,
  F_EVENT = 0x03,
  F_HEALTH = 0x04,
  F_LOG = 0x05,
  F_PARAMS = 0x06,
  F_HELLO = 0x07,
  F_RAW = 0x10,
  // host -> device
  C_SET_PARAM = 0x80,   // u8 id, f32 value
  C_REZERO = 0x81,      // u8 sensor mask
  C_RAW_MODE = 0x82,    // u8 on
  C_TEST_ALARM = 0x83,
  C_SILENCE = 0x84,
  C_SET_CAL = 0x85,     // u8 sensor, f32 off[3], f32 gain[3]
  C_GET_PARAMS = 0x86,
  C_RESET_PARAMS = 0x87,
};

struct PayloadWriter {
  uint8_t buf[280];
  size_t n = 0;
  void u8(uint8_t v) { put(&v, 1); }
  void u16(uint16_t v) { put(&v, 2); }
  void i16(int16_t v) { put(&v, 2); }
  void u32(uint32_t v) { put(&v, 4); }
  void f32(float v) { put(&v, 4); }
  void put(const void* p, size_t len) {
    if (n + len > sizeof(buf)) return;
    memcpy(buf + n, p, len);
    n += len;
  }
};

void protocolInit();
bool sendFrame(uint8_t type, const uint8_t* payload, size_t len);
inline bool sendFrame(uint8_t type, const PayloadWriter& w) { return sendFrame(type, w.buf, w.n); }
void logf(const char* fmt, ...);
void sendParams();
void sendHello();
uint32_t txDropped();
void commsTask(void*);
