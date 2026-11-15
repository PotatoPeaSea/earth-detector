#include "adxl345.h"
#include "config.h"

using namespace adxl;

bool ADXL345::write(uint8_t reg, uint8_t v) {
  wire_.beginTransmission(addr_);
  wire_.write(reg);
  wire_.write(v);
  if (wire_.endTransmission() != 0) {
    i2cErrors++;
    return false;
  }
  return true;
}

bool ADXL345::read(uint8_t reg, uint8_t* buf, size_t n) {
  wire_.beginTransmission(addr_);
  wire_.write(reg);
  if (wire_.endTransmission(false) != 0) {
    i2cErrors++;
    return false;
  }
  size_t got = wire_.requestFrom((uint16_t)addr_, (size_t)n, true);
  if (got != n) {
    i2cErrors++;
    while (wire_.available()) wire_.read();
    return false;
  }
  for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)wire_.read();
  return true;
}

bool ADXL345::probe() {
  bool gotFirst = false;
  for (int i = 0; i < 5; i++) {
    uint8_t id = 0;
    if (read(DEVID, &id, 1)) {
      if (!gotFirst) {
        firstDevId = id;
        gotFirst = true;
      }
      devId = id;
      // 0xE5 = genuine ADXL345. 0xE6 is reported consistently by common clone modules,
      // which are register-compatible; the stream checks (rate, FIFO, noise) verify them.
      if (id == 0xE5 || id == 0xE6) return true;
    }
    delay(2);
  }
  return false;
}

bool ADXL345::averageXYZ(int n, float out[3]) {
  long s[3] = {0, 0, 0};
  uint8_t b[6];
  for (int i = 0; i < n; i++) {
    delayMicroseconds(2600);  // 400 Hz ODR during self-test
    if (!read(DATAX0, b, 6)) return false;
    s[0] += (int16_t)(b[0] | (b[1] << 8));
    s[1] += (int16_t)(b[2] | (b[3] << 8));
    s[2] += (int16_t)(b[4] | (b[5] << 8));
  }
  for (int k = 0; k < 3; k++) out[k] = (float)s[k] / n;
  return true;
}

uint8_t ADXL345::selfTest(int16_t delta[3]) {
  delta[0] = delta[1] = delta[2] = 0;
  bool ok = write(POWER_CTL, 0x00) && write(INT_ENABLE, 0x00) && write(FIFO_CTL, 0x00) &&
            write(BW_RATE, 0x0C) && write(DATA_FORMAT, 0x0B) && write(POWER_CTL, 0x08);
  if (!ok) return ST_UNKNOWN;
  delay(30);
  float off[3], on[3];
  if (!averageXYZ(40, off)) return ST_UNKNOWN;
  if (!write(DATA_FORMAT, 0x8B)) return ST_UNKNOWN;
  delay(30);
  bool okOn = averageXYZ(40, on);
  write(DATA_FORMAT, 0x0B);
  write(POWER_CTL, 0x00);
  if (!okOn) return ST_UNKNOWN;
  for (int k = 0; k < 3; k++) delta[k] = (int16_t)lroundf(on[k] - off[k]);
  // Lenient limits (datasheet: X and Z move positive, Y negative, tens to hundreds of LSB).
  bool pass = delta[0] > 30 && delta[1] < -30 && delta[2] > 45;
  return pass ? ST_PASS : ST_FAIL;
}

bool ADXL345::startStream() {
  bool ok = true;
  ok &= write(POWER_CTL, 0x00);      // standby while configuring
  ok &= write(INT_ENABLE, 0x00);
  ok &= write(BW_RATE, 0x0D);        // 800 Hz, normal power
  ok &= write(DATA_FORMAT, 0x0B);    // full resolution, +/-16 g, INT active high
  ok &= write(FIFO_CTL, 0x00);       // bypass: clears the FIFO
  ok &= write(INT_MAP, 0x00);        // everything on INT1
  ok &= write(FIFO_CTL, 0x80 | FIFO_WATERMARK);  // stream mode
  ok &= write(INT_ENABLE, 0x02);     // watermark
  uint8_t v = 0;
  ok &= read(BW_RATE, &v, 1) && v == 0x0D;
  ok &= read(DATA_FORMAT, &v, 1) && v == 0x0B;
  ok &= read(FIFO_CTL, &v, 1) && v == (0x80 | FIFO_WATERMARK);
  ok &= write(POWER_CTL, 0x08);      // measure
  return ok;
}

int ADXL345::fifoEntries(bool* overrun) {
  uint8_t src = 0, st = 0;
  if (!read(INT_SOURCE, &src, 1)) return -1;
  if (!read(FIFO_STATUS, &st, 1)) return -1;
  if (overrun) *overrun = (src & 0x01) != 0;
  return st & 0x3F;
}

bool ADXL345::readSample(RawSample& s) {
  uint8_t b[6];
  if (!read(DATAX0, b, 6)) return false;
  s.x = (int16_t)(b[0] | (b[1] << 8));
  s.y = (int16_t)(b[2] | (b[3] << 8));
  s.z = (int16_t)(b[4] | (b[5] << 8));
  return true;
}

#if HAL_RP2040
// RP2040 GPIOs have no open-drain mode: emulate it by driving low or releasing to an input with the
// pull-up (the modules' own 4.7 k pull-ups do the real work).
static inline void odLow(int pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}
static inline void odRelease(int pin) { pinMode(pin, INPUT_PULLUP); }

void i2cBusRecover(int sda, int scl) {
  odRelease(sda);
  odRelease(scl);
  delayMicroseconds(10);
  for (int i = 0; i < 9 && digitalRead(sda) == LOW; i++) {
    odLow(scl);
    delayMicroseconds(10);
    odRelease(scl);
    delayMicroseconds(10);
  }
  // STOP condition: SDA rises while SCL is high
  odLow(sda);
  delayMicroseconds(10);
  odRelease(scl);
  delayMicroseconds(10);
  odRelease(sda);
  delayMicroseconds(10);
  pinMode(sda, INPUT);
  pinMode(scl, INPUT);
}
#else
void i2cBusRecover(int sda, int scl) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, OUTPUT_OPEN_DRAIN);
  digitalWrite(scl, HIGH);
  delayMicroseconds(10);
  for (int i = 0; i < 9 && digitalRead(sda) == LOW; i++) {
    digitalWrite(scl, LOW);
    delayMicroseconds(10);
    digitalWrite(scl, HIGH);
    delayMicroseconds(10);
  }
  // STOP condition: SDA rises while SCL is high
  pinMode(sda, OUTPUT_OPEN_DRAIN);
  digitalWrite(sda, LOW);
  delayMicroseconds(10);
  digitalWrite(scl, HIGH);
  delayMicroseconds(10);
  digitalWrite(sda, HIGH);
  delayMicroseconds(10);
  pinMode(sda, INPUT);
  pinMode(scl, INPUT);
}
#endif
