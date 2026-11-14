#pragma once
#include <Wire.h>

namespace adxl {
enum Reg : uint8_t {
  DEVID = 0x00,
  BW_RATE = 0x2C,
  POWER_CTL = 0x2D,
  INT_ENABLE = 0x2E,
  INT_MAP = 0x2F,
  INT_SOURCE = 0x30,
  DATA_FORMAT = 0x31,
  DATAX0 = 0x32,
  FIFO_CTL = 0x38,
  FIFO_STATUS = 0x39,
};
}

struct RawSample {
  int16_t x, y, z;
};

// Self-test result codes
enum : uint8_t { ST_UNKNOWN = 0, ST_PASS = 1, ST_FAIL = 2 };

class ADXL345 {
 public:
  ADXL345(TwoWire& w, uint8_t addr) : wire_(w), addr_(addr) {}

  bool probe();                       // DEVID check with retries
  uint8_t selfTest(int16_t delta[3]); // ST_PASS / ST_FAIL / ST_UNKNOWN (I2C error)
  bool startStream();                 // 800 Hz, full-res +/-16 g, FIFO stream, watermark on INT1
  int fifoEntries(bool* overrun);     // -1 on I2C error
  bool readSample(RawSample& s);      // pops one FIFO entry

  bool write(uint8_t reg, uint8_t v);
  bool read(uint8_t reg, uint8_t* buf, size_t n);

  uint8_t firstDevId = 0;
  uint8_t devId = 0;
  uint32_t i2cErrors = 0;

 private:
  bool averageXYZ(int n, float out[3]);
  TwoWire& wire_;
  uint8_t addr_;
};

// Clocks a stuck slave off the bus and issues a STOP. Call before Wire.begin().
void i2cBusRecover(int sda, int scl);
