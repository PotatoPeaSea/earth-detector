// Unshakeable Earth Detector - ESP32 firmware
// Two ADXL345s on separate I2C buses, sampled at 800 Hz by their own clocks (FIFO + watermark
// interrupt), decimated to 200 Hz, band-passed, classified on-board (bump vs earthquake), with
// LED/buzzer alarms and a binary COBS frame stream over USB serial for the dashboard.
#include <Arduino.h>
#include "alerts.h"
#include "config.h"
#include "detector.h"
#include "params.h"
#include "protocol.h"
#include "sensor.h"

void setup() {
  Serial.setTxBufferSize(8192);
  Serial.setRxBufferSize(1024);
  Serial.begin(SERIAL_BAUD);

  protocolInit();
  paramsLoad();
  alertsInit();
  xTaskCreatePinnedToCore(commsTask, "comms", 4096, nullptr, 5, nullptr, 0);
  sendHello();
  logf("Earth detector firmware %s booting", FW_VERSION);

  sensorsInit();
  xTaskCreatePinnedToCore(detectorTask, "detector", 6144, nullptr, 18, nullptr, 1);
  xTaskCreatePinnedToCore(alertTask, "alerts", 3072, nullptr, 10, nullptr, 1);
  sendParams();
}

void loop() { vTaskDelete(nullptr); }
