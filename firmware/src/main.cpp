// Unshakeable Earth Detector firmware (ESP32 or RP2040, see hal.h)
// Two ADXL345s on separate I2C buses, sampled at 800 Hz by their own clocks (FIFO + watermark
// interrupt), decimated to 200 Hz, band-passed, classified on-board (bump vs earthquake), with
// LED/buzzer alarms and a binary COBS frame stream over USB serial for the dashboard.
#include <Arduino.h>
#include "alerts.h"
#include "config.h"
#include "detector.h"
#include "display.h"
#include "hal.h"
#include "params.h"
#include "protocol.h"
#include "sensor.h"

static void startup() {
#if !HAL_RP2040
  Serial.setTxBufferSize(8192);
  Serial.setRxBufferSize(1024);
  Serial.begin(SERIAL_BAUD);
#endif

  protocolInit();
  paramsLoad();
  alertsInit();
  halTaskCreate(commsTask, "comms", 4096, nullptr, PRIO_COMMS, CORE_COMMS, nullptr);
  sendHello();
  logf("Earth detector firmware %s booting (%s)", FW_VERSION, HAL_NAME);
  char crash[100];
  if (halLastCrash(crash, sizeof(crash))) logf("%s", crash);

  sensorsInit();
  halTaskCreate(detectorTask, "detector", 6144, nullptr, PRIO_DETECTOR, CORE_DSP, nullptr);
  halTaskCreate(alertTask, "alerts", 3072, nullptr, PRIO_ALERTS, CORE_DSP, nullptr);
  sendParams();
#if HAL_RP2040
  // Lowest priority: drawing must never delay the sensors, detector or alarm.
  halTaskCreate(displayTask, "display", 8192, nullptr, 1, 0, nullptr);
#endif
  halWatchdogEnable();  // fed by alertTask
}

#if HAL_RP2040
// arduino-pico runs setup() on a fixed 4 KB task stack, too small for printf with floats and the
// ADXL self-test, so the real startup runs in a short-lived task with a bigger stack.
static void bootTask(void*) {
  startup();
  vTaskDelete(nullptr);
}

void setup() {
  // USB serial must be brought up here, in the Arduino setup task: starting it from the boot task
  // left the port silent. ignoreFlowControl makes it stream although the host keeps DTR low
  // (see usbWrite in protocol.cpp). The baud rate is ignored on USB CDC.
  Serial.ignoreFlowControl(true);
  Serial.begin(SERIAL_BAUD);
  halTaskCreate(bootTask, "boot", 8192, nullptr, PRIO_BOOT, CORE_BOOT, nullptr);
}
void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
#else
void setup() { startup(); }
void loop() { vTaskDelete(nullptr); }
#endif
