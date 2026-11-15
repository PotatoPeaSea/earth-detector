#pragma once
// Portability layer. The firmware builds for two targets from the same sources:
//   * ESP32 (original target, Arduino-ESP32, FreeRTOS built in)
//   * RP2040 (YD-RP2040, arduino-pico with FreeRTOS SMP enabled)
// Everything chip-specific (timers, critical sections, task pinning, PWM, interrupts, ADC, heap)
// lives behind the small API below so the signal chain, detector and protocol code stay identical.
#include <Arduino.h>

#if defined(ARDUINO_ARCH_RP2040)
#define HAL_RP2040 1
#define HAL_NAME "rp2040"
#include <FreeRTOS.h>
#include <hardware/sync.h>
#include <pico/time.h>
#include <queue.h>
#include <task.h>
// Keep the interrupt handler in RAM so a flash write (settings save) cannot stall it.
#define HAL_ISR_ATTR __not_in_flash("hal_isr")
#define HAL_YIELD_FROM_ISR(woken) portYIELD_FROM_ISR(woken)
#else
#define HAL_RP2040 0
#define HAL_NAME "esp32"
#include <esp_timer.h>
#define HAL_ISR_ATTR IRAM_ATTR
#define HAL_YIELD_FROM_ISR(woken) \
  do {                            \
    if (woken) portYIELD_FROM_ISR(); \
  } while (0)
#endif

// ---- time ------------------------------------------------------------------------------------
// Microseconds from the chip's crystal-driven free-running timer (ESP32: esp_timer, RP2040: TIMER).
inline int64_t halNowUs() {
#if HAL_RP2040
  return (int64_t)time_us_64();
#else
  return esp_timer_get_time();
#endif
}

// ---- critical sections -----------------------------------------------------------------------
// Short sections shared between tasks, and between a task and an interrupt handler, on either core.
// RP2040 uses one hardware spinlock for all of them (the sections are a few dozen bytes long).
#if HAL_RP2040
uint32_t halSpinEnter();
void halSpinLeave(uint32_t state);
#endif

class SpinMux {
 public:
  uint32_t enter() {
#if HAL_RP2040
    return halSpinEnter();
#else
    portENTER_CRITICAL(&m_);
    return 0;
#endif
  }
  void leave(uint32_t s) {
#if HAL_RP2040
    halSpinLeave(s);
#else
    (void)s;
    portEXIT_CRITICAL(&m_);
#endif
  }
  uint32_t enterIsr() {
#if HAL_RP2040
    return halSpinEnter();
#else
    portENTER_CRITICAL_ISR(&m_);
    return 0;
#endif
  }
  void leaveIsr(uint32_t s) {
#if HAL_RP2040
    halSpinLeave(s);
#else
    (void)s;
    portEXIT_CRITICAL_ISR(&m_);
#endif
  }

 private:
#if !HAL_RP2040
  portMUX_TYPE m_ = portMUX_INITIALIZER_UNLOCKED;
#endif
};

struct CritGuard {  // task context
  explicit CritGuard(SpinMux& m) : m_(m), s_(m.enter()) {}
  ~CritGuard() { m_.leave(s_); }
  CritGuard(const CritGuard&) = delete;
  CritGuard& operator=(const CritGuard&) = delete;

 private:
  SpinMux& m_;
  uint32_t s_;
};

struct CritIsrGuard {  // interrupt handler context
  explicit CritIsrGuard(SpinMux& m) : m_(m), s_(m.enterIsr()) {}
  ~CritIsrGuard() { m_.leaveIsr(s_); }
  CritIsrGuard(const CritIsrGuard&) = delete;
  CritIsrGuard& operator=(const CritIsrGuard&) = delete;

 private:
  SpinMux& m_;
  uint32_t s_;
};

// ---- tasks and interrupts --------------------------------------------------------------------
// Creates a task pinned to one core. stackBytes follows the ESP32 convention; on RP2040 the stack
// is allocated in 32-bit words and doubled (Cortex-M0+ soft-float code is stack hungry).
// Priorities are chip specific: the RP2040 FreeRTOS build only has levels 0-7 (see config.h).
void halTaskCreate(TaskFunction_t fn, const char* name, uint32_t stackBytes, void* arg, int prio,
                   int core, TaskHandle_t* handle);

// Rising-edge GPIO interrupt with a user argument.
void halAttachRisingIsr(int pin, void (*fn)(void*), void* arg);

// ---- PWM (LED brightness and buzzer tone) ----------------------------------------------------
// Channel-based like ESP32 LEDC. On RP2040 a channel is a pin; two pins that share a PWM slice
// (GPIO 2n and 2n+1) must use the same frequency, which config.h's pin choice guarantees.
#if HAL_RP2040
void halPwmSetup(int ch, uint32_t hz, uint8_t bits);
void halPwmAttachPin(int pin, int ch);
void halPwmWrite(int ch, uint32_t duty);
void halPwmTone(int ch, uint32_t hz);  // 50 % duty square wave, 0 = silent
#else
inline void halPwmSetup(int ch, uint32_t hz, uint8_t bits) { ledcSetup(ch, hz, bits); }
inline void halPwmAttachPin(int pin, int ch) { ledcAttachPin(pin, ch); }
inline void halPwmWrite(int ch, uint32_t duty) { ledcWrite(ch, duty); }
inline void halPwmTone(int ch, uint32_t hz) { ledcWriteTone(ch, hz); }
#endif

// ---- misc ------------------------------------------------------------------------------------
// Hardware watchdog (RP2040 only; no-ops on ESP32). If halWatchdogFeed() is not called for 2 s the
// chip resets, which also silences a buzzer left running by a wedged firmware.
void halWatchdogEnable();
void halWatchdogFeed();

// RP2040: if the previous boot ended in a panic / assert / stack overflow, fills `out` with a
// description (once) and returns true. ESP32 keeps its own reset-reason log, so always false there.
bool halLastCrash(char* out, size_t n);
uint32_t halFreeHeap();
uint32_t halBatteryMv();  // pack voltage in mV through the 1:2 divider on PIN_VBAT
