#include "hal.h"
#include "config.h"

#if HAL_RP2040
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>

// ---- critical sections -----------------------------------------------------------------------
static spin_lock_t* const s_lock = spin_lock_init(spin_lock_claim_unused(true));
uint32_t halSpinEnter() { return spin_lock_blocking(s_lock); }
void halSpinLeave(uint32_t state) { spin_unlock(s_lock, state); }

// ---- tasks and interrupts --------------------------------------------------------------------
void halTaskCreate(TaskFunction_t fn, const char* name, uint32_t stackBytes, void* arg, int prio,
                   int core, TaskHandle_t* handle) {
  xTaskCreateAffinitySet(fn, name, stackBytes / 2, arg, prio, (UBaseType_t)1u << core, handle);
}

void halAttachRisingIsr(int pin, void (*fn)(void*), void* arg) {
  attachInterruptParam(pin, fn, RISING, arg);
}

// ---- PWM -------------------------------------------------------------------------------------
namespace {
struct PwmChan {
  int pin = -1;
  uint32_t slice = 0, chan = 0, hz = 1000, wrap = 255;
  uint8_t bits = 8;
  bool tone = false;
};
PwmChan g_pwm[8];

void applyFixed(PwmChan& c) {
  c.tone = false;
  c.wrap = (1u << c.bits) - 1;
  float div = (float)clock_get_hz(clk_sys) / ((float)c.hz * (float)(c.wrap + 1));
  if (div < 1.0f) div = 1.0f;
  if (div > 255.9f) div = 255.9f;
  pwm_set_clkdiv(c.slice, div);
  pwm_set_wrap(c.slice, c.wrap);
}
}  // namespace

void halPwmSetup(int ch, uint32_t hz, uint8_t bits) {
  PwmChan& c = g_pwm[ch];
  c.hz = hz;
  c.bits = bits;
  if (c.pin >= 0) {
    applyFixed(c);
    pwm_set_chan_level(c.slice, c.chan, 0);
  }
}

void halPwmAttachPin(int pin, int ch) {
  PwmChan& c = g_pwm[ch];
  c.pin = pin;
  gpio_set_function(pin, GPIO_FUNC_PWM);
  c.slice = pwm_gpio_to_slice_num(pin);
  c.chan = pwm_gpio_to_channel(pin);
  applyFixed(c);
  pwm_set_chan_level(c.slice, c.chan, 0);
  pwm_set_enabled(c.slice, true);
}

void halPwmWrite(int ch, uint32_t duty) {
  PwmChan& c = g_pwm[ch];
  if (c.pin < 0) return;
  if (c.tone) applyFixed(c);  // back from tone mode to the fixed frequency
  if (duty > c.wrap + 1) duty = c.wrap + 1;
  pwm_set_chan_level(c.slice, c.chan, duty);
}

void halPwmTone(int ch, uint32_t hz) {
  PwmChan& c = g_pwm[ch];
  if (c.pin < 0) return;
  if (hz == 0) {
    pwm_set_chan_level(c.slice, c.chan, 0);
    return;
  }
  const float div = 4.0f;  // 133 MHz / 4 = 33 MHz counter: 16-bit wrap covers 500 Hz and up
  uint32_t wrap = (uint32_t)((float)clock_get_hz(clk_sys) / (div * (float)hz)) - 1;
  if (wrap > 65535) wrap = 65535;
  c.tone = true;
  pwm_set_clkdiv(c.slice, div);
  pwm_set_wrap(c.slice, wrap);
  pwm_set_chan_level(c.slice, c.chan, (wrap + 1) / 2);
}

// ---- crash recorder --------------------------------------------------------------------------
// arduino-pico turns a FreeRTOS assert, stack overflow or malloc failure into panic(), which drops
// the board into the USB bootloader and leaves it looking dead. Instead: note why in the watchdog
// scratch registers (they survive a watchdog reset), reboot normally, and report it from the next
// boot over the log channel. Linked in through -Wl,--wrap (see platformio.ini).
#include <hardware/watchdog.h>
namespace {
constexpr uint32_t kCrashMagic = 0xC4A5D000u;
void recordCrash(uint32_t kind, uint32_t where, const char* text) {
  uint32_t w[2] = {0, 0};
  if (text) strncpy((char*)w, text, 8);
  watchdog_hw->scratch[0] = kCrashMagic | kind;
  watchdog_hw->scratch[1] = where;
  watchdog_hw->scratch[2] = w[0];
  watchdog_hw->scratch[3] = w[1];
  watchdog_reboot(0, 0, 5);
  for (;;) {
  }
}
}  // namespace

extern "C" {
void vApplicationStackOverflowHook(TaskHandle_t, char* name) {
  recordCrash(1, 0, name);
}
void vApplicationMallocFailedHook(void) { recordCrash(2, 0, nullptr); }
void __real_rtosFatalError(void);
void __wrap_rtosFatalError(void) {  // configASSERT failed: the return address is the assert site
  recordCrash(3, (uint32_t)__builtin_return_address(0), nullptr);
}
void __wrap_panic(const char* fmt, ...) {
  recordCrash(4, (uint32_t)__builtin_return_address(0), fmt);
}
}

void halWatchdogEnable() { watchdog_enable(2000, true); }
void halWatchdogFeed() { watchdog_update(); }

bool halLastCrash(char* out, size_t n) {
  uint32_t m = watchdog_hw->scratch[0];
  if ((m & 0xFFFFF000u) != kCrashMagic) {
    if (!watchdog_enable_caused_reboot()) return false;
    snprintf(out, n, "PREVIOUS BOOT ENDED IN A WATCHDOG RESET (firmware stopped feeding it)");
    return true;
  }
  uint32_t kind = m & 0xFFF, where = watchdog_hw->scratch[1];
  char text[9] = {0};
  memcpy(text, (const void*)&watchdog_hw->scratch[2], 8);
  watchdog_hw->scratch[0] = 0;
  const char* what = kind == 1 ? "stack overflow in task" : kind == 2 ? "malloc failed" : kind == 3 ? "FreeRTOS assert" : "panic";
  snprintf(out, n, "PREVIOUS BOOT CRASHED: %s '%s' at 0x%08lx", what, text, (unsigned long)where);
  return true;
}

// ---- misc ------------------------------------------------------------------------------------
uint32_t halFreeHeap() { return (uint32_t)rp2040.getFreeHeap(); }

uint32_t halBatteryMv() {
  analogReadResolution(12);
  uint32_t raw = analogRead(PIN_VBAT);
  return raw * 3300u * 2u / 4095u;  // 3.3 V reference, 1:2 divider
}

#else  // ESP32

bool halLastCrash(char*, size_t) { return false; }
void halWatchdogEnable() {}
void halWatchdogFeed() {}

void halTaskCreate(TaskFunction_t fn, const char* name, uint32_t stackBytes, void* arg, int prio,
                   int core, TaskHandle_t* handle) {
  xTaskCreatePinnedToCore(fn, name, stackBytes, arg, prio, handle, core);
}

void halAttachRisingIsr(int pin, void (*fn)(void*), void* arg) {
  attachInterruptArg(pin, fn, arg, RISING);
}

uint32_t halFreeHeap() { return ESP.getFreeHeap(); }
uint32_t halBatteryMv() { return analogReadMilliVolts(PIN_VBAT) * 2; }

#endif
