#pragma once
// Кто держал loop, когда сработал task watchdog. Crash handler ESPHome сохраняет
// backtrace только того ядра, где отработало прерывание сторожа, а это обычно
// простаивающее ядро 1, и виновника там не видно (power-meter-mk, 28.10.2026).
// IDF зовет esp_task_wdt_isr_user_handler из прерывания сторожа прямо перед panic:
// здесь запоминаем текущий компонент ESPHome и задачи на обоих ядрах в RTC-память,
// она переживает программный сброс. После загрузки packages/wdt_trace.yaml
// показывает запись в логе и в сенсоре "WDT Trace".

#ifdef USE_ESP32

#include <cstring>
#include <cstdio>
#include <string>
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

struct WdtTraceRec {
  uint32_t magic;
  uint32_t uptime_s;
  char comp[40];
  char task0[16];
  char task1[16];
};

static const uint32_t WDT_TRACE_MAGIC = 0x57445431;  // "WDT1"
RTC_NOINIT_ATTR static WdtTraceRec wdt_trace_rec;

static void wdt_trace_copy_(char *dst, size_t n, const char *src) {
  if (src == nullptr) src = "-";
  strncpy(dst, src, n - 1);
  dst[n - 1] = 0;
}

extern "C" void esp_task_wdt_isr_user_handler(void) {
  esphome::Component *c = esphome::App.get_current_component();
  wdt_trace_copy_(wdt_trace_rec.comp, sizeof(wdt_trace_rec.comp),
                  c != nullptr ? LOG_STR_ARG(c->get_component_log_str()) : "none");
  wdt_trace_copy_(wdt_trace_rec.task0, sizeof(wdt_trace_rec.task0),
                  pcTaskGetName(xTaskGetCurrentTaskHandleForCore(0)));
  wdt_trace_copy_(wdt_trace_rec.task1, sizeof(wdt_trace_rec.task1),
                  pcTaskGetName(xTaskGetCurrentTaskHandleForCore(1)));
  wdt_trace_rec.uptime_s = esphome::millis() / 1000;
  wdt_trace_rec.magic = WDT_TRACE_MAGIC;
}

// Пустая строка, если прошлая перезагрузка была не от сторожа. Запись читается один раз.
static std::string wdt_trace_take() {
  if (wdt_trace_rec.magic != WDT_TRACE_MAGIC) return "";
  wdt_trace_rec.magic = 0;
  char buf[120];
  snprintf(buf, sizeof(buf), "%s; core0 %s, core1 %s; uptime %us", wdt_trace_rec.comp,
           wdt_trace_rec.task0, wdt_trace_rec.task1, (unsigned) wdt_trace_rec.uptime_s);
  return buf;
}

#endif
