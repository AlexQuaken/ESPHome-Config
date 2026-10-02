#pragma once
// Разбор BLE-колпачков AIYATO BLE TPMS, история давления и экран загрузки
// для tpms-s3.yaml.
//
// Формат TPMSII (Type A в github.com/bkbilly/tpms_ble): company id 0x0100,
// сервис 0xFBB0, 16 байт данных производителя:
//   [0..5]   MAC датчика, первый байт это позиция 0x80..0x83
//   [6..9]   давление, int32 little endian, паскали
//   [10..13] температура, int32 little endian, сотые градуса
//   [14]     батарея, %
//   [15]     флаг тревоги самого датчика
//
// Сразу после установки батарейки датчик шлет пачку пакетов-заглушек
// 80EACA00000A040302010D0C0B0A6400, одинаковых у всех четырех. Своего MAC в
// них нет, поэтому сверка первых 6 байт с таблицей ниже отсекает их сама.
//
// Без давления датчик засыпает, тряска его не будит. Будит установка батарейки
// или давление в колесе, дальше вещает сам.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include "esp_heap_caps.h"
#include "esphome/core/hal.h"

namespace tpms {

static const int N = 4;

// Сняты сниффером 02.10.2026. Порядок в массиве = порядок плиток на экране.
static const uint8_t MAC[N][6] = {
    {0x80, 0xEA, 0xCA, 0x01, 0x02, 0xD1},
    {0x81, 0xEA, 0xCA, 0x01, 0x03, 0x01},
    {0x82, 0xEA, 0xCA, 0x01, 0x02, 0xCA},
    {0x83, 0xEA, 0xCA, 0x01, 0x03, 0x28},
};

// История: точка раз в HIST_STEP_S секунд, всего на час. Нет свежих данных,
// в точку пишется NAN, и на графике получается разрыв, а не ложная линия.
static const int HIST = 360;
static const int HIST_STEP_S = 10;

struct Wheel {
  float bar = NAN;
  float temp = NAN;
  int batt = -1;
  int rssi = 0;
  bool sensor_alarm = false;
  bool seen = false;
  uint32_t last_ms = 0;
  uint32_t packets = 0;
  float gap_s = NAN;    // сглаженный интервал между пакетами
  float bar_min = NAN;  // минимум и максимум с включения
  float bar_max = NAN;
  bool alarm = false;   // давление вне порогов или тревога от самого датчика
  bool acked = false;   // тревогу подтвердили кнопкой, мигание остановлено
  float pub_bar = NAN;  // что последний раз ушло в сенсоры HA
  uint32_t pub_ms = 0;
  float hist[HIST];
  int head = 0;         // куда пишется следующая точка
  int count = 0;
  Wheel() {
    for (int i = 0; i < HIST; i++) hist[i] = NAN;
  }
};

static Wheel W[N];

// Номер датчика 0..3 или -1, если пакет не наш.
inline int parse(const std::vector<uint8_t> &d, int rssi) {
  if (d.size() != 16) return -1;
  int idx = -1;
  for (int i = 0; i < N; i++) {
    if (memcmp(d.data(), MAC[i], 6) == 0) {
      idx = i;
      break;
    }
  }
  if (idx < 0) return -1;
  int32_t pa, tc;
  memcpy(&pa, &d[6], 4);  // ESP32 little endian, порядок совпадает
  memcpy(&tc, &d[10], 4);
  Wheel &w = W[idx];
  const uint32_t now = esphome::millis();
  if (w.seen) {
    // Пачки по несколько пакетов подряд в интервал не считаем.
    float g = (now - w.last_ms) / 1000.0f;
    if (g > 0.5f) w.gap_s = std::isnan(w.gap_s) ? g : w.gap_s * 0.8f + g * 0.2f;
  }
  w.bar = pa / 100000.0f;
  w.temp = tc / 100.0f;
  w.batt = d[14];
  w.rssi = rssi;
  w.sensor_alarm = d[15] != 0;
  w.seen = true;
  w.last_ms = now;
  w.packets++;
  if (std::isnan(w.bar_min) || w.bar < w.bar_min) w.bar_min = w.bar;
  if (std::isnan(w.bar_max) || w.bar > w.bar_max) w.bar_max = w.bar;
  return idx;
}

inline bool fresh(int i, uint32_t stale_ms) {
  return W[i].seen && (esphome::millis() - W[i].last_ms) < stale_ms;
}

// Вызывается раз в HIST_STEP_S секунд.
inline void hist_push(uint32_t stale_ms) {
  for (int i = 0; i < N; i++) {
    Wheel &w = W[i];
    w.hist[w.head] = fresh(i, stale_ms) ? w.bar : NAN;
    w.head = (w.head + 1) % HIST;
    if (w.count < HIST) w.count++;
  }
}

// k-я точка с конца: 0 это самая свежая. NAN, если ее еще нет.
inline float hist_at(int i, int k) {
  const Wheel &w = W[i];
  if (k >= w.count) return NAN;
  return w.hist[(w.head - 1 - k + HIST) % HIST];
}

// Скорость изменения давления, бар в час, по окну в minutes минут.
// NAN, пока данных на окно не набралось. Отрицательная значит утечку.
inline float rate_bar_h(int i, int minutes) {
  const int back = minutes * 60 / HIST_STEP_S;
  float now_v = hist_at(i, 0);
  if (std::isnan(now_v)) return NAN;
  for (int k = back; k >= back / 2; k--) {
    float v = hist_at(i, k);
    if (!std::isnan(v)) return (now_v - v) / (k * HIST_STEP_S / 3600.0f);
  }
  return NAN;
}

// ---------------------------------------------------------------- загрузка
// Экран загрузки в духе dmesg и systemd. Строки про BLE, NVS, датчики и Wi-Fi
// настоящие, строки про ЭБУ, зажигание и подвеску это декорация: к проводке
// мотоцикла плата не подключена.

enum BootStyle { B_KERN = 0, B_OK = 1, B_HEAD = 2, B_WARN = 3, B_END = -1 };

struct BootCtx {
  float lo[N], hi[N];
  int on_air;          // сколько датчиков уже прислали пакет
  bool wifi_up;
  const char *ip;
  uint32_t heap_kb;
  uint32_t psram_kb;
};

// Время появления строки k, мс от старта экрана загрузки.
inline uint32_t boot_at(int k) {
  static const uint16_t AT[] = {0,    150,  320,  450,  600,  820,  1000, 1250,
                                1700, 2050, 2350, 2600, 2850, 3100, 3350, 3500,
                                3650, 3800, 3950, 4500, 4900, 5300, 5700};
  const int n = sizeof(AT) / sizeof(AT[0]);
  return k < n ? AT[k] : 0xFFFFFFFFu;
}

inline int boot_line(int k, const BootCtx &c, char *t, size_t n) {
  switch (k) {
    case 0: snprintf(t, n, "KTM TPMS-OS 1.0  //  350 EXC-F"); return B_HEAD;
    case 1: snprintf(t, n, "Booting SIX DAYS kernel..."); return B_KERN;
    case 2: snprintf(t, n, "cpu: Xtensa LX7 x2 @ 240 MHz"); return B_KERN;
    case 3: snprintf(t, n, "mem: %uK heap, %uK psram", (unsigned) c.heap_kb, (unsigned) c.psram_kb); return B_KERN;
    case 4: snprintf(t, n, "nvs: %d thresholds restored", N * 2); return B_KERN;
    case 5: snprintf(t, n, "Started BLE 5 scanner"); return B_OK;
    case 6: snprintf(t, n, "can0: K-line init 10400 baud"); return B_KERN;
    case 7: snprintf(t, n, "ecu: KTM EMS handshake 0x7E0"); return B_KERN;
    case 8: snprintf(t, n, "Ignition circuit check"); return B_OK;
    case 9: snprintf(t, n, "Fuel pump prime 2.0 s"); return B_OK;
    case 10: snprintf(t, n, "Lambda probe heater"); return B_OK;
    case 11: snprintf(t, n, "WP suspension telemetry"); return B_OK;
    case 12: snprintf(t, n, "Mounted TUbliss chambers"); return B_OK;
    case 13: snprintf(t, n, "tpms: %d sensors registered", N); return B_KERN;
    case 14:
    case 15:
    case 16:
    case 17: {
      const int i = k - 14;
      snprintf(t, n, "tpms: S%d %02X%02X%02X %.1f-%.1f bar", i + 1, MAC[i][3], MAC[i][4], MAC[i][5],
               c.lo[i], c.hi[i]);
      return B_KERN;
    }
    case 18:
      if (c.on_air > 0) {
        snprintf(t, n, "tpms: %d/%d sensors on air", c.on_air, N);
        return B_OK;
      }
      snprintf(t, n, "tpms: sensors asleep, waiting");
      return B_WARN;
    case 19:
      if (c.wifi_up) {
        snprintf(t, n, "wlan0: up %s, sync on", c.ip);
        return B_OK;
      }
      snprintf(t, n, "wlan0: no carrier, offline");
      return B_WARN;
    case 20: snprintf(t, n, "Reached target Ride"); return B_OK;
    case 21: snprintf(t, n, ">>>  READY TO RACE  <<<"); return B_HEAD;
    default: return B_END;
  }
}

static const uint32_t BOOT_TOTAL_MS = 7000;

}  // namespace tpms
