#pragma once
// Экран мотоцикла на garage-coop-mini (T-Display-S3, 320x170): давление в четырех
// колесах KTM и аккумулятор. Показывается вместо часов, пока у платы виден браслет
// хозяина (packages/moto_screen.yaml).
//
// Колпачки AIYATO BLE TPMS, формат TPMSII, разбор взят из tpms_s3.h (плата tpms-s3,
// она висит на мотоцикле и слушает те же датчики): company id 0x0100, 16 байт
//   [0..5]   MAC датчика, первый байт позиция 0x80..0x83
//   [6..9]   давление, int32 LE, паскали
//   [10..13] температура, int32 LE, сотые градуса
//   [14]     батарея, %
//   [15]     флаг тревоги самого датчика
// Пороги те же, что по умолчанию в tpms-s3.yaml.
//
// Аккумулятор HJTZ5S-FP-C, литий-железо-фосфат 12 В 2 Ач (мануал 350 EXC-F 2025,
// раздел 22.3.1). Процент заряда BM200 считает по-своему и отдает 0 даже на
// 13,17 В, поэтому заряд тут оценивается по напряжению покоя LiFePO4.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>
#include "esphome/core/hal.h"
#include "esphome/core/time.h"
#include "esphome/components/display/display.h"

namespace moto {

using esphome::Color;
using esphome::display::BaseFont;
using esphome::display::Display;
using esphome::display::TextAlign;

static const int N = 4;
static const uint8_t MAC[N][6] = {
    {0x80, 0xEA, 0xCA, 0x01, 0x02, 0xD1},
    {0x81, 0xEA, 0xCA, 0x01, 0x03, 0x01},
    {0x82, 0xEA, 0xCA, 0x01, 0x02, 0xCA},
    {0x83, 0xEA, 0xCA, 0x01, 0x03, 0x28},
};
// Сердечники TUbliss держат 6,4-9 бар, сами покрышки 0,4-1,2
static const char *const NAME[N] = {"TUB ПЕР", "TUB ЗАД", "ШИНА ПЕР", "ШИНА ЗАД"};
static const float LO[N] = {6.4f, 6.4f, 0.4f, 0.4f};
static const float HI[N] = {9.0f, 9.0f, 1.2f, 1.2f};
// Датчик вещает раз в несколько секунд, пока в колесе есть давление
static const uint32_t TIRE_STALE_MS = 15 * 60 * 1000;
// Аккумулятор опрашивается раз в 2-10 минут
static const uint32_t BATT_STALE_MS = 30 * 60 * 1000;

struct Tire {
  float bar = NAN;
  float temp = NAN;
  int batt = -1;
  bool sensor_alarm = false;
  bool seen = false;
  uint32_t last_ms = 0;
  float pub_bar = NAN;  // что последний раз ушло в HA
  uint32_t pub_ms = 0;
};
static Tire T[N];

struct Batt {
  float volts = NAN;
  float temp = NAN;
  bool seen = false;
  uint32_t last_ms = 0;
};
static Batt B;

// Пришли новые данные, кадр перерисовать сразу, не дожидаясь смены секунды
static bool dirty = true;

// Показ экрана без хозяина, служба moto_preview: до этого момента по millis()
static uint32_t preview_until = 0;
inline bool preview() { return preview_until != 0 && (int32_t) (preview_until - esphome::millis()) > 0; }

// Номер колеса 0..3 или -1, если пакет не от наших датчиков
inline int parse(const std::vector<uint8_t> &d) {
  if (d.size() != 16) return -1;
  int idx = -1;
  for (int i = 0; i < N; i++)
    if (memcmp(d.data(), MAC[i], 6) == 0) { idx = i; break; }
  if (idx < 0) return -1;
  int32_t pa, tc;
  memcpy(&pa, &d[6], 4);
  memcpy(&tc, &d[10], 4);
  Tire &w = T[idx];
  w.bar = pa / 100000.0f;
  w.temp = tc / 100.0f;
  w.batt = d[14];
  w.sensor_alarm = d[15] != 0;
  w.seen = true;
  w.last_ms = esphome::millis();
  dirty = true;
  return idx;
}

inline void battery(float volts, float temp) {
  B.volts = volts;
  B.temp = temp;
  B.seen = true;
  B.last_ms = esphome::millis();
  dirty = true;
}

inline bool tire_fresh(int i) { return T[i].seen && esphome::millis() - T[i].last_ms < TIRE_STALE_MS; }
inline bool batt_fresh() { return B.seen && esphome::millis() - B.last_ms < BATT_STALE_MS; }

// Есть ли вообще что показывать: мотоцикла нет в гараже, значит и экран не нужен
inline bool has_data() {
  if (batt_fresh()) return true;
  for (int i = 0; i < N; i++)
    if (tire_fresh(i)) return true;
  return false;
}

// Заряд LiFePO4 по напряжению покоя, 4 банки. На зарядке и сразу после нее
// напряжение выше покоя, и оценка завышена, это нормально.
inline float lfp_soc(float v) {
  static const float V[] = {10.0f, 12.0f, 12.5f, 12.8f, 12.9f, 13.0f, 13.1f, 13.2f, 13.3f, 13.4f, 13.6f};
  static const float P[] = {0.0f, 9.0f, 14.0f, 17.0f, 20.0f, 30.0f, 40.0f, 70.0f, 90.0f, 99.0f, 100.0f};
  const int n = sizeof(V) / sizeof(V[0]);
  if (std::isnan(v)) return NAN;
  if (v <= V[0]) return 0.0f;
  if (v >= V[n - 1]) return 100.0f;
  for (int i = 1; i < n; i++)
    if (v < V[i]) return P[i - 1] + (P[i] - P[i - 1]) * (v - V[i - 1]) / (V[i] - V[i - 1]);
  return 100.0f;
}

static const Color C_BG(0, 0, 0);
static const Color C_FRAME(55, 55, 55);
static const Color C_TEXT(200, 200, 200);
static const Color C_DIM(110, 110, 110);
static const Color C_OK(0, 225, 70);
static const Color C_WARN(255, 170, 0);
static const Color C_BAD(255, 45, 45);
static const Color C_KTM(255, 102, 0);

// "сейчас", "5м", "2ч"
inline void age_str(char *buf, size_t n, uint32_t ms) {
  uint32_t m = ms / 60000;
  if (m < 1) snprintf(buf, n, "сейчас");
  else if (m < 60) snprintf(buf, n, "%uм", (unsigned) m);
  else snprintf(buf, n, "%uч", (unsigned) (m / 60));
}

inline void draw_tire(Display &it, BaseFont *ft, BaseFont *fn, int i, int x, int y, int w, int h) {
  const Tire &t = T[i];
  bool fresh = tire_fresh(i);
  bool bad = fresh && (t.bar < LO[i] || t.bar > HI[i] || t.sensor_alarm);
  Color c = !fresh ? C_DIM : (bad ? C_BAD : C_OK);
  it.rectangle(x, y, w, h, bad ? C_BAD : C_FRAME);
  it.printf(x + 5, y + 3, ft, C_TEXT, TextAlign::TOP_LEFT, "%s", NAME[i]);
  if (t.seen)
    it.printf(x + w / 2, y + 19, fn, c, TextAlign::TOP_CENTER, i < 2 ? "%.1f" : "%.2f", t.bar);
  else
    it.printf(x + w / 2, y + 19, fn, C_DIM, TextAlign::TOP_CENTER, "--");
  if (!t.seen) return;
  int by = y + h - 19;
  if (!std::isnan(t.temp))
    it.printf(x + 5, by, ft, C_DIM, TextAlign::TOP_LEFT, "%d°", (int) lroundf(t.temp));
  uint32_t age = esphome::millis() - t.last_ms;
  if (age >= 60000) {
    char a[24];  // кириллица по 2 байта на букву
    age_str(a, sizeof(a), age);
    it.printf(x + w - 5, by, ft, fresh ? C_WARN : C_DIM, TextAlign::TOP_RIGHT, "%s", a);
  } else if (t.batt >= 0) {
    // батарейка колпачка, пока данные свежие; садится, значит менять
    it.printf(x + w - 5, by, ft, t.batt < 20 ? C_WARN : C_DIM, TextAlign::TOP_RIGHT, "%d%%", t.batt);
  }
}

inline void draw_batt(Display &it, BaseFont *ft, BaseFont *fn, int x, int y, int w, int h) {
  bool fresh = batt_fresh();
  float v = B.volts;
  float soc = lfp_soc(v);
  Color c = C_DIM;
  const char *st = "НЕТ СВЯЗИ";
  if (B.seen) {
    if (v >= 13.8f) { c = C_OK; st = "ЗАРЯЖАЕТСЯ"; }
    else if (v >= 13.3f) { c = C_OK; st = "ПОЛНЫЙ"; }
    else if (v >= 13.0f) { c = C_OK; st = "НОРМА"; }
    else if (v >= 12.8f) { c = C_WARN; st = "НИЗКИЙ"; }
    else { c = C_BAD; st = "РАЗРЯЖЕН"; }
    if (!fresh) c = C_DIM;
  }
  it.rectangle(x, y, w, h, (B.seen && fresh && v < 12.8f) ? C_BAD : C_FRAME);
  it.printf(x + 5, y + 3, ft, C_TEXT, TextAlign::TOP_LEFT, "АКБ");
  if (B.seen) {
    char a[24];  // кириллица по 2 байта на букву
    age_str(a, sizeof(a), esphome::millis() - B.last_ms);
    it.printf(x + w - 5, y + 3, ft, C_DIM, TextAlign::TOP_RIGHT, "%s", a);
    it.printf(x + w / 2, y + 19, fn, c, TextAlign::TOP_CENTER, "%.2f", v);
  } else {
    it.printf(x + w / 2, y + 19, fn, C_DIM, TextAlign::TOP_CENTER, "--");
  }
  // Полоска заряда с процентом
  int bx = x + 6, bw = w - 12, by = y + 62, bh = 14;
  it.rectangle(bx, by, bw, bh, C_FRAME);
  if (!std::isnan(soc)) {
    int fw = (int) ((bw - 4) * soc / 100.0f + 0.5f);
    if (fw > 0) it.filled_rectangle(bx + 2, by + 2, fw, bh - 4, c);
    it.printf(x + w / 2, by + bh + 3, ft, C_TEXT, TextAlign::TOP_CENTER, "SOC %d%%", (int) lroundf(soc));
  } else {
    it.printf(x + w / 2, by + bh + 3, ft, C_DIM, TextAlign::TOP_CENTER, "SOC --");
  }
  it.printf(x + w / 2, y + 102, ft, c, TextAlign::TOP_CENTER, "%s", st);
  if (B.seen && !std::isnan(B.temp))
    it.printf(x + w / 2, y + 122, ft, C_DIM, TextAlign::TOP_CENTER, "%d°C", (int) lroundf(B.temp));
}

// Весь кадр. Колеса сеткой 2x2 (столбцы перед и зад, строки сердечник и покрышка),
// справа аккумулятор, сверху модель и время с секундами.
inline void draw(Display &it, BaseFont *ft, BaseFont *fn, const esphome::ESPTime &now) {
  const int W = it.get_width();
  it.fill(C_BG);
  it.printf(4, 1, ft, C_KTM, TextAlign::TOP_LEFT, "KTM 350 EXC-F");
  if (now.is_valid())
    it.printf(W - 4, 1, ft, C_TEXT, TextAlign::TOP_RIGHT, "%02d:%02d:%02d", now.hour, now.minute, now.second);
  it.horizontal_line(0, 18, W, C_FRAME);
  const int GX = 2, GY = 21, CW = 102, CH = 72, GAP = 3;
  for (int i = 0; i < N; i++) {
    int col = i % 2, row = i / 2;
    draw_tire(it, ft, fn, i, GX + col * (CW + GAP), GY + row * (CH + GAP), CW, CH);
  }
  int bx = GX + 2 * (CW + GAP);
  draw_batt(it, ft, fn, bx, GY, W - bx - 2, 2 * CH + GAP);
  dirty = false;
}

}  // namespace moto
