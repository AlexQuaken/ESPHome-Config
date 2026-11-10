// Full-screen flip clock for esp32-c6-lcd (ST7789 320x172, part of the screen is damaged, so digits are huge).
// Two cards HH and MM, Bebas Neue 180 px. A changed card flips: the old top half folds down to the hinge,
// then the new bottom half falls from the hinge. Colon blinks with seconds.
#pragma once
#define FLIPCLOCK_SCREEN 1
#define CUSTOM_SCREEN 1
#include "esphome.h"
#include <string>
#include <cstdio>

namespace flipclock {
using esphome::Color;
using esphome::display::Display;
using esphome::display::Rect;

const int W = 320, H = 172, CW = 150, CY0 = 4, CH = 164, HINGE = CY0 + CH / 2;
const int CX[2] = {4, 166};
const int INK_TOP = 35, INK_H = 128;           // Bebas Neue 180: digits ink from 35 to 163 below the text top
const int TEXT_Y = CY0 + CH / 2 - INK_TOP - INK_H / 2;
const uint32_t FLIP_MS = 520;

inline Color hex(uint32_t h) { return Color((h >> 16) & 255, (h >> 8) & 255, h & 255); }
const uint32_t BG = 0x07070e, TOP = 0x1f1c2b, BOT = 0x181622, EDGE = 0x2a2540, INK = 0xf3effd, DIMINK = 0xb9b3cf, HINGEC = 0x000000;

struct State { std::string cur[2] = {"", ""}, old[2] = {"", ""}; uint32_t t0[2] = {0, 0}; };
inline State &S() { static State s; return s; }

// one half of a card with its digits, clipped to [y0, y0+h)
inline void half(Display &it, esphome::font::Font *f, int c, bool top, const std::string &txt, int y0, int h, bool shade) {
  if (h <= 0) return;
  int x = CX[c];
  it.start_clipping(Rect(x, y0, CW, h));
  it.filled_rectangle(x, top ? CY0 : HINGE, CW, CH / 2, hex(top ? TOP : BOT));
  it.print(x + 3, TEXT_Y, f, hex(shade ? DIMINK : INK), esphome::display::TextAlign::TOP_LEFT, txt.c_str());
  it.end_clipping();
}

inline void card(Display &it, esphome::font::Font *f, int c, uint32_t now) {
  State &s = S();
  int x = CX[c];
  uint32_t dt = now - s.t0[c];
  const int HH = CH / 2;
  if (s.old[c].empty() || dt >= FLIP_MS) {
    half(it, f, c, true, s.cur[c], CY0, HH, false);
    half(it, f, c, false, s.cur[c], HINGE, HH, false);
  } else {
    float p = (float) dt / FLIP_MS;
    // behind the flap: new top, old bottom
    half(it, f, c, true, s.cur[c], CY0, HH, false);
    half(it, f, c, false, s.old[c], HINGE, HH, false);
    if (p < 0.5f) {                          // old top half folds down onto the hinge
      int h = (int) (HH * (1.0f - 2.0f * p));
      half(it, f, c, true, s.old[c], HINGE - h, h, p > 0.25f);
    } else {                                 // new bottom half falls from the hinge
      int h = (int) (HH * (2.0f * p - 1.0f));
      half(it, f, c, false, s.cur[c], HINGE, h, p < 0.75f);
    }
  }
  // card edge, hinge gap and the two side notches
  it.rectangle(x, CY0, CW, CH, hex(EDGE));
  it.filled_rectangle(x, HINGE - 1, CW, 2, hex(HINGEC));
  it.filled_rectangle(x, HINGE - 4, 3, 8, hex(BG));
  it.filled_rectangle(x + CW - 3, HINGE - 4, 3, 8, hex(BG));
  for (int k = 0; k < 2; k++) {               // cut the corners
    it.draw_pixel_at(x, CY0 + k * (CH - 1), hex(BG)); it.draw_pixel_at(x + CW - 1, CY0 + k * (CH - 1), hex(BG));
  }
}

inline void draw(Display &it, esphome::font::Font *f, bool valid, int hour, int minute, int second) {
  State &s = S();
  uint32_t now = esphome::millis();
  char hh[4], mm[4];
  if (valid) { snprintf(hh, sizeof hh, "%02d", hour); snprintf(mm, sizeof mm, "%02d", minute); }
  else { snprintf(hh, sizeof hh, "--"); snprintf(mm, sizeof mm, "--"); }
  const char *v[2] = {hh, mm};
  for (int c = 0; c < 2; c++) {
    if (s.cur[c] != v[c]) {
      s.old[c] = s.cur[c]; s.cur[c] = v[c]; s.t0[c] = now;
      if (s.old[c].empty()) s.t0[c] = now - FLIP_MS;   // first frame: no animation
    }
  }
  it.fill(hex(BG));
  card(it, f, 0, now);
  card(it, f, 1, now);
  // colon between the cards, on for the first half of every second
  bool on = valid && (now % 1000) < 500;
  Color cc = hex(on ? INK : EDGE);
  it.filled_rectangle(157, HINGE - 30, 6, 6, cc);
  it.filled_rectangle(157, HINGE + 24, 6, 6, cc);
}
}  // namespace flipclock

namespace flipclock {
inline esphome::font::Font *&F() { static esphome::font::Font *f = nullptr; return f; }
inline void set_font(esphome::font::Font *f) { F() = f; }
}  // namespace flipclock

// hook used by packages/ble_radar.yaml (#ifdef CUSTOM_SCREEN): screen mode 3 is the flip clock
namespace custom_screen {
inline bool draw(esphome::display::Display &it, int mode, esphome::ESPTime tn) {
  if (mode != 3 || flipclock::F() == nullptr) return false;
  flipclock::draw(it, flipclock::F(), tn.is_valid(), tn.hour, tn.minute, tn.second);
  return true;
}
}  // namespace custom_screen
