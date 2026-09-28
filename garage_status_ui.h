// Интерфейс «Неоновые статусы» для Guition JC4827W543 (480x272), прошивка esp32-s3-status.yaml.
// Макет: D:\MegaSync\Misc\exceltable-dashboards\Гараж, выбранный вариант (v05b.py, v08b.py).
//
// Весь интерфейс строится здесь на LVGL 9 API: три слоя (главный экран, календарь, пин-панель
// поверх главного). YAML только передает данные (State) и выполняет команды касаний через
// колбэки cb_*. Каждый создаваемый объект чистится lv_obj_remove_style_all: тема ESPHome иначе
// дает белый фон, рамки и прокрутку. Кликабельны только кнопки, остальные объекты не ловят
// касание, поэтому тап по графику доходит до панели графика.
#pragma once
#include "esphome.h"
#include "lvgl.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace gs {

// ------------------------------------------------------------------ палитра
enum : uint32_t {
  BG1 = 0x060414, BG2 = 0x0e0828, PANEL = 0x120c2e, PANEL2 = 0x0c0820, EDGE = 0x3c2878,
  VIOLET = 0x8c46ff, PURPLE = 0xbe3cff, MAGENTA = 0xff32c8, PINK = 0xff5aa0, ORANGE = 0xff8c28,
  AMBER = 0xffbe3c, CYAN = 0x3cc8ff, TEXT = 0xf0ecff, SUB = 0xa096d2, DIM = 0x645a96, RED = 0xff2d55,
};
static inline lv_color_t C(uint32_t h) { return lv_color_hex(h); }
static inline uint32_t mix(uint32_t a, uint32_t b, float t) {
  auto ch = [&](int s) {
    int x = (a >> s) & 255, y = (b >> s) & 255;
    return (uint32_t) (x + (y - x) * t) & 255;
  };
  return (ch(16) << 16) | (ch(8) << 8) | ch(0);
}

// ------------------------------------------------------------------ шрифты (из YAML)
static const lv_font_t *FS = nullptr, *FM = nullptr, *FB = nullptr;
static int tw(const char *s, int cw) {  // ширина строки моноширинного шрифта, UTF-8
  int n = 0;
  for (; *s; s++)
    if ((*s & 0xC0) != 0x80) n++;
  return n * cw;
}

// ------------------------------------------------------------------ данные
struct State {
  bool gate_open = false, light_on = false, motion = false, charge_on = false;
  std::string alarm = "";              // armed_away / disarmed / triggered / ...
  int light_timeout = 5;
  float p_now = NAN, volt = NAN, p_chg = NAN;
  float d_sock = NAN, d_light = NAN, d_chg = NAN, m_sock = NAN, m_light = NAN, m_chg = NAN;
  bool sched_on = false;
  std::string sched_start = "23:00", sched_end = "07:00", line_name = "";
  bool time_ok = false;
  esphome::ESPTime now;
};
// история от HA (set_history)
static std::vector<float> H_POWER, H_MONTHS, H_DAYS;
static int H_FIRST_HOUR = 0;
// время последнего события по дорожкам (ворота, свет, охрана, движение), global во флеше
static std::array<uint32_t, 4> *LAST = nullptr;
static uint32_t last_of(int k) { return LAST ? (*LAST)[k] : 0; }
static uint32_t H_VER = 0;

// события 72 ч: 72 часовых ведра x 4 дорожки, счетчики; ведро 71 это текущий час (UTC-час)
static std::array<uint16_t, 288> *EV = nullptr;
static uint32_t *EV_BASE = nullptr;
static uint32_t EV_VER = 0;
static void ev_roll(uint32_t hour) {
  if (!EV) return;
  if (*EV_BASE == 0) { *EV_BASE = hour; return; }
  if (hour <= *EV_BASE) return;
  uint32_t sh = hour - *EV_BASE;
  auto &e = *EV;
  if (sh >= 72) e.fill(0);
  else {
    for (int i = 0; i < 72 - (int) sh; i++)
      for (int k = 0; k < 4; k++) e[i * 4 + k] = e[(i + sh) * 4 + k];
    for (int i = 72 - sh; i < 72; i++)
      for (int k = 0; k < 4; k++) e[i * 4 + k] = 0;
  }
  *EV_BASE = hour;
  EV_VER++;
}
static void ev_add(int lane, uint32_t epoch, uint32_t now_epoch) {
  if (!EV || lane < 0 || lane > 3 || epoch == 0) return;
  ev_roll(now_epoch / 3600);
  uint32_t h = epoch / 3600;
  if (h > *EV_BASE) return;
  int i = 71 - (int) (*EV_BASE - h);
  if (i < 0) return;
  auto &c = (*EV)[i * 4 + lane];
  if (c < 65535) c++;
  EV_VER++;
}
static void ev_live(int lane, uint32_t now_epoch) {
  ev_add(lane, now_epoch, now_epoch);
  if (LAST && lane >= 0 && lane < 4) (*LAST)[lane] = now_epoch;
}

// колбэки команд касаний (ставит YAML)
static std::function<void()> cb_arm, cb_disarm, cb_light, cb_charge;
static std::string PIN = "0000";
static float TARIFF = 4.32f;  // грн за кВт·ч, условный

// ------------------------------------------------------------------ помощники LVGL
static lv_obj_t *obj(lv_obj_t *p, int x, int y, int w, int h) {
  lv_obj_t *o = lv_obj_create(p);
  lv_obj_remove_style_all(o);
  lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(o, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_pos(o, x, y);
  lv_obj_set_size(o, w, h);
  return o;
}
static void fill(lv_obj_t *o, uint32_t c1, int r, uint32_t c2 = 0xFFFFFFFF, bool hor = false) {
  lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(o, C(c1), 0);
  if (c2 != 0xFFFFFFFF) {
    lv_obj_set_style_bg_grad_color(o, C(c2), 0);
    lv_obj_set_style_bg_grad_dir(o, hor ? LV_GRAD_DIR_HOR : LV_GRAD_DIR_VER, 0);
  } else {
    lv_obj_set_style_bg_grad_dir(o, LV_GRAD_DIR_NONE, 0);
  }
  lv_obj_set_style_radius(o, r, 0);
}
static void border(lv_obj_t *o, uint32_t c, int w = 1) {
  lv_obj_set_style_border_color(o, C(c), 0);
  lv_obj_set_style_border_width(o, w, 0);
  lv_obj_set_style_border_opa(o, LV_OPA_COVER, 0);
}
static void glow(lv_obj_t *o, uint32_t c, int w, lv_opa_t opa = 180) {
  lv_obj_set_style_shadow_color(o, C(c), 0);
  lv_obj_set_style_shadow_width(o, w, 0);
  lv_obj_set_style_shadow_opa(o, w ? opa : 0, 0);
  lv_obj_set_style_shadow_spread(o, 0, 0);
}
static lv_obj_t *panel(lv_obj_t *p, int x, int y, int w, int h, int r = 10, uint32_t edge = EDGE,
                       uint32_t f1 = PANEL, uint32_t f2 = PANEL2) {
  lv_obj_t *o = obj(p, x, y, w, h);
  fill(o, f1, r, f2);
  border(o, edge);
  return o;
}
// метка: align 0 слева от x, 1 по центру x, 2 справа до x
static lv_obj_t *lbl(lv_obj_t *p, int x, int y, const lv_font_t *f, uint32_t c, const char *t, int align = 0, int w = 0) {
  lv_obj_t *l = lv_label_create(p);
  lv_obj_remove_style_all(l);
  lv_obj_remove_flag(l, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_text_font(l, f, 0);
  lv_obj_set_style_text_color(l, C(c), 0);
  lv_label_set_text(l, t);
  if (align == 0) {
    lv_obj_set_pos(l, x, y);
  } else {
    if (!w) w = 160;
    lv_label_set_long_mode(l, LV_LABEL_LONG_CLIP);
    lv_obj_set_width(l, w);
    lv_obj_set_style_text_align(l, align == 1 ? LV_TEXT_ALIGN_CENTER : LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_pos(l, align == 1 ? x - w / 2 : x - w, y);
  }
  return l;
}
static void set_text(lv_obj_t *l, const char *t) {
  if (strcmp(lv_label_get_text(l), t) != 0) lv_label_set_text(l, t);
}
static void set_color(lv_obj_t *l, uint32_t c) { lv_obj_set_style_text_color(l, C(c), 0); }

// холст ARGB с прозрачным фоном
static lv_obj_t *canvas(lv_obj_t *p, int x, int y, int w, int h) {
  lv_obj_t *c = lv_canvas_create(p);
  lv_obj_remove_style_all(c);
  lv_obj_remove_flag(c, LV_OBJ_FLAG_CLICKABLE);
  lv_draw_buf_t *b = lv_draw_buf_create(w, h, LV_COLOR_FORMAT_ARGB8888, 0);
  lv_canvas_set_draw_buf(c, b);
  lv_obj_set_pos(c, x, y);
  lv_canvas_fill_bg(c, C(0), LV_OPA_TRANSP);
  return c;
}
struct Layer {
  lv_obj_t *c;
  lv_layer_t l;
  explicit Layer(lv_obj_t *cv) : c(cv) {
    lv_canvas_fill_bg(c, C(0), LV_OPA_TRANSP);
    lv_canvas_init_layer(c, &l);
  }
  ~Layer() { lv_canvas_finish_layer(c, &l); }
  void line(float x1, float y1, float x2, float y2, uint32_t col, int w = 1, lv_opa_t opa = LV_OPA_COVER) {
    lv_draw_line_dsc_t d;
    lv_draw_line_dsc_init(&d);
    d.p1.x = x1; d.p1.y = y1; d.p2.x = x2; d.p2.y = y2;
    d.color = C(col); d.width = w; d.opa = opa; d.round_start = 1; d.round_end = 1;
    lv_draw_line(&l, &d);
  }
  void tri(float ax, float ay, float bx, float by, float cx, float cy, uint32_t col, lv_opa_t opa = LV_OPA_COVER) {
    lv_draw_triangle_dsc_t d;
    lv_draw_triangle_dsc_init(&d);
    d.p[0].x = ax; d.p[0].y = ay; d.p[1].x = bx; d.p[1].y = by; d.p[2].x = cx; d.p[2].y = cy;
    d.color = C(col); d.opa = opa;
    lv_draw_triangle(&l, &d);
  }
  void poly(const std::vector<lv_point_precise_t> &p, uint32_t col, lv_opa_t opa = LV_OPA_COVER) {
    for (size_t i = 1; i + 1 < p.size(); i++) tri(p[0].x, p[0].y, p[i].x, p[i].y, p[i + 1].x, p[i + 1].y, col, opa);
  }
  void rect(float x, float y, float w, float h, int r, uint32_t fillc, lv_opa_t fo, uint32_t bc = 0, int bw = 0) {
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = C(fillc); d.bg_opa = fo; d.radius = r;
    d.border_color = C(bc); d.border_width = bw; d.border_opa = bw ? LV_OPA_COVER : LV_OPA_TRANSP;
    lv_area_t a = {(int32_t) x, (int32_t) y, (int32_t) (x + w - 1), (int32_t) (y + h - 1)};
    lv_draw_rect(&l, &d, &a);
  }
  void circle(float x, float y, float r, uint32_t fillc, lv_opa_t fo = LV_OPA_COVER, uint32_t bc = 0, int bw = 0) {
    rect(x - r, y - r, 2 * r + 1, 2 * r + 1, LV_RADIUS_CIRCLE, fillc, fo, bc, bw);
  }
  void arc(float cx, float cy, int r, int w, float a0, float a1, uint32_t col, lv_opa_t opa = LV_OPA_COVER) {
    lv_draw_arc_dsc_t d;
    lv_draw_arc_dsc_init(&d);
    d.center.x = cx; d.center.y = cy; d.radius = r; d.width = w;
    d.start_angle = a0; d.end_angle = a1; d.color = C(col); d.opa = opa;
    lv_draw_arc(&l, &d);
  }
  // дуга с переходом цвета: сегменты
  void arc_grad(float cx, float cy, int r, int w, float a0, float a1, uint32_t c1, uint32_t c2, int steps = 40) {
    for (int i = 0; i < steps; i++) {
      float t0 = a0 + (a1 - a0) * i / steps, t1 = a0 + (a1 - a0) * (i + 1) / steps + 0.8f;
      arc(cx, cy, r, w, t0, std::min(t1, a1), mix(c1, c2, (float) i / std::max(1, steps - 1)));
    }
  }
};

// ------------------------------------------------------------------ иконки плиток (16x16 на холсте 20x20)
static void icon(lv_obj_t *cv, int kind, uint32_t c) {
  Layer L(cv);
  const float x = 2, y = 2;
  switch (kind) {
    case 0:  // ворота
      L.rect(x, y + 2, 17, 14, 2, 0, LV_OPA_TRANSP, c, 1);
      for (int i = 1; i < 4; i++) L.line(x + 4 * i, y + 4, x + 4 * i, y + 13, c);
      break;
    case 1:  // щит
      L.poly({{x + 8, y}, {x + 16, y + 3}, {x + 15, y + 10}, {x + 8, y + 16}, {x + 1, y + 10}, {x, y + 3}}, c);
      L.poly({{x + 8, y + 4}, {x + 12, y + 5}, {x + 11, y + 10}, {x + 8, y + 12}, {x + 5, y + 10}, {x + 4, y + 5}}, 0xff9678);
      break;
    case 2:  // лампа
      L.circle(x + 8, y + 6, 6, 0, LV_OPA_TRANSP, c, 1);
      L.rect(x + 5, y + 12, 7, 5, 1, c, LV_OPA_COVER);
      break;
    case 3:  // движение
      L.circle(x + 3, y + 8, 2, c);
      L.arc(x + 3, y + 8, 7, 1, -45, 45, c);
      L.arc(x + 3, y + 8, 12, 1, -40, 40, c);
      break;
    case 4:  // молния
      L.poly({{x + 10, y}, {x + 3, y + 9}, {x + 8, y + 9}, {x + 6, y + 16}, {x + 13, y + 6}, {x + 8, y + 6}}, c);
      break;
  }
}

// ------------------------------------------------------------------ главный экран
static const int M = 8, G = 6, TW = 88, TH = 54;
static const int Y2 = M + TH + G, H2 = 116, PW = 3 * TW + 2 * G;
static const int RX = M + PW + G, RW = 480 - M - RX;
static const int Y3 = Y2 + H2 + G, H3 = 272 - M - Y3;

struct Tile {
  lv_obj_t *box, *title, *val, *sub, *ico, *ring, *halo;
  int kind;
  uint32_t ic;
  int hot = -1;
};
static Tile T[5];
static lv_obj_t *scr_main = nullptr, *scr_cal = nullptr, *pin_layer = nullptr;
static lv_obj_t *l_clock, *pw_panel, *ch_canvas, *pk_glow;
static lv_obj_t *pills[2], *pill_lbl[2];
static lv_obj_t *ylab[3], *hstrip[24], *hlab[12];
static lv_obj_t *d_today_t, *d_month, *d_today, *rings_cv, *ring_pct[3], *ring_pctu[3], *ring_val[3], *today_val[3], *ring_halo[3];
static lv_obj_t *ev_panel, *ev_cv, *ev_last, *ev_cnt[4], *ev_date[4];
static lv_obj_t *cg_box, *cg_title, *cg_tog, *cg_knob, *cg_state, *cg_win, *cg_sched, *cg_bar, *cg_wbar, *cg_now, *cg_hl[5], *cg_l1, *cg_l2;
static const uint32_t LANE_C[4] = {PINK, AMBER, MAGENTA, PURPLE};
static const char *LANE_N[4] = {"ВОРОТА", "СВЕТ", "ОХРАНА", "ДВИЖЕНИЕ"};
static int CUR_PAGE = 0;  // 0 главный, 1 календарь
static uint32_t cal_open_ms = 0, pin_open_ms = 0;
static State S;
// TZ в libc платы не выставлен (localtime_r дает UTC), местное время считаем
// смещением из часов HA: местные поля ESPTime минус его же UTC-эпоха
static int TZ_OFF = 0;
static void loc_tm(time_t t, struct tm *r) {
  t += TZ_OFF;
  gmtime_r(&t, r);
}

static void show_page(int p);
static void pin_open();
static bool pin_is_open() { return pin_open_ms != 0; }

static void tile_style(Tile &t, bool hot, uint32_t g1, uint32_t g2, uint32_t gc, bool tap) {
  int key = hot ? (int) (g1 & 0xffff) + 2 : 0;
  if (t.hot == key) return;
  t.hot = key;
  if (hot) {
    fill(t.box, g1, 10, g2, true);
    border(t.box, 0xffd2af);
    glow(t.box, gc, 16, 200);
    set_color(t.title, 0xffebf5);
    set_color(t.sub, 0xffe6eb);
    icon(t.ico, t.kind, 0xfff0fa);
    if (t.ring) lv_obj_set_style_border_color(t.ring, C(0xfff0fa), 0);
    lv_obj_add_flag(t.halo, LV_OBJ_FLAG_HIDDEN);
  } else {
    fill(t.box, PANEL, 10, PANEL2);
    border(t.box, mix(EDGE, t.ic, tap ? 0.45f : 0.35f));
    glow(t.box, 0, 0);
    set_color(t.title, SUB);
    set_color(t.sub, SUB);
    icon(t.ico, t.kind, tap ? mix(t.ic, SUB, 0.4f) : t.ic);
    if (t.ring) lv_obj_set_style_border_color(t.ring, C(mix(t.ic, SUB, 0.3f)), 0);
    lv_obj_remove_flag(t.halo, LV_OBJ_FLAG_HIDDEN);
  }
}

static uint32_t alarm_press_ms = 0;
static void on_tile(lv_event_t *e) {
  int i = (int) (intptr_t) lv_event_get_user_data(e);
  if (lv_event_get_code(e) != LV_EVENT_SHORT_CLICKED) return;
  if (i == 1) {
    bool armed = S.alarm.rfind("armed", 0) == 0 || S.alarm == "triggered" || S.alarm == "pending";
    if (armed) pin_open();
    else if (cb_arm) cb_arm();
  } else if (i == 2) {
    if (cb_light) cb_light();
  }
}
static void on_charge(lv_event_t *e) {
  if (lv_event_get_code(e) == LV_EVENT_SHORT_CLICKED && cb_charge) cb_charge();
}
static void on_power(lv_event_t *e) {
  if (lv_event_get_code(e) == LV_EVENT_SHORT_CLICKED) show_page(1);
}

static void build_main(lv_obj_t *scr) {
  scr_main = scr;
  lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
  fill(scr, BG1, 0, BG2);
  // ---- верхние плитки
  const char *titles[5] = {"ВОРОТА", "ОХРАНА", "СВЕТ", "ДВИЖЕНИЕ", "МОЩНОСТЬ"};
  const uint32_t ics[5] = {PINK, MAGENTA, AMBER, PURPLE, ORANGE};
  for (int i = 0; i < 5; i++) {
    Tile &t = T[i];
    int x = M + i * (TW + G), y = M;
    t.kind = i;
    t.ic = ics[i];
    t.halo = obj(scr, x + TW - 20, y + 8, 12, 12);
    fill(t.halo, ics[i], LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(t.halo, 40, 0);
    glow(t.halo, ics[i], 16, 110);
    t.box = panel(scr, x, y, TW, TH, 10);
    lv_obj_move_background(t.halo);
    lv_obj_move_background(t.box);
    lv_obj_move_foreground(t.halo);
    t.title = lbl(t.box, 7, 6, FS, SUB, titles[i]);
    t.val = lbl(t.box, 7, 22, FM, TEXT, "");
    t.sub = lbl(t.box, 7, 41, FS, SUB, "");
    t.ico = canvas(t.box, TW - 24, 4, 20, 20);
    t.ring = nullptr;
    if (i == 1 || i == 2) {
      t.ring = obj(t.box, TW - 25, 2, 21, 21);
      lv_obj_set_style_radius(t.ring, LV_RADIUS_CIRCLE, 0);
      border(t.ring, SUB, 1);
      lv_obj_add_flag(t.box, LV_OBJ_FLAG_CLICKABLE);
      lv_obj_add_event_cb(t.box, on_tile, LV_EVENT_SHORT_CLICKED, (void *) (intptr_t) i);
    }
    t.hot = -1;
    tile_style(t, false, 0, 0, 0, t.ring != nullptr);
  }
  // ---- мощность за сутки (касание открывает календарь)
  pw_panel = panel(scr, M, Y2, PW, H2);
  lv_obj_add_flag(pw_panel, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(pw_panel, on_power, LV_EVENT_SHORT_CLICKED, nullptr);
  lbl(pw_panel, 8, 6, FS, SUB, "МОЩНОСТЬ ЗА 24 Ч");
  {
    int kx = 8 + tw("МОЩНОСТЬ ЗА 24 Ч", 6) + 14, ky = 11;
    lv_obj_t *r = obj(pw_panel, kx - 8, ky - 8, 17, 17);
    lv_obj_set_style_radius(r, LV_RADIUS_CIRCLE, 0);
    border(r, mix(VIOLET, SUB, 0.3f));
    lv_obj_t *k = canvas(pw_panel, kx - 6, ky - 5, 12, 11);
    Layer L(k);
    L.rect(1, 1, 9, 8, 1, 0, LV_OPA_TRANSP, SUB, 1);
    L.line(1, 3, 9, 3, SUB);
    L.circle(4, 6, 1, ORANGE);
  }
  l_clock = lbl(pw_panel, PW - 8, 3, FM, ORANGE, "--:--:--", 2, 80);
  pk_glow = obj(pw_panel, 0, 0, 10, 10);
  fill(pk_glow, ORANGE, LV_RADIUS_CIRCLE);
  lv_obj_set_style_bg_opa(pk_glow, 50, 0);
  glow(pk_glow, ORANGE, 30, 90);
  lv_obj_add_flag(pk_glow, LV_OBJ_FLAG_HIDDEN);
  ch_canvas = canvas(pw_panel, 34, 38 - 2, PW - 46, 52 + 3);
  for (int k = 0; k < 3; k++) ylab[k] = lbl(pw_panel, 34 - 5, 0, FS, DIM, "", 2, 40);
  for (int k = 0; k < 2; k++) {
    pills[k] = obj(pw_panel, 0, 0, 10, 11);
    fill(pills[k], 0xf5eeff, 3);
    pill_lbl[k] = lbl(pills[k], 4, 1, FS, 0x281046, "");
    lv_obj_add_flag(pills[k], LV_OBJ_FLAG_HIDDEN);
  }
  float cw = (PW - 46) / 24.0f;
  for (int k = 0; k < 24; k++) {
    hstrip[k] = obj(pw_panel, (int) (34 + k * cw + 0.5f), 38 + 52 + 4, (int) cw - 1, 6);
    fill(hstrip[k], 0x1e163e, 2);
  }
  for (int k = 0; k < 12; k++) hlab[k] = lbl(pw_panel, (int) (34 + (2 * k + 0.5f) * cw), 38 + 52 + 13, FS, SUB, "", 1, 16);
  // ---- доли линий
  lv_obj_t *dp = panel(scr, RX, Y2, RW, H2);
  lbl(dp, 8, 6, FS, SUB, "МЕСЯЦ");
  d_month = lbl(dp, 8 + 30 + 4, 3, FM, ORANGE, "--");
  d_today = lbl(dp, RW - 8, 3, FM, ORANGE, "--", 2, 48);
  d_today_t = lbl(dp, RW - 8 - 32 - 4, 6, FS, SUB, "СЕГОДНЯ", 2, 48);
  const char *ln[3] = {"СВЕТ", "РОЗЕТКИ", "ЗАРЯДКА"};
  const uint32_t lc[3] = {AMBER, MAGENTA, ORANGE};
  float step = (RW - 16) / 3.0f;
  for (int k = 0; k < 3; k++) {
    int cx = (int) (8 + step * (k + 0.5f));
    ring_halo[k] = obj(dp, cx - 12, 44 - 12, 24, 24);
    fill(ring_halo[k], lc[k], LV_RADIUS_CIRCLE);
    lv_obj_set_style_bg_opa(ring_halo[k], 30, 0);
    glow(ring_halo[k], lc[k], 22, 70);
  }
  rings_cv = canvas(dp, 0, 20, RW, 50);
  for (int k = 0; k < 3; k++) {
    int cx = (int) (8 + step * (k + 0.5f));
    ring_pct[k] = lbl(dp, cx, 44 - 8, FB, TEXT, "", 0);
    ring_pctu[k] = lbl(dp, cx, 44 - 5, FS, SUB, "%");
    lbl(dp, cx, 69, FS, SUB, ln[k], 1, 56);
    ring_val[k] = lbl(dp, cx, 78, FS, TEXT, "", 1, 56);
    today_val[k] = lbl(dp, cx, 105, FS, SUB, "", 1, 56);
  }
  lbl(dp, 8, 91, FS, SUB, "сегодня по линиям, кВт·ч");
  lv_obj_t *hl = obj(dp, 8 + tw("сегодня по линиям, кВт·ч", 6) + 6, 95, RW - 16 - tw("сегодня по линиям, кВт·ч", 6) - 6, 1);
  fill(hl, 0x322464, 0);
  // ---- события 72 ч
  ev_panel = panel(scr, M, Y3, PW, H3);
  lbl(ev_panel, 8, 4, FS, SUB, "СОБЫТИЯ ЗА 72 Ч");
  ev_last = lbl(ev_panel, PW - 8, 4, FS, SUB, "", 2, 170);
  for (int k = 0; k < 4; k++) {
    lbl(ev_panel, 8, 18 + k * 10, FS, SUB, LANE_N[k]);
    ev_cnt[k] = lbl(ev_panel, PW - 8, 18 + k * 10, FS, LANE_C[k], "", 2, 30);
  }
  ev_cv = canvas(ev_panel, 62, 15, PW - 96, 44);
  for (int k = 0; k < 4; k++) ev_date[k] = lbl(ev_panel, 0, H3 - 12, FS, SUB, "", 1, 40);
  // ---- зарядка (касание включает/выключает линию)
  cg_box = panel(scr, RX, Y3, RW, H3, 10, mix(EDGE, MAGENTA, 0.45f));
  lv_obj_add_flag(cg_box, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(cg_box, on_charge, LV_EVENT_SHORT_CLICKED, nullptr);
  cg_title = lbl(cg_box, 8, 4, FS, SUB, "ЗАРЯДКА");
  cg_tog = obj(cg_box, RW - 30, 4, 22, 10);
  fill(cg_tog, 0x221a42, 5);
  border(cg_tog, EDGE);
  cg_knob = obj(cg_tog, 1, 1, 8, 8);
  fill(cg_knob, SUB, LV_RADIUS_CIRCLE);
  cg_state = lbl(cg_box, RW - 35, 4, FS, ORANGE, "", 2, 60);
  cg_win = lbl(cg_box, 8, 14, FB, TEXT, "23:00 - 07:00");
  cg_sched = lbl(cg_box, 8 + 7 * 6 + 6, 4, FS, SUB, "");
  cg_bar = obj(cg_box, 8, 34, RW - 16, 4);
  fill(cg_bar, 0x221a42, 2);
  cg_wbar = obj(cg_box, 8, 34, 10, 4);
  fill(cg_wbar, 0xbe3cff, 2, 0xff8c28, true);
  glow(cg_wbar, MAGENTA, 8, 140);
  cg_now = canvas(cg_box, 0, 28, 8, 6);
  {
    Layer L(cg_now);
    L.tri(0, 0, 7, 0, 3.5f, 4, TEXT);
  }
  const int hrs[5] = {18, 22, 2, 6, 10};
  for (int k = 0; k < 5; k++) {
    int x = (int) (8 + (RW - 16) * (((hrs[k] - 18 + 24) % 24) / 16.0f));
    char b[4];
    snprintf(b, sizeof b, "%02d", hrs[k]);
    cg_hl[k] = lbl(cg_box, k == 0 ? x : (k == 4 ? x : x), 41, FS, SUB, b, k == 0 ? 0 : (k == 4 ? 2 : 1), 16);
  }
  cg_l1 = lbl(cg_box, 8, 51, FS, SUB, "");
  cg_l2 = lbl(cg_box, 8, 61, FS, SUB, "");
}

static std::string dm(uint32_t epoch) {
  if (!epoch) return "нет данных";
  time_t t = epoch;
  struct tm tmv;
  loc_tm(t, &tmv);
  char b[16];
  strftime(b, sizeof b, "%d.%m %H:%M", &tmv);
  return b;
}

static uint32_t chart_ver = 0xFFFFFFFF;
static void draw_chart() {
  const float gw = PW - 46, gh = 52;
  std::vector<float> v;
  for (float x : H_POWER) v.push_back(x * 1000.0f);  // Вт
  Layer L(ch_canvas);
  int n = v.size();
  static const int SC[5][3] = {{0, 100, 200}, {0, 250, 500}, {0, 500, 1000}, {0, 1000, 2000}, {0, 2500, 5000}};
  static const int VM[5] = {250, 500, 1000, 2500, 5000};
  float mx = 0;
  for (float x : v) mx = std::max(mx, x);
  int si = 4;
  for (int i = 0; i < 5; i++)
    if (mx * 1.15f <= VM[i]) { si = i; break; }
  float vmax = VM[si];
  auto yv = [&](float x) { return 2 + gh - x / vmax * gh; };
  for (int k = 0; k < 3; k++) {
    float yy = yv(SC[si][k]);
    L.line(0, yy, gw, yy, 0x281e50);
    char b[8];
    snprintf(b, sizeof b, "%d", SC[si][k]);
    set_text(ylab[k], b);
    lv_obj_set_y(ylab[k], (int) (36 + yy - 5));
  }
  if (n < 2) {
    for (int k = 0; k < 2; k++) lv_obj_add_flag(pills[k], LV_OBJ_FLAG_HIDDEN);
    return;
  }
  std::vector<lv_point_precise_t> pts;
  for (int i = 0; i < n; i++) pts.push_back({(lv_value_precise_t) ((i + 0.5f) * gw / n), (lv_value_precise_t) yv(v[i])});
  std::vector<lv_point_precise_t> area;
  area.push_back({0, (lv_value_precise_t) yv(v[0])});
  for (auto &p : pts) area.push_back(p);
  area.push_back({(lv_value_precise_t) gw, (lv_value_precise_t) yv(v[n - 1])});
  for (size_t i = 0; i + 1 < area.size(); i++) {
    L.tri(area[i].x, area[i].y, area[i + 1].x, area[i + 1].y, area[i].x, 2 + gh, 0x2e1046);
    L.tri(area[i + 1].x, area[i + 1].y, area[i + 1].x, 2 + gh, area[i].x, 2 + gh, 0x2e1046);
  }
  for (size_t i = 0; i + 1 < area.size(); i++) L.line(area[i].x, area[i].y, area[i + 1].x, area[i + 1].y, MAGENTA, 2);
  // метки: главный пик и второй не ближе 4 часов, если больше 20 Вт
  int ipk = 0;
  for (int i = 1; i < n; i++) if (v[i] > v[ipk]) ipk = i;
  int marks[2] = {ipk, -1};
  float best = 20;
  for (int i = 0; i < n; i++)
    if (std::abs(i - ipk) > 3 && v[i] > best) { best = v[i]; marks[1] = i; }
  for (int k = 0; k < 2; k++) {
    int i = marks[k];
    if (i < 0) { lv_obj_add_flag(pills[k], LV_OBJ_FLAG_HIDDEN); continue; }
    float x0 = pts[i].x, y0 = pts[i].y;
    L.line(x0, y0, x0, 2 + gh, 0xc896e6);
    L.circle(x0, y0, 3, TEXT, LV_OPA_COVER, MAGENTA, 1);
    char b[24];
    snprintf(b, sizeof b, "%.0f Вт %02d:00", v[i], (H_FIRST_HOUR + i) % 24);
    set_text(pill_lbl[k], b);
    int w = tw(b, 6) + 8;
    lv_obj_set_size(pills[k], w, 11);
    int px = (int) (34 + x0 - w / 2.0f), py = (int) (36 + y0 - 17);
    px = std::max(4, std::min(px, PW - 4 - w));
    lv_obj_set_pos(pills[k], px, py);
    lv_obj_remove_flag(pills[k], LV_OBJ_FLAG_HIDDEN);
    if (k == 0) {
      lv_obj_set_pos(pk_glow, (int) (34 + x0 - 5), (int) (36 + y0 - 2));
      lv_obj_remove_flag(pk_glow, LV_OBJ_FLAG_HIDDEN);
    }
  }
  // ленточка часов
  for (int k = 0; k < 24; k++) {
    bool act = k < n && v[k] > 30;
    if (act) fill(hstrip[k], 0xe628be, 2, 0xff783c, true);
    else fill(hstrip[k], 0x1e163e, 2);
  }
  for (int k = 0; k < 12; k++) {
    int i = 2 * k;
    int hr = (H_FIRST_HOUR + i) % 24;
    // подписи четных часов по фактическому часу
    int idx = (hr % 2 == 0) ? i : i + 1;
    if (idx >= 24) idx = 23;
    hr = (H_FIRST_HOUR + idx) % 24;
    char b[4];
    snprintf(b, sizeof b, "%02d", hr);
    set_text(hlab[k], b);
    lv_obj_set_x(hlab[k], (int) (34 + (idx + 0.5f) * gw / 24 - 8));
    set_color(hlab[k], idx < n && v[idx] > 30 ? ORANGE : SUB);
  }
}

static float last_shares[3] = {-1, -1, -1};
static void draw_rings(const float mv[3]) {
  float tot = mv[0] + mv[1] + mv[2];
  float sh[3];
  bool same = true;
  for (int k = 0; k < 3; k++) {
    sh[k] = tot > 0 ? mv[k] / tot * 100 : 0;
    if (std::abs(sh[k] - last_shares[k]) > 0.2f) same = false;
  }
  if (same) return;
  const uint32_t c1[3] = {AMBER, MAGENTA, ORANGE}, c2[3] = {ORANGE, PURPLE, MAGENTA};
  Layer L(rings_cv);
  float step = (RW - 16) / 3.0f;
  for (int k = 0; k < 3; k++) {
    last_shares[k] = sh[k];
    float cx = 8 + step * (k + 0.5f), cy = 24;
    L.circle(cx, cy, 14, 0x0c081e);
    L.arc(cx, cy, 21, 6, 0, 360, 0x241a48);
    if (sh[k] >= 0.5f) L.arc_grad(cx, cy, 21, 6, -90, -90 + 360 * sh[k] / 100, c1[k], c2[k]);
    char b[8];
    snprintf(b, sizeof b, "%d", (int) lroundf(sh[k]));
    set_text(ring_pct[k], b);
    int wn = tw(b, 8) + 1 + 6;
    lv_obj_set_x(ring_pct[k], (int) (cx - wn / 2.0f));
    lv_obj_set_x(ring_pctu[k], (int) (cx - wn / 2.0f + tw(b, 8) + 1));
  }
}

static uint32_t ev_drawn = 0xFFFFFFFF;
static void draw_events(uint32_t now_epoch, int tz_off) {
  if (!EV) return;
  ev_roll(now_epoch / 3600);
  if (ev_drawn == EV_VER) return;
  ev_drawn = EV_VER;
  const float lw = PW - 96, cw = lw / 72.0f;
  Layer L(ev_cv);
  // полосы-дорожки
  for (int k = 0; k < 4; k++) {
    float yy = 3 + k * 10 + 1;
    L.rect(0, yy, lw, 7, 3, mix(PANEL2, LANE_C[k], 0.12f), LV_OPA_COVER, mix(PANEL, LANE_C[k], 0.5f), 1);
  }
  // границы суток по местному времени
  int di = 0;
  int prev_i = 0;
  for (int i = 0; i < 72; i++) {
    uint32_t h = *EV_BASE - (71 - i);
    int local_h = (int) ((h + tz_off) % 24);
    if (local_h == 0 && i > 0) {
      float x = i * cw - 0.5f;
      L.line(x, 1, x, 41, 0x5a4696);
      if (di < 4 && (i - prev_i) >= 6) {
        time_t t = (time_t) (h - 1) * 3600;
        struct tm tmv;
        loc_tm(t, &tmv);
        char b[8];
        strftime(b, sizeof b, "%d.%m", &tmv);
        set_text(ev_date[di], b);
        lv_obj_set_x(ev_date[di], (int) (62 + (prev_i + i) / 2.0f * cw - 20));
        di++;
      }
      prev_i = i;
    }
  }
  if (di < 4 && 72 - prev_i >= 6) {
    time_t t = (time_t) (*EV_BASE) * 3600;
    struct tm tmv;
    loc_tm(t, &tmv);
    char b[8];
    strftime(b, sizeof b, "%d.%m", &tmv);
    set_text(ev_date[di], b);
    lv_obj_set_x(ev_date[di], (int) (62 + (prev_i + 72) / 2.0f * cw - 20));
    di++;
  }
  for (; di < 4; di++) set_text(ev_date[di], "");
  // события
  for (int k = 0; k < 4; k++) {
    int cnt = 0;
    float yy = 3 + k * 10 + 1;
    for (int i = 0; i < 72; i++) {
      int c = (*EV)[i * 4 + k];
      if (!c) continue;
      cnt += c;
      L.rect(i * cw - 0.5f, yy - 1, cw + 1.5f, 9, 1, LANE_C[k], 70);
      L.rect(i * cw + 0.2f, yy, cw - 0.4f, 7, 0, LANE_C[k], LV_OPA_COVER);
    }
    char b[8];
    snprintf(b, sizeof b, "%d", cnt);
    set_text(ev_cnt[k], b);
  }
}

// ------------------------------------------------------------------ календарь
static const int CM = 6, TB_Y = 6, TB_H = 24, CY = TB_Y + TB_H + 6, CH = 272 - CM - CY, CXP = CM, CW = 290;
static const int CRX = CXP + CW + 6, CRW = 480 - CM - CRX, CRH = 140;
static lv_obj_t *c_cd, *c_cost, *c_tab[12], *c_tabl[12], *c_title, *c_med, *c_cell[42], *c_day[42], *c_val[42];
static lv_obj_t *c_vmax, *c_rcv, *c_ctr1, *c_ctr2, *c_leg[4], *c_legu[4], *c_ltot, *c_lv[4], *c_lb[4];
static const int CCW = 36, CGX = 4, CGY = 3;
static const uint32_t STOPS[4] = {0x221450, VIOLET, MAGENTA, ORANGE};
static uint32_t heat(float t) {
  t = std::max(0.0f, std::min(1.0f, t));
  const float at[4] = {0, 0.35f, 0.7f, 1};
  for (int i = 0; i < 3; i++)
    if (t <= at[i + 1]) return mix(STOPS[i], STOPS[i + 1], (t - at[i]) / (at[i + 1] - at[i]));
  return STOPS[3];
}
static void on_cal(lv_event_t *e) {
  if (lv_event_get_code(e) == LV_EVENT_SHORT_CLICKED) show_page(0);
}

static void build_cal() {
  scr_cal = lv_obj_create(nullptr);
  lv_obj_remove_style_all(scr_cal);
  lv_obj_remove_flag(scr_cal, LV_OBJ_FLAG_SCROLLABLE);
  fill(scr_cal, BG1, 0, BG2);
  lv_obj_add_flag(scr_cal, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(scr_cal, on_cal, LV_EVENT_SHORT_CLICKED, nullptr);
  lv_obj_t *tb = panel(scr_cal, CM, TB_Y, 480 - 2 * CM, TB_H, 8, EDGE, 0x1a103e, 0x0e0924);
  lv_obj_t *bk = obj(tb, 4, 4 - 1, 84, 16);
  fill(bk, 0x7828be, 8, 0xc83296, true);
  border(bk, 0xffbee6);
  glow(bk, MAGENTA, 10, 150);
  lv_obj_t *ar = canvas(bk, 5, 3, 8, 10);
  {
    Layer L(ar);
    L.tri(1, 4, 7, 0, 7, 8, TEXT);
  }
  lbl(bk, 17, 3, FS, TEXT, "назад");
  c_cd = lbl(bk, 84 - 6, 3, FS, 0xffdcaa, "5:00", 2, 30);
  const char *MSH[12] = {"ЯНВ", "ФЕВ", "МАР", "АПР", "МАЙ", "ИЮН", "ИЮЛ", "АВГ", "СЕН", "ОКТ", "НОЯ", "ДЕК"};
  for (int i = 0; i < 12; i++) {
    c_tab[i] = obj(tb, 97 - CM + i * 25, 4, 24, 14);
    c_tabl[i] = lbl(c_tab[i], 12, 3, FS, 0x7a6eb2, MSH[i], 1, 24);
  }
  c_cost = lbl(tb, 480 - 2 * CM - 8, 3, FB, AMBER, "", 2, 90);
  // сетка
  lv_obj_t *cp = panel(scr_cal, CXP, CY, CW, CH, 10, 0x503296, 0x160e38, 0x0a071e);
  c_title = lbl(cp, 9, 6, FM, TEXT, "");
  c_med = lbl(cp, CW - 9, 9, FS, SUB, "", 2, 170);
  int gx0 = (CW - (7 * CCW + 6 * CGX)) / 2;
  const char *DOW[7] = {"Пн", "Вт", "Ср", "Чт", "Пт", "Сб", "Вс"};
  for (int i = 0; i < 7; i++) lbl(cp, gx0 + i * (CCW + CGX) + CCW / 2, 26, FS, i >= 5 ? PINK : SUB, DOW[i], 1, 20);
  for (int i = 0; i < 42; i++) {
    c_cell[i] = obj(cp, 0, 0, CCW, 30);
    c_day[i] = lbl(c_cell[i], 4, 2, FM, SUB, "");
    c_val[i] = lbl(c_cell[i], CCW - 4, 30 - 11, FS, AMBER, "", 2, 34);
  }
  // легенда
  int ly = CH - 17, lx = gx0;
  lbl(cp, lx, ly, FS, SUB, "0");
  int bx = lx + 9, bw = 64;
  const float at[4] = {0, 0.35f, 0.7f, 1};
  for (int i = 0; i < 3; i++) {
    lv_obj_t *s = obj(cp, bx + (int) (bw * at[i]), ly + 1, (int) (bw * (at[i + 1] - at[i])) + 1, 6);
    fill(s, STOPS[i], 0, STOPS[i + 1], true);
  }
  c_vmax = lbl(cp, bx + bw + 4, ly, FS, SUB, "");
  int x2 = bx + bw + 4 + 30 + 12;
  lv_obj_t *q = obj(cp, x2, ly, 8, 8);
  fill(q, 0x0e0a22, 2);
  border(q, 0x5c4c98);
  lbl(cp, x2 + 12, ly, FS, SUB, "до учета");
  int x3 = x2 + 12 + 48 + 10;
  q = obj(cp, x3, ly, 8, 8);
  fill(q, 0x180e2c, 2);
  border(q, 0x783c6e);
  lbl(cp, x3 + 12, ly, FS, SUB, "нет");
  int x4 = x3 + 12 + 18 + 10;
  q = obj(cp, x4, ly, 8, 8);
  border(q, CYAN);
  lv_obj_set_style_radius(q, 2, 0);
  lbl(cp, x4 + 12, ly, FS, SUB, "сегодня");
  // итоги месяца
  lv_obj_t *rp = panel(scr_cal, CRX, CY, CRW, CRH, 10, 0x503296, 0x180e3c, 0x0a071e);
  lbl(rp, 9, 6, FS, SUB, "ИТОГИ МЕСЯЦА");
  c_rcv = canvas(rp, 52 - 50, 80 - 50, 100, 100);
  c_ctr1 = lbl(rp, 52, 80 - 13, FB, 0xbea0ff, "", 1, 40);
  c_ctr2 = lbl(rp, 52, 80 + 3, FS, 0xbea0ff, "", 1, 40);
  const char *LN[4] = {"СЕГОДНЯ", "МЕСЯЦ", "ДНИ", "ПРОГНОЗ"};
  const uint32_t LC[4] = {ORANGE, MAGENTA, VIOLET, 0};
  for (int i = 0; i < 4; i++) {
    int y = 20 + i * 28, lgx = 102;
    if (LC[i]) {
      lv_obj_t *d = obj(rp, lgx, y + 1, 7, 7);
      fill(d, LC[i], LV_RADIUS_CIRCLE);
      lbl(rp, lgx + 9, y, FS, SUB, LN[i]);
    } else {
      lbl(rp, lgx, y, FS, AMBER, LN[i]);
    }
    c_leg[i] = lbl(rp, lgx, y + 10, FB, LC[i] ? TEXT : AMBER, "");
    c_legu[i] = lbl(rp, lgx, y + 17, FS, LC[i] ? SUB : AMBER, i == 2 ? "" : "кВт·ч");
  }
  // линии
  int LY = CY + CRH + 6, LH = CY + CH - LY;
  lv_obj_t *lp = panel(scr_cal, CRX, LY, CRW, LH, 10, 0x503296, 0x160e38, 0x0a071e);
  lbl(lp, 9, 5, FS, SUB, "ЛИНИИ ЗА МЕСЯЦ");
  c_ltot = lbl(lp, CRW - 9, 5, FS, AMBER, "", 2, 80);
  const char *RN[4] = {"Розетки", "Свет", "Вне линий", "Авто"};
  for (int i = 0; i < 4; i++) {
    int y = 18 + i * 16;
    lbl(lp, 9, y, FS, i == 2 ? SUB : TEXT, RN[i]);
    c_lv[i] = lbl(lp, CRW - 9, y, FS, TEXT, "", 2, 80);
    lv_obj_t *bg = obj(lp, 9, y + 10, CRW - 18, 3);
    fill(bg, 0x221848, 1);
    c_lb[i] = obj(lp, 9, y + 10, 2, 3);
  }
}

static void refresh_cal() {
  if (!S.time_ok) return;
  auto &t = S.now;
  int year = t.year, mon = t.month, today = t.day_of_month;
  int ndays = 30;
  {
    static const int dim[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    ndays = dim[mon - 1] + ((mon == 2 && (year % 4 == 0)) ? 1 : 0);
  }
  // энергия по числам текущего месяца из H_DAYS (последний это сегодня)
  std::vector<float> dval(32, -1);  // -1 нет в выгрузке
  int nd = H_DAYS.size();
  int first_data = 99;
  for (int i = 0; i < nd; i++) {
    int back = nd - 1 - i;
    time_t ts = t.timestamp - (time_t) back * 86400;
    struct tm tmv;
    loc_tm(ts, &tmv);
    if (tmv.tm_mon + 1 != mon) continue;
    dval[tmv.tm_mday] = H_DAYS[i];
  }
  // начало учета: самый ранний день выгрузки с расходом больше нуля
  for (int i = 0; i < nd; i++)
    if (H_DAYS[i] > 0) {
      time_t ts = t.timestamp - (time_t) (nd - 1 - i) * 86400;
      struct tm tmv;
      loc_tm(ts, &tmv);
      first_data = (tmv.tm_mon + 1 == mon) ? tmv.tm_mday : 0;
      break;
    }
  float month = (int) H_MONTHS.size() >= mon ? H_MONTHS[mon - 1] : 0;
  std::vector<float> have;
  float vmax = 0;
  int dmax = -1;
  for (int d = 1; d <= today; d++)
    if (dval[d] > 0) {
      have.push_back(dval[d]);
      if (dval[d] > vmax) { vmax = dval[d]; dmax = d; }
    }
  std::sort(have.begin(), have.end());
  float med = 0;
  if (!have.empty()) med = have.size() % 2 ? have[have.size() / 2] : (have[have.size() / 2 - 1] + have[have.size() / 2]) / 2;
  float avg = have.empty() ? 0 : month / have.size();
  float today_kwh = dval[today] > 0 ? dval[today] : 0;
  float forecast = month + med * (ndays - today);
  // шапка
  static const char *MON[12] = {"ЯНВАРЬ", "ФЕВРАЛЬ", "МАРТ", "АПРЕЛЬ", "МАЙ", "ИЮНЬ", "ИЮЛЬ", "АВГУСТ", "СЕНТЯБРЬ", "ОКТЯБРЬ", "НОЯБРЬ", "ДЕКАБРЬ"};
  for (int i = 0; i < 12; i++) {
    if (i + 1 == mon) {
      fill(c_tab[i], PURPLE, 3, MAGENTA, true);
      glow(c_tab[i], MAGENTA, 8, 150);
      lv_obj_set_style_border_width(c_tab[i], 0, 0);
      set_color(c_tabl[i], TEXT);
    } else {
      lv_obj_set_style_bg_opa(c_tab[i], LV_OPA_TRANSP, 0);
      lv_obj_set_style_radius(c_tab[i], 3, 0);
      border(c_tab[i], 0x3e2e7a);
      glow(c_tab[i], 0, 0);
      set_color(c_tabl[i], 0x7a6eb2);
    }
  }
  char b[48];
  snprintf(b, sizeof b, "≈%.0f грн", month * TARIFF);
  set_text(c_cost, b);
  snprintf(b, sizeof b, "%s %d", MON[mon - 1], year);
  set_text(c_title, b);
  snprintf(b, sizeof b, "МЕДИАНА %.1f кВт·ч/сут", med);
  set_text(c_med, b);
  // сетка
  // день недели первого числа от сегодняшнего (day_of_week: 1 это воскресенье)
  int dow_today = (t.day_of_week + 5) % 7;
  int off = ((dow_today - (today - 1)) % 7 + 7) % 7;
  int rows = (off + ndays + 6) / 7;
  int chh = rows > 5 ? 26 : 30;
  int gx0 = (CW - (7 * CCW + 6 * CGX)) / 2, gy0 = 26 + 11;
  float lvmax = std::log1p(std::max(vmax, 0.01f));
  for (int i = 0; i < 42; i++) {
    lv_obj_t *c = c_cell[i];
    int d = i - off + 1;
    if (d < 1 || d > ndays) { lv_obj_add_flag(c, LV_OBJ_FLAG_HIDDEN); continue; }
    lv_obj_remove_flag(c, LV_OBJ_FLAG_HIDDEN);
    int col = i % 7, row = i / 7;
    lv_obj_set_pos(c, gx0 + col * (CCW + CGX), gy0 + row * (chh + CGY));
    lv_obj_set_size(c, CCW, chh);
    lv_obj_set_y(c_val[i], chh - 11);
    snprintf(b, sizeof b, "%d", d);
    set_text(c_day[i], b);
    glow(c, 0, 0);
    float v = dval[d];
    if (d <= today && v > 0) {
      float tv = std::log1p(v) / lvmax;
      uint32_t hc = heat(tv);
      fill(c, hc, 5, mix(hc, 0, 0.45f));
      border(c, mix(hc, 0xffffff, 0.25f));
      set_color(c_day[i], TEXT);
      snprintf(b, sizeof b, "%.1f", v);
      set_text(c_val[i], b);
      set_color(c_val[i], tv > 0.55f ? 0xffffff : AMBER);
      if (d == dmax) glow(c, ORANGE, 12, 160);
    } else if (d < first_data) {
      fill(c, 0x0e0a22, 5);
      border(c, 0x221a46);
      set_color(c_day[i], 0x50468a);
      set_text(c_val[i], "");
    } else if (d <= today) {
      fill(c, 0x180e2c, 5);
      border(c, 0x783c6e);
      set_color(c_day[i], SUB);
      set_text(c_val[i], "нет");
      set_color(c_val[i], PINK);
    } else {
      lv_obj_set_style_bg_opa(c, LV_OPA_TRANSP, 0);
      lv_obj_set_style_radius(c, 5, 0);
      border(c, 0x463782);
      set_color(c_day[i], SUB);
      set_text(c_val[i], "");
    }
    if (d == today) {
      border(c, CYAN, 2);
      glow(c, CYAN, 10, 170);
    }
  }
  snprintf(b, sizeof b, "%.1f", vmax);
  set_text(c_vmax, b);
  // кольца итогов
  {
    Layer L(c_rcv);
    const float cx = 50, cy = 50;
    const int rr[3] = {44, 36, 28};
    const float fr[3] = {(float) today / ndays, forecast > 0 ? month / forecast : 0, avg > 0 ? today_kwh / avg : 0};
    const uint32_t a1[3] = {0x5032be, PURPLE, PINK}, a2[3] = {VIOLET, MAGENTA, ORANGE};
    for (int k = 0; k < 3; k++) {
      float f = std::max(0.0f, std::min(1.0f, fr[k]));
      L.arc(cx, cy, rr[k], 6, 0, 360, 0x24184c);
      if (f > 0.005f) L.arc_grad(cx, cy, rr[k], 6, -90, -90 + 360 * f, a1[k], a2[k], 36);
      float ang = (-90 + 360 * f) * (float) M_PI / 180;
      float ex = cx + (rr[k] - 3) * cosf(ang), ey = cy + (rr[k] - 3) * sinf(ang);
      L.circle(ex, ey, 4, a2[k], 90);
      L.circle(ex, ey, 2.5f, 0xffffff);
    }
  }
  snprintf(b, sizeof b, "%d", today);
  set_text(c_ctr1, b);
  snprintf(b, sizeof b, "из %d", ndays);
  set_text(c_ctr2, b);
  const char *vals[4];
  char v0[16], v1[16], v2[16], v3[16];
  snprintf(v0, sizeof v0, "%.1f", today_kwh);
  snprintf(v1, sizeof v1, "%.1f", month);
  snprintf(v2, sizeof v2, "%d/%d", today, ndays);
  snprintf(v3, sizeof v3, "%.1f", forecast);
  vals[0] = v0; vals[1] = v1; vals[2] = v2; vals[3] = v3;
  for (int i = 0; i < 4; i++) {
    set_text(c_leg[i], vals[i]);
    lv_obj_set_x(c_legu[i], 102 + tw(vals[i], 8) + 3);
  }
  // линии
  float ms = std::isnan(S.m_sock) ? 0 : S.m_sock, ml = std::isnan(S.m_light) ? 0 : S.m_light,
        mc = std::isnan(S.m_chg) ? 0 : S.m_chg;
  float outside = std::max(0.0f, month - ms - ml - mc);
  float tot = std::max(month, ms + ml + mc);
  snprintf(b, sizeof b, "%.1f кВт·ч", tot);
  set_text(c_ltot, b);
  const float lv[4] = {ms, ml, outside, mc};
  const uint32_t lc1[4] = {MAGENTA, VIOLET, 0x5a468c, PINK}, lc2[4] = {ORANGE, CYAN, 0x826eb4, ORANGE};
  for (int i = 0; i < 4; i++) {
    snprintf(b, sizeof b, "%.1f кВт·ч", lv[i]);
    set_text(c_lv[i], b);
    set_color(c_lv[i], (i == 2 || lv[i] <= 0) ? SUB : TEXT);
    int w = tot > 0 ? (int) ((CRW - 18) * lv[i] / tot) : 0;
    if (w < 3) lv_obj_add_flag(c_lb[i], LV_OBJ_FLAG_HIDDEN);
    else {
      lv_obj_remove_flag(c_lb[i], LV_OBJ_FLAG_HIDDEN);
      lv_obj_set_width(c_lb[i], w);
      fill(c_lb[i], lc1[i], 1, lc2[i], true);
    }
  }
}

// ------------------------------------------------------------------ пин-панель снятия охраны
static lv_obj_t *p_dots[4], *p_msg, *p_cd, *p_cdcv;
static std::string pin_buf;
static uint32_t pin_err_ms = 0;
static void pin_close() {
  if (pin_layer) lv_obj_add_flag(pin_layer, LV_OBJ_FLAG_HIDDEN);
  pin_open_ms = 0;
  pin_buf.clear();
}
static void pin_dots(uint32_t col) {
  for (int i = 0; i < 4; i++) {
    bool on = i < (int) pin_buf.size();
    fill(p_dots[i], on ? col : 0x221a48, LV_RADIUS_CIRCLE);
    border(p_dots[i], col);
    glow(p_dots[i], col, on ? 10 : 0, 160);
  }
}
static void on_key(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_SHORT_CLICKED) return;
  int k = (int) (intptr_t) lv_event_get_user_data(e);
  pin_open_ms = esphome::millis();  // любое нажатие продлевает минуту
  if (k == 10) { pin_close(); return; }                                   // отмена
  if (k == 11) { if (!pin_buf.empty()) pin_buf.pop_back(); pin_dots(MAGENTA); set_text(p_msg, ""); set_color(p_msg, SUB); return; }
  if (pin_buf.size() >= 4) return;
  pin_buf.push_back('0' + k);
  pin_dots(MAGENTA);
  if (pin_buf.size() == 4) {
    if (pin_buf == PIN) {
      set_text(p_msg, "код принят");
      set_color(p_msg, CYAN);
      if (cb_disarm) cb_disarm();
      pin_close();
    } else {
      set_text(p_msg, "неверный код");
      set_color(p_msg, RED);
      pin_dots(RED);
      pin_err_ms = esphome::millis();
    }
  }
}
static void on_pin_bg(lv_event_t *e) {}  // затемнение глотает касания мимо панели

static void build_pin(lv_obj_t *scr) {
  pin_layer = obj(scr, 0, 0, 480, 272);
  fill(pin_layer, 0x04020c, 0);
  lv_obj_set_style_bg_opa(pin_layer, 215, 0);
  lv_obj_add_flag(pin_layer, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(pin_layer, on_pin_bg, LV_EVENT_SHORT_CLICKED, nullptr);
  lv_obj_t *p = obj(pin_layer, 16, 14, 448, 244);
  fill(p, 0x1c1046, 14, 0x0c0822);
  border(p, 0xb45cff, 2);
  glow(p, PURPLE, 24, 170);
  // слева: заголовок, щит, точки кода, отсчет
  lbl(p, 14, 12, FM, TEXT, "СНЯТИЕ С ОХРАНЫ");
  lbl(p, 14, 30, FS, SUB, "гараж, шлюз 4");
  lv_obj_t *sh = canvas(p, 14, 48, 20, 20);
  icon(sh, 1, MAGENTA);
  lbl(p, 40, 53, FS, SUB, "введите пин-код");
  for (int i = 0; i < 4; i++) {
    p_dots[i] = obj(p, 18 + i * 30, 84, 16, 16);
    fill(p_dots[i], 0x221a48, LV_RADIUS_CIRCLE);
    border(p_dots[i], MAGENTA);
  }
  p_msg = lbl(p, 14, 110, FS, SUB, "");
  p_cdcv = canvas(p, 34, 130, 90, 90);
  p_cd = lbl(p, 34 + 45, 130 + 45 - 16, FM, TEXT, "60", 1, 40);
  lbl(p, 34 + 45, 130 + 45 + 2, FS, SUB, "сек", 1, 40);
  // справа: клавиатура 3x4
  const int kx0 = 160, ky0 = 12, kw = 86, kh = 50, kg = 6;
  const char *cap[12] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "ОТМЕНА", "0", "СТЕРЕТЬ"};
  const int val[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0, 11};
  for (int i = 0; i < 12; i++) {
    int x = kx0 + (i % 3) * (kw + kg), y = ky0 + (i / 3) * (kh + kg);
    lv_obj_t *k = obj(p, x, y, kw, kh);
    bool fn = val[i] >= 10;
    fill(k, fn ? 0x261442 : 0x2a1860, 10, fn ? 0x160c2c : 0x140c34);
    border(k, fn ? (val[i] == 10 ? 0x8c3c78 : 0x6e4ab4) : 0x8c5ae6);
    lv_obj_set_style_bg_color(k, C(0x6a28b4), LV_STATE_PRESSED);
    lv_obj_set_style_bg_grad_color(k, C(0xc83296), LV_STATE_PRESSED);
    lv_obj_set_style_shadow_color(k, C(MAGENTA), LV_STATE_PRESSED);
    lv_obj_set_style_shadow_width(k, 16, LV_STATE_PRESSED);
    lv_obj_set_style_shadow_opa(k, 200, LV_STATE_PRESSED);
    lv_obj_add_flag(k, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(k, on_key, LV_EVENT_SHORT_CLICKED, (void *) (intptr_t) val[i]);
    lbl(k, kw / 2, fn ? kh / 2 - 4 : kh / 2 - 8, fn ? FS : FB, fn ? (val[i] == 10 ? PINK : SUB) : TEXT, cap[i], 1, kw - 4);
  }
  lv_obj_add_flag(pin_layer, LV_OBJ_FLAG_HIDDEN);
}
static void pin_draw_cd(int left) {
  Layer L(p_cdcv);
  L.arc(45, 45, 40, 6, 0, 360, 0x24184c);
  if (left > 0) L.arc_grad(45, 45, 40, 6, -90, -90 + 360.0f * left / 60, PURPLE, ORANGE, 40);
  char b[4];
  snprintf(b, sizeof b, "%d", left);
  set_text(p_cd, b);
}
static void pin_open() {
  if (!pin_layer) return;
  pin_buf.clear();
  pin_dots(MAGENTA);
  set_text(p_msg, "");
  set_color(p_msg, SUB);
  pin_open_ms = esphome::millis();
  pin_draw_cd(60);
  lv_obj_remove_flag(pin_layer, LV_OBJ_FLAG_HIDDEN);
  lv_obj_move_foreground(pin_layer);
}

// ------------------------------------------------------------------ навигация, обновление
static void show_page(int p) {
  if (p == 1) {
    refresh_cal();
    cal_open_ms = esphome::millis();
    CUR_PAGE = 1;
    lv_screen_load(scr_cal);
  } else {
    CUR_PAGE = 0;
    lv_screen_load(scr_main);
  }
}

static void init(lv_obj_t *main_scr, const lv_font_t *fs, const lv_font_t *fm, const lv_font_t *fb) {
  FS = fs; FM = fm; FB = fb;
  build_main(main_scr);
  build_cal();
  build_pin(main_scr);
}

// раз в секунду: часы, отсчеты
static int last_sec = -1;
static void tick(const esphome::ESPTime &t) {
  if (t.is_valid() && t.second != last_sec) {
    last_sec = t.second;
    char b[12];
    snprintf(b, sizeof b, "%02d:%02d:%02d", t.hour, t.minute, t.second);
    set_text(l_clock, b);
  }
  uint32_t now = esphome::millis();
  if (CUR_PAGE == 1) {
    int left = 300 - (int) ((now - cal_open_ms) / 1000);
    if (left <= 0) show_page(0);
    else {
      char b[8];
      snprintf(b, sizeof b, "%d:%02d", left / 60, left % 60);
      set_text(c_cd, b);
    }
  }
  if (pin_open_ms) {
    int left = 60 - (int) ((now - pin_open_ms) / 1000);
    if (left <= 0) pin_close();
    else pin_draw_cd(left);
    if (pin_err_ms && now - pin_err_ms > 1200) {
      pin_err_ms = 0;
      pin_buf.clear();
      pin_dots(MAGENTA);
      set_text(p_msg, "");
      set_color(p_msg, SUB);
    }
  }
}

static void refresh(const State &st) {
  S = st;
  if (st.time_ok) {
    int d = st.now.hour * 3600 + st.now.minute * 60 + st.now.second - (int) (st.now.timestamp % 86400);
    if (d > 43200) d -= 86400;
    if (d < -43200) d += 86400;
    TZ_OFF = (int) lroundf(d / 900.0f) * 900;
  }
  char b[48];
  // плитки
  set_text(T[0].val, st.gate_open ? "ОТКРЫТЫ" : "ЗАКРЫТЫ");
  set_text(T[0].sub, dm(last_of(0)).c_str());
  bool armed = st.alarm.rfind("armed", 0) == 0, trig = st.alarm == "triggered";
  const char *at = armed ? "ВКЛЮЧЕНА" : trig ? "ТРЕВОГА" : st.alarm == "disarmed" ? "СНЯТА" :
                   st.alarm == "arming" ? "ВЗВОД" : st.alarm == "pending" ? "ОЖИДАНИЕ" : "НЕТ СВЯЗИ";
  set_text(T[1].val, at);
  set_text(T[1].sub, dm(last_of(2)).c_str());
  tile_style(T[1], armed || trig, trig ? 0xff2d55 : 0xe628be, trig ? 0xff7a1a : 0xff8232, MAGENTA, true);
  set_text(T[2].val, st.light_on ? "ВКЛ" : "ВЫКЛ");
  snprintf(b, sizeof b, "таймер %d мин", st.light_timeout);
  set_text(T[2].sub, b);
  tile_style(T[2], st.light_on, 0xe15f14, 0xffb92d, ORANGE, true);
  set_text(T[3].val, st.motion ? "ЕСТЬ" : "НЕТ");
  set_text(T[3].sub, dm(last_of(3)).c_str());
  if (std::isnan(st.p_now)) set_text(T[4].val, "--");
  else if (st.p_now >= 1000) { snprintf(b, sizeof b, "%.2f кВт", st.p_now / 1000); set_text(T[4].val, b); }
  else { snprintf(b, sizeof b, "%.0f Вт", st.p_now); set_text(T[4].val, b); }
  if (std::isnan(st.volt)) set_text(T[4].sub, "-- В");
  else { snprintf(b, sizeof b, "%.1f В", st.volt); set_text(T[4].sub, b); }
  // график
  if (chart_ver != H_VER) { chart_ver = H_VER; draw_chart(); }
  // доли линий
  float mv[3] = {std::isnan(st.m_light) ? 0 : st.m_light, std::isnan(st.m_sock) ? 0 : st.m_sock, std::isnan(st.m_chg) ? 0 : st.m_chg};
  float dv[3] = {st.d_light, st.d_sock, st.d_chg};
  const uint32_t lc[3] = {AMBER, MAGENTA, ORANGE};
  snprintf(b, sizeof b, "%.1f", mv[0] + mv[1] + mv[2]);
  set_text(d_month, b);
  float today = H_DAYS.empty() ? NAN : H_DAYS.back();
  if (std::isnan(today)) set_text(d_today, "--");
  else { snprintf(b, sizeof b, "%.2f", today); set_text(d_today, b); }
  lv_obj_set_x(d_today_t, RW - 8 - tw(lv_label_get_text(d_today), 8) - 4 - 48);
  draw_rings(mv);
  for (int k = 0; k < 3; k++) {
    snprintf(b, sizeof b, "%.1f", mv[k]);
    set_text(ring_val[k], b);
    if (std::isnan(dv[k])) set_text(today_val[k], "--");
    else { snprintf(b, sizeof b, "%.2f", dv[k]); set_text(today_val[k], b); }
    set_color(today_val[k], !std::isnan(dv[k]) && dv[k] >= 0.05f ? lc[k] : SUB);
  }
  // события
  if (st.time_ok) {
    int loc = st.now.hour * 3600 + st.now.minute * 60 + st.now.second;
    int utc = (int) (st.now.timestamp % 86400);
    int d = loc - utc;
    if (d > 43200) d -= 86400;
    if (d < -43200) d += 86400;
    TZ_OFF = (int) lroundf(d / 900.0f) * 900;
    int tz_h = (((TZ_OFF / 3600) % 24) + 24) % 24;
    draw_events((uint32_t) st.now.timestamp, tz_h);
  }
  uint32_t last_ev = 0;
  for (int k = 0; k < 4; k++) last_ev = std::max(last_ev, last_of(k));
  std::string le = "последнее: " + dm(last_ev);
  set_text(ev_last, le.c_str());
  // зарядка
  bool on = st.charge_on;
  static int cg_hot = -1;
  if (cg_hot != (int) on) {
  cg_hot = on;
  if (on) {
    fill(cg_box, 0x781eaa, 10, 0xa03246);
    border(cg_box, 0xffbeaa);
    glow(cg_box, MAGENTA, 16, 190);
    set_color(cg_title, 0xffe6f0);
    fill(cg_tog, 0xff963c, 5);
    border(cg_tog, 0xffe6c8);
    lv_obj_set_x(cg_knob, 13);
    fill(cg_knob, TEXT, LV_RADIUS_CIRCLE);
    set_color(cg_state, 0xffebc8);
  } else {
    fill(cg_box, PANEL, 10, PANEL2);
    border(cg_box, mix(EDGE, MAGENTA, 0.45f));
    glow(cg_box, 0, 0);
    set_color(cg_title, SUB);
    fill(cg_tog, 0x221a42, 5);
    border(cg_tog, EDGE);
    lv_obj_set_x(cg_knob, 1);
    fill(cg_knob, SUB, LV_RADIUS_CIRCLE);
    set_color(cg_state, ORANGE);
  }
  }
  set_text(cg_state, on ? "вкл" : "выкл");
  snprintf(b, sizeof b, "%s - %s", st.sched_start.c_str(), st.sched_end.c_str());
  set_text(cg_win, b);
  set_text(cg_sched, st.sched_on ? "расп. вкл" : "расп. выкл");
  set_color(cg_sched, on ? 0xffe1eb : SUB);
  int sh = atoi(st.sched_start.c_str()), sm = 0, eh = atoi(st.sched_end.c_str()), em = 0;
  sscanf(st.sched_start.c_str(), "%d:%d", &sh, &sm);
  sscanf(st.sched_end.c_str(), "%d:%d", &eh, &em);
  auto hx = [&](float h) { return 8 + (RW - 16) * (fmodf(h - 18 + 48, 24) / 16.0f); };
  float x0 = hx(sh + sm / 60.0f), x1 = hx(eh + em / 60.0f);
  if (x1 < x0) std::swap(x0, x1);
  lv_obj_set_pos(cg_wbar, (int) x0, 34);
  lv_obj_set_width(cg_wbar, std::max(2, (int) (x1 - x0)));
  if (st.time_ok) {
    float hn = st.now.hour + st.now.minute / 60.0f;
    if (fmodf(hn - 18 + 24, 24) <= 16) {
      lv_obj_remove_flag(cg_now, LV_OBJ_FLAG_HIDDEN);
      lv_obj_set_x(cg_now, (int) (hx(hn) - 3.5f));
    } else {
      lv_obj_add_flag(cg_now, LV_OBJ_FLAG_HIDDEN);
    }
  }
  uint32_t tone = on ? 0xffe1eb : SUB;
  if (!std::isnan(st.p_chg) && st.p_chg > 300) {
    snprintf(b, sizeof b, st.p_chg >= 1000 ? "заряжает %.2f кВт" : "заряжает %.0f Вт", st.p_chg >= 1000 ? st.p_chg / 1000 : st.p_chg);
    set_text(cg_l1, b);
    set_color(cg_l1, on ? 0xffffff : ORANGE);
  } else {
    set_text(cg_l1, on ? "линия включена, не заряжает" : "линия выключена");
    set_color(cg_l1, tone);
  }
  snprintf(b, sizeof b, "день %.1f, месяц %.1f кВт·ч", std::isnan(st.d_chg) ? 0 : st.d_chg, std::isnan(st.m_chg) ? 0 : st.m_chg);
  set_text(cg_l2, b);
  set_color(cg_l2, tone);
  if (CUR_PAGE == 1) refresh_cal();
}

}  // namespace gs
