// Скриншот экрана LVGL по HTTP: GET http://<плата>:8080/screen.bmp
//
// lv_snapshot в сборке ESPHome выключен (LV_USE_SNAPSHOT не попадает в
// lv_conf.h), поэтому кадр собирается иначе: функция вывода LVGL подменяется
// оберткой, которая копирует каждый отрисованный кусок в теневой буфер в PSRAM
// и передает его дальше штатному LvglComponent::static_flush_cb. Координаты и
// пиксели в этот момент логические, до поворота на 180, то есть как видит глаз.
//
// Запрос приходит в задачу HTTP-сервера, а LVGL живет в основном цикле и
// потокобезопасным не является. Поэтому обработчик только ставит флаг и ждет,
// а перерисовку экрана целиком (lv_refr_now) делает loop() из основного цикла.
#pragma once
#include "esphome.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "lvgl.h"

namespace dash_snap {

static const int W = 480, H = 272;
static uint16_t *fb = nullptr;           // RGB565, байты переставлены (LV_COLOR_16_SWAP)
static volatile bool req = false, ready = false;

static void flush_wrap(lv_display_t *d, const lv_area_t *a, uint8_t *px) {
  if (fb != nullptr) {
    int w = a->x2 - a->x1 + 1;
    int x0 = a->x1 < 0 ? 0 : a->x1;
    int x1 = a->x2 >= W ? W - 1 : a->x2;
    for (int y = a->y1; y <= a->y2; y++) {
      if (y < 0 || y >= H || x1 < x0) continue;
      memcpy(fb + y * W + x0, px + ((y - a->y1) * w + (x0 - a->x1)) * 2, (x1 - x0 + 1) * 2);
    }
  }
  esphome::lvgl::LvglComponent::static_flush_cb(d, a, px);
}

static esp_err_t handler(httpd_req_t *r) {
  ready = false;
  req = true;
  for (int i = 0; i < 150 && !ready; i++) vTaskDelay(pdMS_TO_TICKS(20));
  if (!ready) {
    httpd_resp_send_500(r);
    return ESP_FAIL;
  }
  httpd_resp_set_type(r, "image/bmp");
  // BMP 16 бит с масками 565, строки снизу вверх
  const uint32_t row = W * 2, img = row * H, off = 14 + 40 + 12, size = off + img;
  uint8_t h[66] = {0};
  auto u32 = [&](int o, uint32_t v) { h[o] = v; h[o + 1] = v >> 8; h[o + 2] = v >> 16; h[o + 3] = v >> 24; };
  h[0] = 'B'; h[1] = 'M';
  u32(2, size); u32(10, off);
  u32(14, 40); u32(18, W); u32(22, H);
  h[26] = 1; h[28] = 16;
  u32(30, 3);  // BI_BITFIELDS
  u32(34, img);
  u32(54, 0xF800); u32(58, 0x07E0); u32(62, 0x001F);
  httpd_resp_send_chunk(r, (const char *) h, sizeof h);
  static uint8_t line[W * 2];
  for (int y = H - 1; y >= 0; y--) {
    const uint8_t *src = (const uint8_t *) (fb + y * W);
    for (int x = 0; x < W; x++) {  // обратно в little-endian
      line[x * 2] = src[x * 2 + 1];
      line[x * 2 + 1] = src[x * 2];
    }
    if (httpd_resp_send_chunk(r, (const char *) line, sizeof line) != ESP_OK) return ESP_FAIL;
  }
  httpd_resp_send_chunk(r, nullptr, 0);
  return ESP_OK;
}

static void start(lv_display_t *disp) {
  fb = (uint16_t *) heap_caps_calloc(W * H, 2, MALLOC_CAP_SPIRAM);
  if (fb == nullptr) {
    ESP_LOGE("snap", "no PSRAM for screenshot buffer");
    return;
  }
  lv_display_set_flush_cb(disp, flush_wrap);
  httpd_config_t c = HTTPD_DEFAULT_CONFIG();
  c.server_port = 8080;
  c.ctrl_port = 32790;
  c.stack_size = 6144;
  httpd_handle_t s = nullptr;
  if (httpd_start(&s, &c) != ESP_OK) {
    ESP_LOGE("snap", "httpd_start failed");
    return;
  }
  httpd_uri_t u = {};
  u.uri = "/screen.bmp";
  u.method = HTTP_GET;
  u.handler = handler;
  httpd_register_uri_handler(s, &u);
  ESP_LOGI("snap", "screenshot at :8080/screen.bmp");
}

// Вызывать из основного цикла (interval), где живет LVGL
static void loop(lv_display_t *disp) {
  if (!req || fb == nullptr) return;
  req = false;
  lv_obj_invalidate(lv_screen_active());
  lv_refr_now(disp);
  ready = true;
}

}  // namespace dash_snap
