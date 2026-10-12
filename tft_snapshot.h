// Скриншот экрана mipi_spi по HTTP: GET http://<плата>:8080/screen.bmp
//
// Родственник dash_snapshot.h (там LVGL), только проще. У mipi_spi с буфером на
// весь экран (FRACTION 1) кадр целиком лежит в buffer_, а при аппаратном повороте
// (HAS_HARDWARE_ROTATION, как у модели T-DISPLAY-S3) еще и в логических
// координатах, как видит глаз. Значит, достаточно отдать этот буфер картинкой.
// buffer_ защищенный, достаем его через наследника-отмычку Peek.
//
// Читаем буфер из задачи HTTP-сервера без блокировки: в худшем случае попадет
// полукадр, для снимка это не страшно, а основной цикл не тормозится.
#pragma once
#include "esphome.h"
#include "esp_http_server.h"

namespace tft_snap {

template<typename T> struct Peek : public T {
  static uint16_t *buf(T *t) { return reinterpret_cast<uint16_t *>(static_cast<Peek<T> *>(t)->buffer_); }
};

static uint16_t *fb = nullptr;  // RGB565, байты в порядке шины (старший первым)
static int W = 0, H = 0;

static esp_err_t handler(httpd_req_t *r) {
  if (fb == nullptr) {
    httpd_resp_send_500(r);
    return ESP_FAIL;
  }
  httpd_resp_set_type(r, "image/bmp");
  // BMP 16 бит с масками 565, строки снизу вверх, ширина строки кратна 4 байтам
  const uint32_t row = (W * 2 + 3) & ~3u, img = row * H, off = 14 + 40 + 12, size = off + img;
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
  static uint8_t line[1024];
  for (int y = H - 1; y >= 0; y--) {
    const uint8_t *src = (const uint8_t *) (fb + y * W);
    for (int x = 0; x < W; x++) {  // в little-endian, как ждет BMP
      line[x * 2] = src[x * 2 + 1];
      line[x * 2 + 1] = src[x * 2];
    }
    for (uint32_t x = W * 2; x < row; x++) line[x] = 0;
    if (httpd_resp_send_chunk(r, (const char *) line, row) != ESP_OK) return ESP_FAIL;
  }
  httpd_resp_send_chunk(r, nullptr, 0);
  return ESP_OK;
}

// Звать после setup дисплея (on_boot с приоритетом ниже 600)
template<typename T> static void start(T *disp) {
  fb = Peek<T>::buf(disp);
  W = disp->get_width();
  H = disp->get_height();
  if (fb == nullptr || W * 2 > (int) sizeof(uint8_t[1024])) {
    ESP_LOGE("snap", "no display buffer");
    fb = nullptr;
    return;
  }
  httpd_config_t c = HTTPD_DEFAULT_CONFIG();
  c.server_port = 8080;
  c.ctrl_port = 32790;
  c.stack_size = 4096;
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
  ESP_LOGI("snap", "screenshot %dx%d at :8080/screen.bmp", W, H);
}

}  // namespace tft_snap
