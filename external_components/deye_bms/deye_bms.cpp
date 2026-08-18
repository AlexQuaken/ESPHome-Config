#include "deye_bms.h"

#ifdef USE_ESP32

#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cctype>

#include "mbedtls/sha256.h"
#include <esp_random.h>

namespace esphome {
namespace deye_bms {

static const char *const TAG = "deye_bms";

// ---------- компактный AES-128 (public-domain tiny-AES, ECB-ядро) ----------
namespace {
const uint8_t SBOX[256] = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16};
uint8_t RSBOX[256];
const uint8_t RCON[11] = {0x8d,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1b,0x36};

inline uint8_t xtime(uint8_t x) { return (uint8_t)((x << 1) ^ ((x >> 7) * 0x1b)); }
inline uint8_t mul(uint8_t x, uint8_t y) {
  uint8_t r = 0;
  for (int i = 0; i < 8; i++) {
    if (y & 1) r ^= x;
    uint8_t hi = x & 0x80;
    x <<= 1;
    if (hi) x ^= 0x1b;
    y >>= 1;
  }
  return r;
}

struct Aes128 {
  uint8_t rk[176];
  void init(const uint8_t *key) {
    static bool rsbox_done = false;
    if (!rsbox_done) {
      for (int i = 0; i < 256; i++) RSBOX[SBOX[i]] = (uint8_t) i;
      rsbox_done = true;
    }
    memcpy(rk, key, 16);
    for (int i = 4; i < 44; i++) {
      uint8_t t[4];
      memcpy(t, rk + (i - 1) * 4, 4);
      if (i % 4 == 0) {
        uint8_t tmp = t[0];
        t[0] = SBOX[t[1]] ^ RCON[i / 4];
        t[1] = SBOX[t[2]];
        t[2] = SBOX[t[3]];
        t[3] = SBOX[tmp];
      }
      for (int j = 0; j < 4; j++) rk[i * 4 + j] = rk[(i - 4) * 4 + j] ^ t[j];
    }
  }
  void encrypt_block(uint8_t *s) {
    auto addrk = [&](int r) { for (int i = 0; i < 16; i++) s[i] ^= rk[r * 16 + i]; };
    addrk(0);
    for (int round = 1; round <= 10; round++) {
      for (int i = 0; i < 16; i++) s[i] = SBOX[s[i]];
      uint8_t t;
      t = s[1]; s[1] = s[5]; s[5] = s[9]; s[9] = s[13]; s[13] = t;
      t = s[2]; s[2] = s[10]; s[10] = t; t = s[6]; s[6] = s[14]; s[14] = t;
      t = s[15]; s[15] = s[11]; s[11] = s[7]; s[7] = s[3]; s[3] = t;
      if (round != 10) {
        for (int c = 0; c < 4; c++) {
          uint8_t *col = s + c * 4;
          uint8_t a0 = col[0], a1 = col[1], a2 = col[2], a3 = col[3];
          col[0] = (uint8_t)(xtime(a0) ^ (xtime(a1) ^ a1) ^ a2 ^ a3);
          col[1] = (uint8_t)(a0 ^ xtime(a1) ^ (xtime(a2) ^ a2) ^ a3);
          col[2] = (uint8_t)(a0 ^ a1 ^ xtime(a2) ^ (xtime(a3) ^ a3));
          col[3] = (uint8_t)((xtime(a0) ^ a0) ^ a1 ^ a2 ^ xtime(a3));
        }
      }
      addrk(round);
    }
  }
  void decrypt_block(uint8_t *s) {
    auto addrk = [&](int r) { for (int i = 0; i < 16; i++) s[i] ^= rk[r * 16 + i]; };
    addrk(10);
    for (int round = 9; round >= 0; round--) {
      uint8_t t;
      t = s[13]; s[13] = s[9]; s[9] = s[5]; s[5] = s[1]; s[1] = t;
      t = s[2]; s[2] = s[10]; s[10] = t; t = s[6]; s[6] = s[14]; s[14] = t;
      t = s[3]; s[3] = s[7]; s[7] = s[11]; s[11] = s[15]; s[15] = t;
      for (int i = 0; i < 16; i++) s[i] = RSBOX[s[i]];
      addrk(round);
      if (round != 0) {
        for (int c = 0; c < 4; c++) {
          uint8_t *col = s + c * 4;
          uint8_t a0 = col[0], a1 = col[1], a2 = col[2], a3 = col[3];
          col[0] = (uint8_t)(mul(a0,14) ^ mul(a1,11) ^ mul(a2,13) ^ mul(a3,9));
          col[1] = (uint8_t)(mul(a0,9) ^ mul(a1,14) ^ mul(a2,11) ^ mul(a3,13));
          col[2] = (uint8_t)(mul(a0,13) ^ mul(a1,9) ^ mul(a2,14) ^ mul(a3,11));
          col[3] = (uint8_t)(mul(a0,11) ^ mul(a1,13) ^ mul(a2,9) ^ mul(a3,14));
        }
      }
    }
  }
};
}  // namespace

// Константные запросы: приложение шифрует их как ASCII-СТРОКУ hex-кода команды
// (напр. "aa5501040003700d0a" = 18 ASCII-байт), а НЕ как сырые байты. Проверено
// реверсом: func 04 = 16 напряжений ячеек, func a0 = телеметрия. Ответы приходят
// сырыми байтами (AA55...0D0A).
static const char REQ_CELLS[] = "aa5501040003700d0a";
static const char REQ_MAIN[] = "aa5501a00079b00d0a";
static const char REQ_INFO[] = "aa5501010000200d0a";  // func 01: паспорт + число циклов

static const uint8_t IV[16] = {'0', '0', '0', '0', '0', '0', '0', '0',
                               '0', '0', '0', '0', '0', '0', '0', '0'};

// ---------- CRC16 Modbus ----------
uint16_t DeyeBMS::crc16_(const uint8_t *d, size_t len) {
  uint16_t c = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    c ^= d[i];
    for (int b = 0; b < 8; b++) c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
  }
  return c;
}

// ---------- AES-128-CBC ----------
void DeyeBMS::aes_encrypt_(const uint8_t *in, size_t len, std::vector<uint8_t> &out) {
  // PKCS7
  size_t pad = 16 - (len % 16);
  std::vector<uint8_t> buf(in, in + len);
  buf.insert(buf.end(), pad, (uint8_t) pad);
  out.assign(buf.size(), 0);
  uint8_t iv[16];
  memcpy(iv, IV, 16);
  Aes128 aes;
  aes.init(this->key_);
  for (size_t off = 0; off < buf.size(); off += 16) {
    uint8_t blk[16];
    for (int i = 0; i < 16; i++) blk[i] = buf[off + i] ^ iv[i];  // CBC
    aes.encrypt_block(blk);
    memcpy(out.data() + off, blk, 16);
    memcpy(iv, blk, 16);
  }
}

bool DeyeBMS::aes_decrypt_(const uint8_t *in, size_t len, std::vector<uint8_t> &out) {
  if (len == 0 || len % 16 != 0) return false;
  out.assign(len, 0);
  uint8_t iv[16];
  memcpy(iv, IV, 16);
  Aes128 aes;
  aes.init(this->key_);
  for (size_t off = 0; off < len; off += 16) {
    uint8_t blk[16], cph[16];
    memcpy(cph, in + off, 16);
    memcpy(blk, cph, 16);
    aes.decrypt_block(blk);
    for (int i = 0; i < 16; i++) out[off + i] = blk[i] ^ iv[i];  // CBC
    memcpy(iv, cph, 16);
  }
  // снять PKCS7
  uint8_t pad = out.back();
  if (pad >= 1 && pad <= 16 && pad <= out.size()) out.resize(out.size() - pad);
  return true;
}

// ---------- ключ ----------
void DeyeBMS::derive_key_() {
  std::string material = this->ble_name_ + this->random_;
  uint8_t digest[32];
  mbedtls_sha256((const uint8_t *) material.c_str(), material.size(), digest, 0);
  // ключ = первые 16 символов hex-представления SHA256 (как ASCII-байты)
  char hex[65];
  for (int i = 0; i < 32; i++) sprintf(hex + i * 2, "%02x", digest[i]);
  memcpy(this->key_, hex, 16);
  this->have_key_ = true;
  ESP_LOGD(TAG, "ключ сессии готов: %.16s (name=%s random=%s)", hex, this->ble_name_.c_str(),
           this->random_.c_str());
}

// ---------- жизненный цикл ----------
void DeyeBMS::setup() { ESP_LOGCONFIG(TAG, "Deye BMS настроен, имя=%s", this->ble_name_.c_str()); }

void DeyeBMS::dump_config() {
  ESP_LOGCONFIG(TAG, "Deye BMS:");
  ESP_LOGCONFIG(TAG, "  BLE name: %s", this->ble_name_.c_str());
}

void DeyeBMS::loop() {}

void DeyeBMS::update() {
  // опрос по таймеру: если сессия готова — шлём следующую команду
  ESP_LOGV(TAG, "update: state=%d have_key=%d handle=0x%04x", this->state_, this->have_key_,
           this->char_handle_);
  if (this->state_ == READY && this->have_key_) {
    this->send_next_poll_();
  }
}

void DeyeBMS::reset_session_() {
  this->state_ = IDLE;
  this->have_key_ = false;
  this->notify_registered_ = false;
  this->char_handle_ = 0;
  this->rx_buf_.clear();
  this->poll_idx_ = 0;
}

// ---------- GATT ----------
void DeyeBMS::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                  esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT: {
      if (param->open.status == ESP_GATT_OK) ESP_LOGI(TAG, "BLE открыт");
      break;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGW(TAG, "BLE отключён");
      this->reset_session_();
      break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      auto *chr = this->parent()->get_characteristic(espbt::ESPBTUUID::from_uint16(DEYE_SERVICE_UUID),
                                                     espbt::ESPBTUUID::from_uint16(DEYE_CHAR_UUID));
      if (chr == nullptr) {
        ESP_LOGE(TAG, "характеристика FFF2 не найдена");
        break;
      }
      this->char_handle_ = chr->handle;
      ESP_LOGI(TAG, "FFF2 handle=0x%04x, регистрирую notify", this->char_handle_);
      esp_ble_gattc_register_for_notify(gattc_if, this->parent()->get_remote_bda(), this->char_handle_);
      break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      this->notify_registered_ = true;
      ESP_LOGI(TAG, "notify зарегистрирован, старт handshake");
      this->start_handshake_();
      break;
    }
    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.handle != this->char_handle_) break;
      this->handle_notify_(param->notify.value, param->notify.value_len);
      break;
    }
    default:
      break;
  }
}

void DeyeBMS::write_char_(const uint8_t *data, size_t len) {
  esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                           this->char_handle_, len, (uint8_t *) data, ESP_GATT_WRITE_TYPE_NO_RSP,
                           ESP_GATT_AUTH_REQ_NONE);
}

// ---------- handshake ----------
void DeyeBMS::start_handshake_() {
  // random = 16 hex-символов (uppercase)
  const char *H = "0123456789ABCDEF";
  this->random_.clear();
  for (int i = 0; i < 16; i++) this->random_ += H[esp_random() & 0x0F];

  // кадр: 01 ff "random" <random ascii> crc16le
  std::vector<uint8_t> f = {0x01, 0xff};
  const char *rnd = "random";
  f.insert(f.end(), rnd, rnd + 6);
  f.insert(f.end(), this->random_.begin(), this->random_.end());
  uint16_t c = crc16_(f.data(), f.size());
  f.push_back(c & 0xff);
  f.push_back(c >> 8);

  this->rx_buf_.clear();
  this->write_char_(f.data(), f.size());
  this->state_ = HANDSHAKE_SENT;
  this->derive_key_();  // ключ можем вычислить сразу
  ESP_LOGD(TAG, "handshake random отправлен");
}

void DeyeBMS::send_result_() {
  // кадр: 01 ff "result" 01 crc16le
  std::vector<uint8_t> f = {0x01, 0xff};
  const char *r = "result";
  f.insert(f.end(), r, r + 6);
  f.push_back(0x01);
  uint16_t c = crc16_(f.data(), f.size());
  f.push_back(c & 0xff);
  f.push_back(c >> 8);
  this->write_char_(f.data(), f.size());
  this->state_ = READY;
  ESP_LOGI(TAG, "handshake завершён, сессия готова");
}

void DeyeBMS::send_next_poll_() {
  const char *reqs[3] = {REQ_CELLS, REQ_MAIN, REQ_INFO};
  const char *req = reqs[this->poll_idx_ % 3];
  size_t len = strlen(req);
  this->poll_idx_ = (this->poll_idx_ + 1) % 3;
  std::vector<uint8_t> enc;
  this->aes_encrypt_((const uint8_t *) req, len, enc);
  this->rx_buf_.clear();
  ESP_LOGV(TAG, "poll %s (%d ascii -> %d шифр)", req, (int) len, (int) enc.size());
  this->write_char_(enc.data(), enc.size());
}

// ---------- приём ----------
void DeyeBMS::handle_notify_(const uint8_t *data, size_t len) {
  ESP_LOGV(TAG, "notify: state=%d len=%d first=%02x%02x", this->state_, (int) len,
           len > 0 ? data[0] : 0, len > 1 ? data[1] : 0);
  // handshake-ответы приходят открытым текстом
  if (this->state_ == HANDSHAKE_SENT && len >= 8 && data[0] == 0x01 && data[1] == 0xff) {
    // это cipher-ответ батареи — просто продолжаем result
    this->send_result_();
    return;
  }
  // данные — накапливаем и пробуем расшифровать
  this->rx_buf_.insert(this->rx_buf_.end(), data, data + len);
  this->try_parse_response_();
}

void DeyeBMS::try_parse_response_() {
  if (this->rx_buf_.empty() || this->rx_buf_.size() % 16 != 0) return;
  std::vector<uint8_t> pt;
  if (!this->aes_decrypt_(this->rx_buf_.data(), this->rx_buf_.size(), pt)) return;
  // ответ может прийти сырыми байтами (AA55..) или ASCII-строкой hex ("aa55..")
  if (pt.size() >= 4 && pt[0] == 0x61 && pt[1] == 0x61) {
    // ASCII hex-строка -> байты
    std::vector<uint8_t> raw;
    for (size_t i = 0; i + 1 < pt.size(); i += 2) {
      char h[3] = {(char) pt[i], (char) pt[i + 1], 0};
      if (!isxdigit((int) h[0]) || !isxdigit((int) h[1])) break;
      raw.push_back((uint8_t) strtol(h, nullptr, 16));
    }
    pt.swap(raw);
  }
  ESP_LOGV(TAG, "decrypt %d->%d: %02x%02x func=%02x end=%02x%02x", (int) this->rx_buf_.size(),
           (int) pt.size(), pt.size() > 0 ? pt[0] : 0, pt.size() > 1 ? pt[1] : 0,
           pt.size() > 3 ? pt[3] : 0, pt.size() >= 2 ? pt[pt.size() - 2] : 0,
           pt.size() >= 1 ? pt.back() : 0);
  // валидный кадр: aa 55 ... 0d 0a
  if (pt.size() >= 6 && pt[0] == 0xaa && pt[1] == 0x55 && pt[pt.size() - 2] == 0x0d &&
      pt.back() == 0x0a) {
    this->parse_frame_(pt);
    this->rx_buf_.clear();
  }
  // иначе — ждём ещё notify (кадр не собран)
}

static inline uint16_t le16(const std::vector<uint8_t> &d, size_t i) {
  return (uint16_t) d[i] | ((uint16_t) d[i + 1] << 8);
}
static inline int16_t sle16(const std::vector<uint8_t> &d, size_t i) {
  return (int16_t)((uint16_t) d[i] | ((uint16_t) d[i + 1] << 8));
}

void DeyeBMS::parse_frame_(const std::vector<uint8_t> &f) {
  uint8_t func = f[3];
  if (func == 0x04) {
    // aa 55 01 04 <len> maxV(BE) maxNo minV(BE) minNo diff(BE) cell1..cell16(BE)
    // данные с f[5]
    size_t p = 5;
    uint16_t maxv = ((uint16_t) f[p] << 8) | f[p + 1];
    uint8_t maxno = f[p + 2];
    uint16_t minv = ((uint16_t) f[p + 3] << 8) | f[p + 4];
    uint8_t minno = f[p + 5];
    uint16_t diff = ((uint16_t) f[p + 6] << 8) | f[p + 7];
    size_t cellbase = p + 8;
    for (int i = 0; i < 16; i++) {
      uint16_t mv = ((uint16_t) f[cellbase + i * 2] << 8) | f[cellbase + i * 2 + 1];
      if (this->cell_[i] != nullptr) this->cell_[i]->publish_state(mv / 1000.0f);
    }
    if (this->cell_max_ != nullptr) this->cell_max_->publish_state(maxv / 1000.0f);
    if (this->cell_min_ != nullptr) this->cell_min_->publish_state(minv / 1000.0f);
    if (this->cell_diff_ != nullptr) this->cell_diff_->publish_state(diff);
    if (this->cell_max_no_ != nullptr) this->cell_max_no_->publish_state(maxno);
    if (this->cell_min_no_ != nullptr) this->cell_min_no_->publish_state(minno);
    ESP_LOGD(TAG, "ячейки: max=%umV(#%u) min=%umV(#%u) diff=%umV", maxv, maxno, minv, minno, diff);
  } else if (func == 0xa0) {
    // LE поля (см. реверс): soc@5 soh@7 totalV@10 current@12 tCell@14 maxC@16 minC@18 tMos@20 tEnv@22
    if (f.size() < 26) return;
    float soc = le16(f, 5) / 10.0f;
    float soh = le16(f, 7) / 10.0f;
    float totalv = le16(f, 10) / 100.0f;
    float cur = sle16(f, 12) / 100.0f;
    float tcell = sle16(f, 14) / 10.0f;
    float tmos = sle16(f, 20) / 10.0f;
    float tenv = sle16(f, 22) / 10.0f;
    if (this->soc_ != nullptr) this->soc_->publish_state(soc);
    if (this->soh_ != nullptr) this->soh_->publish_state(soh);
    if (this->total_voltage_ != nullptr) this->total_voltage_->publish_state(totalv);
    if (this->current_ != nullptr) this->current_->publish_state(cur);
    if (this->temp_cell_ != nullptr) this->temp_cell_->publish_state(tcell);
    if (this->temp_mos_ != nullptr) this->temp_mos_->publish_state(tmos);
    if (this->temp_env_ != nullptr) this->temp_env_->publish_state(tenv);
    // хвост A0 (стабильные лимиты/ёмкость), поля LE /10
    if (f.size() >= 36) {
      if (this->charge_current_limit_ != nullptr)
        this->charge_current_limit_->publish_state(le16(f, 24) / 10.0f);
      if (this->discharge_current_limit_ != nullptr)
        this->discharge_current_limit_->publish_state(le16(f, 26) / 10.0f);
      if (this->charge_voltage_limit_ != nullptr)
        this->charge_voltage_limit_->publish_state(le16(f, 28) / 10.0f);
      if (this->discharge_voltage_limit_ != nullptr)
        this->discharge_voltage_limit_->publish_state(le16(f, 30) / 10.0f);
      if (this->full_capacity_ != nullptr)
        this->full_capacity_->publish_state(le16(f, 34) / 10.0f);
    }
    ESP_LOGD(TAG, "телеметрия: soc=%.1f%% totalV=%.2fV I=%.2fA tCell=%.1f tMos=%.1f tEnv=%.1f", soc,
             totalv, cur, tcell, tmos, tenv);
  } else if (func == 0x01) {
    // паспорт: число циклов — big-endian 16-бит на смещении 14 (кадр: ...00 00 00 02...)
    if (f.size() >= 16 && this->cycles_ != nullptr) {
      uint16_t cyc = ((uint16_t) f[14] << 8) | f[15];
      this->cycles_->publish_state(cyc);
      ESP_LOGD(TAG, "паспорт: циклы=%u", cyc);
    }
  }
}

}  // namespace deye_bms
}  // namespace esphome

#endif
