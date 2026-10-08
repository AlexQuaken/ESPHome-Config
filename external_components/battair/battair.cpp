#include "battair.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>

#ifdef USE_ESP32

namespace esphome::battair {

static const char *const TAG = "battair";

// Порядок опроса. 0x02 первым: из него берется число банок для разбора 0x10.
// Что отвечает живой BattAir 5 (08.10.2026, после XOR):
//   02 -> 03 железо: HW, SW, тип (01 LiPo), банок (04), дата выпуска
//   06 -> 07 00 00 00 A9 6F C0 ..., назначение не известно
//   04 -> 05 циклы (u16 @3), ошибки, отключения, перегрев, перезаряд, переразряд
//   10 -> 11 банки, мВ, u16 с байта 4
//   12 -> 13 температура (int8 @3), токи разряда и заряда (u32), байт 12 "уровень"
//   14 -> 15 заводские пороги: 3000/3300/4200/3800 мВ, C-рейты 50 и 700
//   0E -> 0F настройки: ток дозаряда 1000, хранение 3800, отсечка 4200, таймер 168
//   0A -> 0B имя "BattAir 5"
//   16 -> 17 производитель банок "Gaoneng"
//   26 -> 27 пусто, 28 -> 29 71 28 00 00, не известно
// В рекламе (mfg 0xABBA) есть общее напряжение в 10 мВ, температура и циклы,
// но не банки, поэтому без подключения просадку одной банки не увидеть.
static const uint8_t COMMANDS[] = {0x02, 0x06, 0x04, 0x10, 0x12, 0x14, 0x0E, 0x0A, 0x16, 0x26, 0x28};
static const size_t COMMAND_COUNT = sizeof(COMMANDS);

static const uint32_t SESSION_TIMEOUT_MS = 25000;  // на весь сеанс с одним датчиком
static const uint32_t COMMAND_TIMEOUT_MS = 1500;
static const uint32_t COMMAND_GAP_MS = 80;
static const uint32_t SETUP_DELAY_MS = 300;
static const uint32_t RETRY_DELAY_MS = 2000;
static const uint32_t CLOSE_TIMEOUT_MS = 15000;
static const uint32_t ADV_LOG_INTERVAL_MS = 300000;
static const uint32_t BOOT_DELAY_MS = 30000;  // после загрузки даем трекеру увидеть датчики

static const char *const BATTERY_TYPES[] = {"LiHv", "LiPo", "LiIon", "LiFe", "Pb", "NiMh/Cd", "LiUHv"};

static std::string hex_str(const uint8_t *d, size_t n) {
  static const char *const digits = "0123456789ABCDEF";
  std::string s;
  s.reserve(n * 3);
  for (size_t i = 0; i < n; i++) {
    if (i)
      s += ' ';
    s += digits[d[i] >> 4];
    s += digits[d[i] & 0x0F];
  }
  return s;
}

static std::string mac_str(uint64_t a) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", (unsigned) ((a >> 40) & 0xFF),
           (unsigned) ((a >> 32) & 0xFF), (unsigned) ((a >> 24) & 0xFF), (unsigned) ((a >> 16) & 0xFF),
           (unsigned) ((a >> 8) & 0xFF), (unsigned) (a & 0xFF));
  return buf;
}

// Имя в эфире вида BATAIR<имя>, Arduino еще вырезал из него "0000".
static std::string normalize_name(std::string s) {
  auto p = s.find("BATAIR");
  if (p != std::string::npos)
    s.erase(0, p + 6);
  auto z = s.find("0000");
  if (z != std::string::npos)
    s.erase(z, 4);
  while (!s.empty() && (s.back() == ' ' || s.back() == '\0'))
    s.pop_back();
  while (!s.empty() && s.front() == ' ')
    s.erase(0, 1);
  return s;
}

static uint16_t u16(const uint8_t *d, size_t i) { return d[i] | (d[i + 1] << 8); }
static uint32_t u32(const uint8_t *d, size_t i) {
  return d[i] | (d[i + 1] << 8) | (d[i + 2] << 16) | ((uint32_t) d[i + 3] << 24);
}

void BattAirHub::add_battery(const std::string &ble_name, uint64_t address) {
  Battery b;
  b.ble_name = normalize_name(ble_name);
  b.address = address;
  b.address_from_config = address != 0;
  this->bats_.push_back(b);
}

void BattAirHub::setup() {
#ifdef USE_OTA_STATE_LISTENER
  ota::get_global_ota_callback()->add_global_state_listener(this);
#endif
}

#ifdef USE_OTA_STATE_LISTENER
void BattAirHub::on_ota_global_state(ota::OTAState state, float progress, uint8_t error,
                                     ota::OTAComponent *component) {
  if (state == ota::OTA_STARTED) {
    this->ota_active_ = true;
    if (this->phase_ != PH_IDLE) {
      ESP_LOGI(TAG, "OTA: обход прерван");
      this->phase_ = PH_IDLE;
      this->cur_ = -1;
    }
  } else if (state == ota::OTA_ERROR || state == ota::OTA_ABORT) {
    this->ota_active_ = false;
  }
}
#endif

void BattAirHub::dump_config() {
  ESP_LOGCONFIG(TAG, "BattAir:");
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Attempts: %u", this->attempts_);
  for (size_t i = 0; i < this->bats_.size(); i++) {
    auto &b = this->bats_[i];
    ESP_LOGCONFIG(TAG, "  Bat%u '%s' %s", (unsigned) i + 1, b.ble_name.c_str(),
                  b.address ? mac_str(b.address).c_str() : "MAC по имени из эфира");
  }
}

bool BattAirHub::parse_device(const espbt::ESPBTDevice &device) {
  std::string adv_name = device.get_name().str();
  uint64_t addr = device.address_uint64();
  bool is_battair = adv_name.find("BATAIR") != std::string::npos;

  for (size_t i = 0; i < this->bats_.size(); i++) {
    auto &b = this->bats_[i];
    bool match = b.address != 0 ? b.address == addr : (is_battair && normalize_name(adv_name) == b.ble_name);
    if (!match)
      continue;
    if (b.address == 0) {
      b.address = addr;
      ESP_LOGI(TAG, "Bat%u '%s' найден в эфире: %s", (unsigned) i + 1, b.ble_name.c_str(), mac_str(addr).c_str());
    }
    b.address_type = device.get_address_type();
    b.rssi = device.get_rssi();
    b.rssi_valid = true;
    // Раз в 5 минут пишем рекламу целиком: вдруг напряжения есть прямо в ней,
    // тогда датчики можно будет слушать без подключения.
    uint32_t now = millis();
    if (b.last_adv_log == 0 || now - b.last_adv_log > ADV_LOG_INTERVAL_MS) {
      b.last_adv_log = now;
      for (auto &md : device.get_manufacturer_datas()) {
        char ub[esp32_ble::UUID_STR_LEN];
        ESP_LOGD(TAG, "Bat%u adv '%s' rssi %d mfg %s: %s", (unsigned) i + 1, adv_name.c_str(), b.rssi,
                 md.uuid.to_str(ub), hex_str(md.data.data(), md.data.size()).c_str());
      }
    }
    return true;
  }

  if (is_battair) {
    for (auto a : this->unknown_logged_)
      if (a == addr)
        return false;
    this->unknown_logged_.push_back(addr);
    ESP_LOGW(TAG, "Чужой BattAir в эфире: '%s' -> '%s' %s rssi %d", adv_name.c_str(),
             normalize_name(adv_name).c_str(), mac_str(addr).c_str(), device.get_rssi());
  }
  return false;
}

void BattAirHub::update() {
  if (this->ota_active_)
    return;
  if (this->phase_ != PH_IDLE) {
    ESP_LOGW(TAG, "Прошлый обход еще идет, пропускаю");
    return;
  }
  if (millis() < BOOT_DELAY_MS) {
    this->set_timeout("first_poll", BOOT_DELAY_MS - millis() + 1000, [this]() { this->update(); });
    return;
  }
  this->counter_++;
  this->found_ = 0;
  this->log_.clear();
  this->cur_ = -1;
  ESP_LOGI(TAG, "Обход #%u", this->counter_);
  this->next_battery_();
}

void BattAirHub::next_battery_() {
  for (this->cur_++; this->cur_ < (int) this->bats_.size(); this->cur_++) {
    auto &b = this->bat_();
    b.attempt = 0;
    b.ok = false;
    if (b.address == 0) {
      ESP_LOGW(TAG, "Bat%d '%s' не видно в эфире", this->cur_ + 1, b.ble_name.c_str());
      if (b.online != nullptr)
        b.online->publish_state(false);
      this->log_ += "B" + std::to_string(this->cur_ + 1) + " -; ";
      continue;
    }
    this->start_attempt_();
    return;
  }
  this->finish_cycle_();
}

void BattAirHub::start_attempt_() {
  auto &b = this->bat_();
  b.attempt++;
  b.s = Session();
  this->handle_ = 0;
  this->handshake_sent_ = false;
  this->setup_at_ = 0;
  this->cmd_idx_ = 0;
  this->cmd_pending_ = false;
  this->next_cmd_at_ = 0;
  this->phase_ = PH_CONNECT;
  this->deadline_ = millis() + SESSION_TIMEOUT_MS;
  ESP_LOGD(TAG, "Bat%d '%s' %s, попытка %u", this->cur_ + 1, b.ble_name.c_str(), mac_str(b.address).c_str(),
           b.attempt);
  this->parent()->set_address(b.address);
  this->parent()->set_remote_addr_type(static_cast<esp_ble_addr_type_t>(b.address_type));
  this->parent()->connect();
}

void BattAirHub::close_(bool ok) {
  if (ok)
    this->bat_().ok = true;
  this->phase_ = PH_CLOSE;
  this->deadline_ = millis() + CLOSE_TIMEOUT_MS;
  this->parent()->disconnect();
}

void BattAirHub::after_close_() {
  auto &b = this->bat_();
  if (!b.ok && b.attempt < this->attempts_) {
    this->phase_ = PH_WAIT;
    this->wait_until_ = millis() + RETRY_DELAY_MS;
    return;
  }
  if (b.online != nullptr)
    b.online->publish_state(b.ok);
  if (b.ok) {
    this->found_++;
    this->publish_battery_(b);
  } else {
    ESP_LOGW(TAG, "Bat%d '%s' не ответил за %u попыток", this->cur_ + 1, b.ble_name.c_str(), b.attempt);
    this->log_ += "B" + std::to_string(this->cur_ + 1) + " нет связи; ";
  }
  this->phase_ = PH_IDLE;
  this->next_battery_();
}

void BattAirHub::finish_cycle_() {
  this->phase_ = PH_IDLE;
  this->cur_ = -1;
  if (this->found_sensor_ != nullptr)
    this->found_sensor_->publish_state(this->found_);
  if (this->counter_sensor_ != nullptr)
    this->counter_sensor_->publish_state(this->counter_);
  std::string msg = std::to_string(this->found_) + "/" + std::to_string(this->bats_.size()) + ": " + this->log_ +
                    "next " + std::to_string(this->get_update_interval() / 1000) + "s";
  if (msg.size() > 250)
    msg.resize(250);
  ESP_LOGI(TAG, "%s", msg.c_str());
  if (this->log_sensor_ != nullptr)
    this->log_sensor_->publish_state(msg);
}

void BattAirHub::loop() {
  if (this->phase_ == PH_IDLE)
    return;
  uint32_t now = millis();

  switch (this->phase_) {
    case PH_WAIT:
      if ((int32_t) (now - this->wait_until_) >= 0)
        this->start_attempt_();
      return;

    case PH_CLOSE:
      if (this->parent()->state() == espbt::ClientState::IDLE || (int32_t) (now - this->deadline_) >= 0)
        this->after_close_();
      return;

    default:
      break;
  }

  if ((int32_t) (now - this->deadline_) >= 0) {
    ESP_LOGW(TAG, "Bat%d: таймаут сеанса (шаг %u, команда %u)", this->cur_ + 1, this->phase_,
             (unsigned) this->cmd_idx_);
    // Что успели получить, все равно публикуем, если есть напряжения банок.
    this->close_(this->bat_().s.cells);
    return;
  }

  switch (this->phase_) {
    case PH_CONNECT:
      // connect() сразу переводит клиента в CONNECTING; вернулся в IDLE, значит не соединился.
      if (this->parent()->state() == espbt::ClientState::IDLE) {
        ESP_LOGD(TAG, "Bat%d: соединение не установлено", this->cur_ + 1);
        this->phase_ = PH_CLOSE;
      }
      break;

    case PH_SETUP:
      if (this->setup_at_ == 0)
        break;
      if (!this->handshake_sent_ && now - this->setup_at_ >= SETUP_DELAY_MS) {
        this->send_handshake_();
        this->handshake_sent_ = true;
        this->handshake_at_ = now;
      } else if (this->handshake_sent_ && now - this->handshake_at_ >= SETUP_DELAY_MS) {
        this->phase_ = PH_QUERY;
        this->send_command_();
      }
      break;

    case PH_QUERY:
      if (this->next_cmd_at_ != 0 && (int32_t) (now - this->next_cmd_at_) >= 0) {
        this->next_cmd_at_ = 0;
        this->send_command_();
      } else if (this->cmd_pending_ && now - this->cmd_at_ >= COMMAND_TIMEOUT_MS) {
        ESP_LOGD(TAG, "Bat%d: нет ответа на 0x%02X", this->cur_ + 1, COMMANDS[this->cmd_idx_]);
        this->cmd_pending_ = false;
        this->cmd_idx_++;
        this->send_command_();
      }
      break;

    default:
      break;
  }
}

void BattAirHub::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                     esp_ble_gattc_cb_param_t *param) {
  if (this->cur_ < 0)
    return;

  switch (event) {
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      if (this->phase_ != PH_CONNECT)
        break;
      auto *chr = this->parent()->get_characteristic(espbt::ESPBTUUID::from_uint16(BATTAIR_SERVICE_UUID),
                                                     espbt::ESPBTUUID::from_uint16(BATTAIR_CHAR_UUID));
      if (chr == nullptr) {
        ESP_LOGW(TAG, "Bat%d: нет характеристики BA01", this->cur_ + 1);
        this->close_(false);
        break;
      }
      this->handle_ = chr->handle;
      auto err = this->parent()->register_for_notify(this->handle_);
      if (err) {
        ESP_LOGW(TAG, "Bat%d: register_for_notify %d", this->cur_ + 1, err);
        this->close_(false);
        break;
      }
      this->phase_ = PH_SETUP;
      break;
    }

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      if (this->phase_ != PH_SETUP || param->reg_for_notify.handle != this->handle_)
        break;
      if (param->reg_for_notify.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Bat%d: подписка не удалась, status %d", this->cur_ + 1, param->reg_for_notify.status);
        this->close_(false);
        break;
      }
      // Дескриптор CCCD пишет BLEClientBase. Дальше кэш сервисов нам не нужен.
      this->node_state = espbt::ClientState::ESTABLISHED;
      this->setup_at_ = millis();
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.handle != this->handle_ || (this->phase_ != PH_SETUP && this->phase_ != PH_QUERY))
        break;
      this->on_packet_(param->notify.value, param->notify.value_len);
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT:
    case ESP_GATTC_CLOSE_EVT: {
      if (this->phase_ == PH_SETUP || this->phase_ == PH_QUERY) {
        ESP_LOGW(TAG, "Bat%d: датчик оборвал связь (команда %u)", this->cur_ + 1, (unsigned) this->cmd_idx_);
        this->bat_().ok = this->bat_().s.cells;
        this->phase_ = PH_CLOSE;
        this->deadline_ = millis() + CLOSE_TIMEOUT_MS;
      }
      break;
    }

    default:
      break;
  }
}

void BattAirHub::write_(const uint8_t *data, size_t len) {
  uint8_t buf[16];
  uint8_t k = this->key_();
  for (size_t i = 0; i < len; i++)
    buf[i] = data[i] ^ k;
  auto err = esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(), this->handle_,
                                      len, buf, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (err)
    ESP_LOGW(TAG, "Bat%d: write_char %d", this->cur_ + 1, err);
}

void BattAirHub::send_handshake_() {
  // 00 00 00, MAC задом наперед, 01, последний байт MAC.
  uint64_t a = this->bat_().address;
  uint8_t p[11] = {0, 0, 0};
  for (int i = 0; i < 6; i++)
    p[3 + i] = (a >> (8 * i)) & 0xFF;
  p[9] = 0x01;
  p[10] = a & 0xFF;
  this->write_(p, sizeof(p));
}

void BattAirHub::send_command_() {
  if (this->cmd_idx_ >= COMMAND_COUNT) {
    this->close_(this->bat_().s.cells);
    return;
  }
  uint8_t p[4] = {COMMANDS[this->cmd_idx_], 0, 0, 0};
  this->write_(p, sizeof(p));
  this->cmd_pending_ = true;
  this->cmd_at_ = millis();
}

void BattAirHub::on_packet_(const uint8_t *raw, size_t len) {
  if (len == 0 || len > 64)
    return;
  uint8_t d[64];
  uint8_t k = this->key_();
  for (size_t i = 0; i < len; i++)
    d[i] = raw[i] ^ k;
  auto &s = this->bat_().s;
  ESP_LOGD(TAG, "Bat%d <- %s", this->cur_ + 1, hex_str(d, len).c_str());

  switch (d[0]) {
    case 0x03:  // железо
      if (len < 11)
        break;
      s.hw_major = d[3];
      s.hw_minor = d[4];
      s.sw_major = d[5];
      s.sw_minor = d[6];
      s.battery_type = d[9];
      if (d[10] >= 1 && d[10] <= 6)
        s.cell_count = d[10];
      if (len >= 18)
        memcpy(s.production, d + 14, 4);
      s.hw = true;
      break;

    case 0x05:  // статистика: циклы и счетчики аварий
      if (len < 14)
        break;
      s.cycles = u16(d, 3);
      s.errors = u16(d, 5);
      s.outages = d[7];
      s.overheats = u16(d, 8);
      s.overcharges = u16(d, 10);
      s.overdischarges = u16(d, 12);
      s.stats = true;
      break;

    case 0x11:  // напряжения банок, мВ
      if (len < 4u + 2u * s.cell_count)
        break;
      for (uint8_t c = 0; c < s.cell_count; c++)
        s.cell_mv[c] = (int16_t) u16(d, 4 + 2 * c);
      s.cells = true;
      break;

    case 0x13:  // температура и токи
      if (len < 12)
        break;
      s.temperature = (int8_t) d[3];
      s.discharge_current = u32(d, 4);
      s.charge_current = u32(d, 8);
      if (len > 12) {
        s.level_raw = d[12];
        s.level = true;
      }
      s.realtime = true;
      break;

    case 0x0B: {  // имя: длина в d[3], дальше строка с хвостом из пробелов.
      // В Arduino-классе 0x07 и 0x0B были перепутаны, проверено на живом датчике 08.10.2026.
      if (len < 4)
        break;
      size_t n = std::min<size_t>(d[3], len - 4);
      s.name = normalize_name(std::string((const char *) d + 4, strnlen((const char *) d + 4, n)));
      break;
    }

    case 0x17: {  // производитель банок, строка с нулем на конце ("Gaoneng")
      if (len < 5)
        break;
      size_t n = std::min<size_t>(d[3], len - 4);
      s.maker = std::string((const char *) d + 4, strnlen((const char *) d + 4, n));
      break;
    }

    default:
      break;
  }

  if (this->phase_ == PH_QUERY && this->cmd_pending_ && d[0] == COMMANDS[this->cmd_idx_] + 1) {
    this->cmd_pending_ = false;
    this->cmd_idx_++;
    this->next_cmd_at_ = millis() + COMMAND_GAP_MS;
  }
}

void BattAirHub::publish_battery_(Battery &b) {
  auto &s = b.s;
  auto pub = [&b](uint8_t kind, float v) {
    if (b.sensors[kind] != nullptr)
      b.sensors[kind]->publish_state(v);
  };
  auto pub_text = [&b](uint8_t kind, const std::string &v) {
    if (b.texts[kind] != nullptr)
      b.texts[kind]->publish_state(v);
  };
  int n = this->cur_ + 1;

  pub_text(TEXT_LABEL, s.name.empty() ? b.ble_name : s.name);
  if (s.hw) {
    pub_text(TEXT_TYPE, s.battery_type < 7 ? BATTERY_TYPES[s.battery_type] : "?" + std::to_string(s.battery_type));
    // Дата выпуска в 0x03 байты 14..17, у BattAir 5 это 00 00 04 22: похоже на 2022-04,
    // но формат не подтвержден, поэтому отдаем как есть.
    char fw[64];
    snprintf(fw, sizeof(fw), "HW %u.%u SW %u.%u, %s, %02X%02X%02X%02X", s.hw_major, s.hw_minor, s.sw_major,
             s.sw_minor, s.maker.empty() ? "?" : s.maker.c_str(), s.production[0], s.production[1],
             s.production[2], s.production[3]);
    pub_text(TEXT_FIRMWARE, fw);
  }

  std::string line = "B" + std::to_string(n);
  if (s.cells) {
    // Проверка из v2.2: живая LiPo-банка лежит в 2,5..4,3 В, остальное мусор в пакете.
    bool valid = true;
    int32_t total = 0, lo = INT32_MAX, hi = INT32_MIN;
    for (uint8_t c = 0; c < s.cell_count; c++) {
      int32_t mv = s.cell_mv[c];
      if (mv < 2500 || mv > 4300)
        valid = false;
      total += mv;
      lo = std::min(lo, mv);
      hi = std::max(hi, mv);
    }
    if (valid) {
      for (uint8_t c = 0; c < s.cell_count && c < 4; c++)
        pub(SENSOR_CELL_1 + c, s.cell_mv[c] / 1000.0f);
      pub(SENSOR_VOLTAGE, total / 1000.0f);
      pub(SENSOR_CELL_DELTA, hi - lo);
      char v[12];
      snprintf(v, sizeof(v), " %.2f", total / 1000.0f);
      line += v;
    } else {
      ESP_LOGW(TAG, "Bat%d: банки вне 2.5..4.3 В: %d %d %d %d", n, (int) s.cell_mv[0], (int) s.cell_mv[1],
               (int) s.cell_mv[2], (int) s.cell_mv[3]);
      line += " bad";
    }
  }
  if (s.stats) {
    if (s.cycles <= 1000)
      pub(SENSOR_CYCLES, s.cycles);
    pub(SENSOR_ERRORS, s.errors);
    pub(SENSOR_OUTAGES, s.outages);
    pub(SENSOR_OVERHEATS, s.overheats);
    pub(SENSOR_OVERCHARGES, s.overcharges);
    pub(SENSOR_OVERDISCHARGES, s.overdischarges);
  }
  if (s.realtime) {
    pub(SENSOR_TEMPERATURE, s.temperature);
    pub(SENSOR_CHARGE_CURRENT, s.charge_current);
    pub(SENSOR_DISCHARGE_CURRENT, s.discharge_current);
    if (s.level)
      pub(SENSOR_LEVEL, s.level_raw);
  }
  if (b.rssi_valid)
    pub(SENSOR_RSSI, b.rssi);

  ESP_LOGI(TAG, "Bat%d '%s': %d|%d|%d|%d мВ, циклов %u, %d°C, заряд %" PRIu32 " разряд %" PRIu32 " ур. %u", n,
           b.ble_name.c_str(), (int) s.cell_mv[0], (int) s.cell_mv[1], (int) s.cell_mv[2], (int) s.cell_mv[3],
           s.cycles, s.temperature,
           s.charge_current, s.discharge_current, s.level_raw);
  this->log_ += line + "; ";
}

}  // namespace esphome::battair

#endif
