#include "ninebot.h"

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/components/esp32_ble/ble.h"

#include <esp_bt.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#ifdef USE_ESP32

namespace esphome::ninebot {

static const char *const TAG = "ninebot";
// Шаги сеанса дублируются в сенсор Log: панель живет на INFO, DEBUG ее перегружает.
// snprintf режет по байтам, обрезанный UTF-8 в text_sensor ломает JSON в HA.
static void nb_utf8_trim(char *s) {
  size_t l = strlen(s), k = l;
  while (k && ((uint8_t) s[k - 1] & 0xC0) == 0x80)
    k--;
  if (k && ((uint8_t) s[k - 1] & 0x80)) {
    uint8_t c = s[k - 1];
    size_t need = c >= 0xF0 ? 4 : c >= 0xE0 ? 3 : 2;
    if (l - (k - 1) < need)
      s[k - 1] = 0;
  }
}
#define NBLOG(...) \
  do { \
    char b_[160]; \
    snprintf(b_, sizeof(b_), __VA_ARGS__); \
    nb_utf8_trim(b_); \
    ESP_LOGI(TAG, "%s", b_); \
    if (this->texts_[TEXT_LOG]) \
      this->texts_[TEXT_LOG]->publish_state(b_); \
  } while (0)
static const uint32_t STORE_MAGIC = 0x4E423301;  // "NB3" v1

static const uint8_t BT_ID = 0x3E;
static const uint8_t T_BLE = 0x04;
static const uint8_t CMD_READ = 0x01, CMD_WRITE = 0x02, CMD_WRITE_NR = 0x03;
static const uint8_t CMD_READ_RESP = 0x04, CMD_WRITE_RESP = 0x05;
static const uint8_t CMD_PRECOMM = 0x5B, CMD_SETPWD = 0x5C, CMD_AUTH = 0x5D;

// Кандидаты адреса VCU и BMS. Для hw 258 (MAX G3) по boards.json: vcu 0x16,
// bms1 0x07, bms2 0x06; 0x09 запасной. Первый ответивший запоминается во флеш.
static const uint8_t VCU_CANDS[] = {0x16, 0x09};
static const uint8_t BMS_CANDS[] = {0x07, 0x06};

// Что читаем за обход. group: 0 vcu, 1 bms1. Длина как в таблице G3.
struct Reg {
  uint8_t group, index, len, kind;
};
static const Reg REGS[] = {
    {0, 0x55, 2, SENSOR_BATTERY}, {0, 0x5F, 2, SENSOR_RANGE},    {0, 0x62, 4, SENSOR_ODOMETER},
    {0, 0x6B, 2, SENSOR_BODY_TEMP}, {0, 0x58, 2, SENSOR_ERROR},  {0, 0x59, 2, SENSOR_WARNING},
    {1, 0x8F, 2, SENSOR_BMS_SOC}, {1, 0x8C, 2, SENSOR_VOLTAGE},  {1, 0x8D, 2, SENSOR_CURRENT},
    {1, 0x96, 4, SENSOR_BMS_TEMP}, {1, 0x8A, 2, SENSOR_REMAIN_CAP}, {1, 0x59, 2, SENSOR_CYCLES},
    {1, 0x94, 2, SENSOR_TIME_FULL}, {1, 0xA0, 26, SENSOR_CELL_MIN},
};

static const uint32_t SESSION_TIMEOUT_MS = 20000;
static const uint32_t STEP_TIMEOUT_MS = 2000;
static const uint32_t READ_TIMEOUT_MS = 1200;
static const uint32_t FRAG_GAP_MS = 12;
static const uint32_t SETUP_DELAY_MS = 350;
static const uint32_t RETRY_DELAY_MS = 2000;
static const uint32_t CLOSE_TIMEOUT_MS = 240000;
static const uint32_t BLE_OFF_MS = 3000;
static const uint32_t BLE_SETTLE_MS = 20000;
static const uint32_t PAIR_WINDOW_MS = 120000;
static const uint8_t MAX_ATTEMPTS = 3;

static std::string hex_str(const uint8_t *d, size_t n) {
  static const char *const h = "0123456789ABCDEF";
  std::string s;
  s.reserve(n * 3);
  for (size_t i = 0; i < n; i++) {
    if (i)
      s += ' ';
    s += h[d[i] >> 4];
    s += h[d[i] & 15];
  }
  return s;
}
static std::string mac_str(uint64_t a) {
  char b[18];
  snprintf(b, sizeof b, "%02X:%02X:%02X:%02X:%02X:%02X", (unsigned) ((a >> 40) & 0xFF), (unsigned) ((a >> 32) & 0xFF),
           (unsigned) ((a >> 24) & 0xFF), (unsigned) ((a >> 16) & 0xFF), (unsigned) ((a >> 8) & 0xFF),
           (unsigned) (a & 0xFF));
  return b;
}
static uint16_t u16(const uint8_t *d) { return d[0] | (d[1] << 8); }
static uint32_t u32(const uint8_t *d) { return d[0] | (d[1] << 8) | (d[2] << 16) | ((uint32_t) d[3] << 24); }

// java.util.Random: 48-битный LCG. Нужен для генерации пароля в SET_PWD.
struct JavaRandom {
  uint64_t seed;
  explicit JavaRandom(int64_t s) { seed = ((uint64_t) s ^ 0x5DEECE66DULL) & ((1ULL << 48) - 1); }
  int32_t next(int bits) {
    seed = (seed * 0x5DEECE66DULL + 0xB) & ((1ULL << 48) - 1);
    return (int32_t) (seed >> (48 - bits));
  }
  void next_bytes(uint8_t *out, size_t n) {
    for (size_t i = 0; i < n;) {
      int32_t r = next(32);
      for (int k = 0; k < 4 && i < n; k++, i++) {
        out[i] = (uint8_t) (r & 0xFF);
        r >>= 8;
      }
    }
  }
};

void NinebotHub::setup() {
#ifdef USE_OTA_STATE_LISTENER
  ota::get_global_ota_callback()->add_global_state_listener(this);
#endif
  // Ключ флеша завязан на MAC/суффикс, чтобы на разных платах не путалось.
  uint32_t h = this->cfg_mac_ ? (uint32_t) this->cfg_mac_ : (0xB0000000u | this->mac_suffix_);
  this->pref_ = global_preferences->make_preference<Stored>(h);
  if (this->pref_.load(&this->st_) && this->st_.magic == STORE_MAGIC) {
    if (this->st_.mac)
      this->address_ = this->st_.mac;
    ESP_LOGCONFIG(TAG, "Загружено из флеша: mac %s, пароль %s, vcu 0x%02X bms 0x%02X",
                  this->st_.mac ? mac_str(this->st_.mac).c_str() : "-", this->st_.has_pwd ? "есть" : "нет",
                  this->st_.vcu, this->st_.bms);
  } else {
    memset(&this->st_, 0, sizeof(this->st_));
    this->st_.magic = STORE_MAGIC;
    this->st_.variant = 0xFF;
  }
  if (this->cfg_mac_)
    this->address_ = this->cfg_mac_;
  // Готовый пароль из конфига перебивает флеш: удобно раздать один ключ платам.
  if (this->cfg_pwd_hex_.size() == 32) {
    for (int i = 0; i < 16; i++)
      this->st_.pwd[i] = (uint8_t) strtol(this->cfg_pwd_hex_.substr(i * 2, 2).c_str(), nullptr, 16);
    this->st_.has_pwd = 1;
    ESP_LOGCONFIG(TAG, "пароль задан в конфиге, кнопка не нужна");
  }
  this->publish_bin_(BINARY_PAIRED, this->st_.has_pwd);
  this->status_(this->st_.has_pwd ? "готов" : "не сопряжен");
}

void NinebotHub::save_() {
  this->st_.magic = STORE_MAGIC;
  this->pref_.save(&this->st_);
}

void NinebotHub::dump_config() {
  ESP_LOGCONFIG(TAG, "Ninebot:");
  LOG_UPDATE_INTERVAL(this);
  if (this->cfg_mac_)
    ESP_LOGCONFIG(TAG, "  MAC: %s", mac_str(this->cfg_mac_).c_str());
  else if (this->has_suffix_)
    ESP_LOGCONFIG(TAG, "  MAC по последним байтам %04X из эфира", this->mac_suffix_);
  if (!this->cfg_name_.empty())
    ESP_LOGCONFIG(TAG, "  BLE name: %s", this->cfg_name_.c_str());
}

#ifdef USE_OTA_STATE_LISTENER
void NinebotHub::on_ota_global_state(ota::OTAState state, float, uint8_t, ota::OTAComponent *) {
  if (state == ota::OTA_STARTED) {
    this->ota_active_ = true;
    if (this->phase_ != PH_IDLE) {
      this->phase_ = PH_IDLE;
      if (this->parent()->state() != espbt::ClientState::IDLE)
        this->parent()->disconnect();
    }
  } else if (state == ota::OTA_ERROR || state == ota::OTA_ABORT) {
    this->ota_active_ = false;
  }
}
#endif

// Ищем самокат в эфире: по MAC из конфига, по последним байтам MAC, либо по имени.
bool NinebotHub::parse_device(const espbt::ESPBTDevice &device) {
  uint64_t addr = device.address_uint64();
  bool match = false;
  if (this->cfg_mac_)
    match = addr == this->cfg_mac_;
  else if (this->st_.mac)
    match = addr == this->st_.mac;
  else if (this->has_suffix_)
    match = (addr & 0xFFFF) == this->mac_suffix_;
  else if (!this->cfg_name_.empty())
    match = device.get_name().str() == this->cfg_name_;
  if (!match)
    return false;
  this->address_ = addr;
  this->address_type_ = device.get_address_type();
  this->last_seen_ = millis();
  this->rssi_ = device.get_rssi();
  this->rssi_valid_ = true;
  std::string name = device.get_name().str();
  if (!name.empty()) {
    bool ascii = std::all_of(name.begin(), name.end(), [](char ch) { return (uint8_t) ch < 0x80; });
    (ascii ? this->adv_name_ : this->adv_alt_name_) = name;
  }
  if (!this->adv_logged_) {  // один раз за загрузку: версия протокола в manufacturer data
    this->adv_logged_ = true;
    NBLOG("эфир: '%s' тип %u rssi %d", name.c_str(), (unsigned) this->address_type_, this->rssi_);
    for (auto &m : device.get_manufacturer_datas()) {
      char ub[espbt::UUID_STR_LEN];
      NBLOG("mfg %s: %s", m.uuid.to_str(ub), hex_str(m.data.data(), std::min(m.data.size(), (size_t) 24)).c_str());
    }
  }
  return true;
}

void NinebotHub::update() {
  if (this->ota_active_ || this->ble_restarting_ || this->paused_)
    return;
  if (this->phase_ != PH_IDLE) {
    // Окно сопряжения: не мешаем идущему обходу, оно подхватится следующим.
    return;
  }
  if (this->address_ == 0) {
    this->status_("не вижу в эфире");
    this->publish_bin_(BINARY_ONLINE, false);
    return;
  }
  if (!this->st_.has_pwd && !this->pairing_) {
    this->status_("не сопряжен, нажми Pair");
    return;
  }
  this->cands_.clear();
  this->cand_i_ = 0;
  this->pair_tried_ = false;
  this->attempt_ = 0;
  this->start_attempt_();
}

void NinebotHub::start_attempt_() {
  this->attempt_++;
  this->phase_ = PH_CONNECT;
  this->deadline_ = millis() + SESSION_TIMEOUT_MS;
  this->write_handle_ = this->notify_handle_ = this->name_handle_ = 0;
  this->rx_.clear();
  this->tx_.clear();
  this->next_tx_at_ = 0;
  this->write_busy_ = false;
  this->counter_ = 0;
  this->session_ok_ = false;
  this->serial_done_ = false;
  this->read_idx_ = 0;
  this->read_pending_ = false;
  this->vcu_ok_ = this->bms_ok_ = this->bms_dead_ = this->vcu_dead_ = false;
  this->cells_sum_ = NAN;
  this->volt_have_ = false;
  this->sends_ = 0;
  this->auth_tries_ = 0;
  this->setup_at_ = 0;  // раньше не сбрасывался: на попытке 2-3 PH_SETUP срабатывал до REG_FOR_NOTIFY
  this->name_done_ = false;
  this->rx_bytes_ = 0;
  this->rx_notifies_ = 0;
  this->last_tx_.clear();
  if (!this->tx_power_set_) {
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    this->tx_power_set_ = true;
  }
  NBLOG("%s, попытка %u", mac_str(this->address_).c_str(), this->attempt_);
  this->parent()->set_address(this->address_);
  this->parent()->set_remote_addr_type(static_cast<esp_ble_addr_type_t>(this->address_type_));
  this->parent()->set_state(espbt::ClientState::DISCOVERED);
}

void NinebotHub::close_(const char *why, bool retry) {
  NBLOG("Закрываю: %s (rx %u б, %u нотиф.)", why, (unsigned) this->rx_bytes_, (unsigned) this->rx_notifies_);
  this->retry_ = retry;
  this->phase_ = PH_CLOSE;
  this->deadline_ = millis() + CLOSE_TIMEOUT_MS;
  if (this->parent()->state() == espbt::ClientState::DISCOVERED)
    this->parent()->set_state(espbt::ClientState::IDLE);
  else
    this->parent()->disconnect();
}

void NinebotHub::after_close_() {
  // в сопряжении самокат рвет связь секунд через 10, если кнопку не нажали: окно держим
  // попытками, около двух минут
  uint8_t max_att = this->pairing_ ? 12 : MAX_ATTEMPTS;
  if (!this->session_ok_ && this->retry_ && this->attempt_ < max_att) {
    this->phase_ = PH_WAIT;
    this->wait_until_ = millis() + RETRY_DELAY_MS;
    return;
  }
  this->phase_ = PH_IDLE;
  this->publish_bin_(BINARY_ONLINE, this->session_ok_);
  if (!this->session_ok_)
    NBLOG("Сеанс не удался (%u попыток)", this->attempt_);
  this->finish_query_();
  if (this->poll_after_) {
    this->poll_after_ = false;
    this->set_timeout("repoll", 1500, [this]() { this->update(); });
  }
}

void NinebotHub::loop() {
  if (this->phase_ == PH_IDLE)
    return;
  uint32_t now = millis();

  if (this->phase_ == PH_WAIT) {
    if ((int32_t) (now - this->wait_until_) >= 0)
      this->start_attempt_();
    return;
  }
  if (this->phase_ == PH_CLOSE) {
    if (this->parent()->state() == espbt::ClientState::IDLE)
      this->after_close_();
    else if ((int32_t) (now - this->deadline_) >= 0)
      this->restart_ble_("клиент не вернулся в IDLE");
    return;
  }
  if ((int32_t) (now - this->deadline_) >= 0) {
    this->close_("таймаут сеанса", true);
    return;
  }

  this->pump_tx_();

  if (this->phase_ == PH_CONNECT && this->parent()->state() == espbt::ClientState::IDLE) {
    this->close_("не подключились", true);
    return;
  }
  if (this->phase_ == PH_SETUP && this->setup_at_ && now - this->setup_at_ >= SETUP_DELAY_MS &&
      (this->name_done_ || now - this->setup_at_ >= 1500)) {
    // ESTABLISHED только теперь: BLEClient освобождает кэш сервисов после него
    this->node_state = espbt::ClientState::ESTABLISHED;
    this->build_cands_();
    if (this->cand_i_ >= this->cands_.size())
      this->cand_i_ = 0;
    NBLOG("GAP '%s', эфир '%s', кандидатов %u, с #%u", this->gap_name_.c_str(), this->adv_name_.c_str(),
          (unsigned) this->cands_.size(), (unsigned) this->cand_i_ + 1);
    this->sends_ = 0;
    this->send_precomm_();
    return;
  }
  // Ручная запись без ответа: подождали паузу и идем дальше по обходу.
  if (this->phase_ == PH_QUERY && this->noreply_ && this->next_tx_at_ == 0 && this->tx_.empty() &&
      now - this->sent_at_ >= READ_TIMEOUT_MS / 3) {
    this->noreply_ = false;
    this->next_read_(true);
    return;
  }
  if (this->phase_ == PH_QUERY && this->read_pending_ && now - this->sent_at_ >= READ_TIMEOUT_MS) {
    NBLOG("нет ответа на регистр");
    this->next_read_(false);
    return;
  }
  // Повтор шагов рукопожатия по таймауту шага.
  if ((this->phase_ == PH_PRECOMM || this->phase_ == PH_SETPWD || this->phase_ == PH_AUTH) &&
      this->next_tx_at_ == 0 && this->tx_.empty() && now - this->sent_at_ >= STEP_TIMEOUT_MS) {
    if (this->phase_ == PH_PRECOMM) {
      // Кадр с чужим ключом самокат молча отбрасывает: молчание тоже значит
      // "не тот ключ", идем к следующему варианту.
      this->write_busy_ = false;  // если WRITE_CHAR_EVT потерялся, очередь не встанет
      if (++this->sends_ >= 2) {
        this->sends_ = 0;
        if (++this->cand_i_ >= this->cands_.size()) {
          this->cand_i_ = 0;
          this->close_("PRE_COMM: молчит на всех кандидатах", false);
          return;
        }
      }
      this->send_precomm_();
    } else if (this->phase_ == PH_SETPWD)
      this->send_setpwd_();
    else
      this->send_auth_();
  }
}

void NinebotHub::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t, esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      if (this->phase_ != PH_CONNECT)
        break;
      // сначала Nordic UART (gen2), сервис Ninebot запасным
      const char *svc_name = "NUS";
      auto nus = espbt::ESPBTUUID::from_raw(NUS_SERVICE_UUID);
      auto *wr = this->parent()->get_characteristic(nus, espbt::ESPBTUUID::from_raw(NUS_WRITE_UUID));
      auto *nt = this->parent()->get_characteristic(nus, espbt::ESPBTUUID::from_raw(NUS_NOTIFY_UUID));
      if (wr == nullptr || nt == nullptr) {
        svc_name = "NB";
        auto svc = espbt::ESPBTUUID::from_raw(SERVICE_UUID);
        wr = this->parent()->get_characteristic(svc, espbt::ESPBTUUID::from_raw(WRITE_UUID));
        nt = this->parent()->get_characteristic(svc, espbt::ESPBTUUID::from_raw(NOTIFY_UUID));
      }
      if (wr == nullptr || nt == nullptr) {
        this->close_("нет сервиса Ninebot", false);
        break;
      }
      this->write_handle_ = wr->handle;
      this->notify_handle_ = nt->handle;
      this->write_no_rsp_ = (wr->properties & ESP_GATT_CHAR_PROP_BIT_WRITE_NR) != 0;
      this->write_rsp_ok_ = (wr->properties & ESP_GATT_CHAR_PROP_BIT_WRITE) != 0;
      {
        // пока кэш сервисов жив: имя GAP 0x2A00, ключ первой фазы
        auto *gn = this->parent()->get_characteristic((uint16_t) 0x1800, (uint16_t) 0x2A00);
        this->name_handle_ = gn ? gn->handle : 0;
      }
      NBLOG("GATT %s wr 0x%04X p%02X nt 0x%04X p%02X name 0x%04X", svc_name, wr->handle, wr->properties, nt->handle,
            nt->properties, this->name_handle_);
      if (this->parent()->register_for_notify(this->notify_handle_)) {
        this->close_("подписка не удалась", true);
        break;
      }
      if (this->name_handle_)
        esp_ble_gattc_read_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(), this->name_handle_,
                                ESP_GATT_AUTH_REQ_NONE);
      else
        this->name_done_ = true;
      this->phase_ = PH_SETUP;
      break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
      if (this->phase_ == PH_SETUP && param->reg_for_notify.handle == this->notify_handle_) {
        if (param->reg_for_notify.status != ESP_GATT_OK)
          NBLOG("reg_for_notify st=0x%02X", param->reg_for_notify.status);
        this->setup_at_ = millis();  // ESTABLISHED здесь не ставим, иначе кэш освободят сразу
      }
      break;
    case ESP_GATTC_WRITE_DESCR_EVT:
      if (this->phase_ != PH_IDLE && param->write.conn_id == this->parent()->get_conn_id()) {
        NBLOG("CCCD 0x%04X st=0x%02X", param->write.handle, param->write.status);
        if (param->write.status == ESP_GATT_INSUF_AUTHENTICATION || param->write.status == ESP_GATT_INSUF_ENCRYPTION)
          this->try_pair_("CCCD");
      }
      break;
    case ESP_GATTC_READ_CHAR_EVT:
      if (this->name_handle_ && param->read.handle == this->name_handle_) {
        if (param->read.status == ESP_GATT_OK && param->read.value_len > 0)
          this->gap_name_.assign((const char *) param->read.value, param->read.value_len);
        else
          NBLOG("0x2A00 st=0x%02X", param->read.status);
        this->name_done_ = true;
      }
      break;
    case ESP_GATTC_NOTIFY_EVT:
      if (param->notify.handle == this->notify_handle_) {
        this->rx_bytes_ += param->notify.value_len;
        if (++this->rx_notifies_ <= 4)
          NBLOG("RX#%u %u: %s", (unsigned) this->rx_notifies_, (unsigned) param->notify.value_len,
                hex_str(param->notify.value, std::min<size_t>(param->notify.value_len, 24)).c_str());
        this->on_bytes_(param->notify.value, param->notify.value_len);
      } else if (this->phase_ != PH_IDLE && ++this->rx_notifies_ <= 4) {
        NBLOG("RX чужой 0x%04X %u б", param->notify.handle, (unsigned) param->notify.value_len);
      }
      break;
    case ESP_GATTC_WRITE_CHAR_EVT:
      this->write_busy_ = false;
      if (param->write.status != ESP_GATT_OK) {
        NBLOG("write 0x%04X st=0x%02X", param->write.handle, param->write.status);
        if (param->write.status == ESP_GATT_INSUF_AUTHENTICATION || param->write.status == ESP_GATT_INSUF_ENCRYPTION)
          this->try_pair_("write");
      }
      break;
    case ESP_GATTC_DISCONNECT_EVT:
    case ESP_GATTC_CLOSE_EVT:
      if (this->phase_ != PH_IDLE && this->phase_ != PH_CLOSE && this->phase_ != PH_WAIT) {
        if (event == ESP_GATTC_DISCONNECT_EVT)
          NBLOG("самокат отключился на шаге %u, reason 0x%02X", this->phase_, param->disconnect.reason);
        this->retry_ = !this->session_ok_;
        this->phase_ = PH_CLOSE;
        this->deadline_ = millis() + CLOSE_TIMEOUT_MS;
      }
      break;
    default:
      break;
  }
}

void NinebotHub::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  if (event == ESP_GAP_BLE_AUTH_CMPL_EVT && this->phase_ != PH_IDLE)
    NBLOG("SMP success=%u fail=0x%02X", param->ble_security.auth_cmpl.success,
          param->ble_security.auth_cmpl.fail_reason);
}

// Самокат просит шифрованный линк: один раз за update() пробуем SMP-сопряжение.
void NinebotHub::try_pair_(const char *why) {
  if (this->pair_tried_)
    return;
  this->pair_tried_ = true;
  NBLOG("%s: нужна шифровка линка, pair()", why);
  this->parent()->pair();
}

void NinebotHub::add_cand_(const std::string &n, uint8_t var, uint8_t sync2) {
  if (n.empty())
    return;
  for (auto &c : this->cands_)
    if (c.name == n && c.var == var && c.sync2 == sync2)
      return;
  this->cands_.push_back({n, var, sync2});
}

static std::string ascii_tail(const std::string &s) {
  size_t i = 0;
  while (i < s.size() && ((uint8_t) s[i] >= 0x80 || s[i] == ' '))
    i++;
  return s.substr(i);
}

// Дополняет список, порядок уже стоящих не трогает: cand_i_ между попытками остается верным.
// Вариант 3 (FW_DATA и как key2, и как статический блок) у libnbcrypto и NinebotCrypto.
void NinebotHub::build_cands_() {
  if (this->st_.name[0] && this->st_.variant < 4)
    this->add_cand_(this->st_.name, this->st_.variant, 0xA5);
  for (std::string *n : {&this->gap_name_, &this->adv_name_})
    this->add_cand_(*n, 3, 0xA5);
  for (std::string *n : {&this->gap_name_, &this->adv_name_})
    this->add_cand_(*n, 0, 0xA5);
  for (std::string *n : {&this->gap_name_, &this->adv_name_})
    this->add_cand_(*n, 0, 0xB5);
  for (std::string *n : {&this->adv_alt_name_, &this->cfg_name_}) {
    this->add_cand_(*n, 3, 0xA5);
    this->add_cand_(ascii_tail(*n), 3, 0xA5);
    this->add_cand_(*n, 0, 0xA5);
  }
  for (std::string *n : {&this->gap_name_, &this->adv_name_}) {
    this->add_cand_(*n, 1, 0xA5);
    this->add_cand_(*n, 2, 0xA5);
  }
  if (this->cands_.empty())
    this->cands_.push_back({"", 3, 0xA5});
}

// --- отправка кадров ---

void NinebotHub::send_plain_(const std::vector<uint8_t> &plain, bool sn) {
  uint32_t cnt = sn ? ++this->counter_ : 0;
  auto wire = this->crypto_.encrypt(plain, cnt);
  if (!sn)
    this->last_tx_ = wire;
  ESP_LOGV(TAG, "-> %s", hex_str(wire.data(), wire.size()).c_str());
  // Режем на куски по 20 байт (MTU-3 при MTU 23).
  for (size_t off = 0; off < wire.size(); off += 20)
    this->tx_.emplace_back(wire.begin() + off, wire.begin() + std::min(wire.size(), off + 20));
  this->next_tx_at_ = millis();
  this->sent_at_ = millis();
}

void NinebotHub::pump_tx_() {
  if (this->write_busy_ || this->tx_.empty() || this->next_tx_at_ == 0)
    return;
  if ((int32_t) (millis() - this->next_tx_at_) < 0)
    return;
  auto frag = this->tx_.front();
  this->tx_.erase(this->tx_.begin());
  // без подтверждения, если характеристика умеет: так пишет и приложение
  bool rsp = !this->write_no_rsp_;
  auto type = rsp ? ESP_GATT_WRITE_TYPE_RSP : ESP_GATT_WRITE_TYPE_NO_RSP;
  if (rsp)
    this->write_busy_ = true;
  esp_err_t e = esp_ble_gattc_write_char(this->parent()->get_gattc_if(), this->parent()->get_conn_id(),
                                         this->write_handle_, frag.size(), frag.data(), type, ESP_GATT_AUTH_REQ_NONE);
  if (e != ESP_OK) {
    NBLOG("write_char rc=0x%X", (unsigned) e);
    this->write_busy_ = false;
  }
  this->next_tx_at_ = this->tx_.empty() ? 0 : millis() + FRAG_GAP_MS;
}

// --- сборка кадров из нотификаций ---

void NinebotHub::on_bytes_(const uint8_t *d, size_t n) {
  for (size_t i = 0; i < n; i++) {
    uint8_t b = d[i];
    size_t sz = this->rx_.size();
    if (sz == 0) {
      if (b == 0x5A)
        this->rx_.push_back(b);
    } else if (sz == 1) {
      if (b == 0xA5 || b == 0xB5)
        this->rx_.push_back(b);
      else
        this->rx_.clear();
    } else {
      this->rx_.push_back(b);
      if (this->rx_.size() >= 3) {
        size_t total = this->rx_[2] + 13;  // Encryption2: LEN + 13 на проводе
        if (this->rx_.size() >= total) {
          this->on_frame_(this->rx_.data(), total);
          this->rx_.erase(this->rx_.begin(), this->rx_.begin() + total);
        }
      }
    }
    if (this->rx_.size() > 512)
      this->rx_.clear();
  }
}

void NinebotHub::on_frame_(const uint8_t *wire, size_t n) {
  ESP_LOGV(TAG, "<- %s", hex_str(wire, n).c_str());
  if (this->phase_ == PH_PRECOMM) {
    this->on_precomm_(wire, n);
    return;
  }
  std::vector<uint8_t> p;
  uint16_t cnt;
  int rc = this->crypto_.decrypt(wire, n, p, &cnt);
  if (rc != nbcrypto::OK && this->phase_ == PH_SETPWD) {
    // G3 после кнопки отвечает на SET_PWD уже ключом нового пароля, а не имени.
    // 28.10.2026 такой ответ не сошелся ни с одним ключом до пароля.
    this->crypto_.set_key(this->pwd_, 16, this->auth_);
    rc = this->crypto_.decrypt(wire, n, p, &cnt);
    if (rc == nbcrypto::OK) {
      NBLOG("ответ SET_PWD ключом нового пароля: %s", hex_str(p.data(), p.size()).c_str());
    } else {
      const std::string &nm = this->cands_[this->cand_i_].name;
      this->crypto_.set_key((const uint8_t *) nm.data(), nm.size(), this->auth_);
    }
  }
  if (rc != nbcrypto::OK) {
    NBLOG("дешифровка rc=%d на шаге %u: %s", rc, this->phase_, hex_str(wire, std::min(n, (size_t) 24)).c_str());
    // на шаге пароля любой ответ приходит только после кнопки: пробуем AUTH, он и рассудит
    if (this->phase_ == PH_SETPWD && this->pairing_) {
      NBLOG("пробую AUTH с новым паролем");
      this->send_auth_();
    }
    return;
  }
  if (p.size() < 7)
    return;
  // p: 5A A5 LEN board 3E? cmd index ... На ответах байт 3 это плата-источник,
  // 4 это 0x3E, 5 cmd, 6 index. Берем cmd/index с конца заголовка.
  uint8_t cmd = p[5], index = p[6];
  const uint8_t *data = p.data() + 7;
  size_t dlen = p.size() > 7 ? p.size() - 7 : 0;

  switch (this->phase_) {
    case PH_SETPWD:
      // INDEX 1 принято, 0 ждем кнопку
      if (cmd == CMD_WRITE_RESP || index == 0x01) {
        if (index == 0x01) {
          ESP_LOGI(TAG, "пароль принят, перехожу к AUTH");
          this->send_auth_();
        }  // index 0: ждем, шаг повторится по таймауту (нажатие кнопки)
      }
      break;
    case PH_AUTH:
      if (index == 0x01) {
        ESP_LOGI(TAG, "аутентификация пройдена");
        this->session_ok_ = true;
        if (this->pwd_new_) {
          memcpy(this->st_.pwd, this->pwd_, 16);
          this->st_.has_pwd = 1;
          this->save_();
          this->publish_bin_(BINARY_PAIRED, true);
          char ph[33];
          for (int i = 0; i < 16; i++)
            snprintf(ph + i * 2, 3, "%02x", this->pwd_[i]);
          ESP_LOGI(TAG, "сопряжение сохранено. Пароль (для переноса на другие платы): %s", ph);
          if (this->texts_[TEXT_PASSWORD])
            this->texts_[TEXT_PASSWORD]->publish_state(ph);
        }
        this->pairing_ = false;
        this->start_query_();
      } else {
        // не прошло: если сохраненный пароль устарел, нужна кнопка
        this->auth_tries_++;
        if (this->st_.has_pwd && !this->pwd_new_) {
          NBLOG("сохраненный пароль отвергнут, нужно сопряжение");
          this->st_.has_pwd = 0;
          this->save_();
          this->publish_bin_(BINARY_PAIRED, false);
          this->status_("пароль устарел, нажми Pair");
          this->close_("пароль устарел", false);
        } else if (this->auth_tries_ < 5) {
          this->send_setpwd_();
        } else {
          this->close_("AUTH не прошел", false);
        }
      }
      break;
    case PH_QUERY:
      if (cmd == CMD_READ_RESP || cmd == CMD_WRITE_RESP) {
        this->read_pending_ = false;
        this->on_read_(p);
      }
      break;
    default:
      break;
  }
}

// --- рукопожатие ---

void NinebotHub::send_precomm_() {
  const Cand &c = this->cands_[this->cand_i_];
  this->crypto_.set_precomm_variant((const uint8_t *) c.name.data(), c.name.size(), c.var);
  this->phase_ = PH_PRECOMM;
  this->deadline_ = millis() + 3 * STEP_TIMEOUT_MS;  // срок на каждого кандидата, а не на весь сеанс
  auto f = nbcrypto::build_frame(T_BLE, CMD_PRECOMM, 0x00, nullptr, 0, c.sync2);
  this->send_plain_(f, false);
  if (this->sends_ == 0)
    NBLOG("PRE_COMM #%u/%u '%s' v%u: %s", (unsigned) this->cand_i_ + 1, (unsigned) this->cands_.size(),
          c.name.c_str(), c.var, hex_str(this->last_tx_.data(), this->last_tx_.size()).c_str());
}

void NinebotHub::on_precomm_(const uint8_t *wire, size_t n) {
  if (n == this->last_tx_.size() && memcmp(wire, this->last_tx_.data(), n) == 0) {
    NBLOG("эхо PRE_COMM #%u: ключ отвергнут", (unsigned) this->cand_i_ + 1);
    return;  // дальше по таймеру шага
  }
  std::vector<uint8_t> p;
  uint16_t cnt;
  int rc = this->crypto_.decrypt(wire, n, p, &cnt);
  if (rc != nbcrypto::OK || p.size() < 7 + 30) {
    // не сдвигаем: поздний ответ на прошлого кандидата не должен сбить перебор, hex разберем на ПК
    NBLOG("PRE_COMM ответ %u б rc=%d при #%u: %s", (unsigned) n, rc, (unsigned) this->cand_i_ + 1,
          hex_str(wire, std::min(n, (size_t) 24)).c_str());
    return;
  }
  const Cand &c = this->cands_[this->cand_i_];
  NBLOG("PRE_COMM ок: '%s' v%u %02X", c.name.c_str(), c.var, c.sync2);
  this->sync2_ = c.sync2;
  this->st_.variant = c.var;
  this->st_.mac = this->address_;
  memset(this->st_.name, 0, sizeof(this->st_.name));
  strncpy(this->st_.name, c.name.c_str(), sizeof(this->st_.name) - 1);
  this->deadline_ = millis() + (this->pairing_ ? 60000 : SESSION_TIMEOUT_MS);
  // payload: auth[16] serial[14], плюс INDEX говорит, есть ли сохраненный пароль
  const uint8_t *data = p.data() + 7;
  memcpy(this->auth_, data, 16);
  memcpy(this->serial_, data + 16, 14);
  this->serial_[14] = 0;
  this->crypto_.set_auth(this->auth_);
  uint8_t index = p[6];
  ESP_LOGI(TAG, "PRE_COMM ок, serial %s, stored_pwd_flag=%u", this->serial_, index);
  if (this->texts_[TEXT_SERIAL])
    this->texts_[TEXT_SERIAL]->publish_state(this->serial_);

  this->counter_ = 1;  // start_sn: первый SN-кадр будет со счетчиком 2
  if (this->st_.has_pwd && !this->pairing_) {
    // переподключение: сразу AUTH с сохраненным паролем
    memcpy(this->pwd_, this->st_.pwd, 16);
    this->pwd_new_ = false;
    this->counter_ = 1;  // AUTH уйдет со счетчиком 2
    this->send_auth_();
  } else {
    this->send_setpwd_();
  }
}

void NinebotHub::send_setpwd_() {
  // новый пароль из auth_param и времени, java.util.Random + SHA-256
  if (!this->pwd_new_ || this->phase_ != PH_SETPWD) {
    int64_t seed_add = 0;
    for (int i = 0; i < 16; i++) {
      int8_t sb = (int8_t) this->auth_[i];
      seed_add += (int64_t) ((int32_t) sb << (((i % 8) * 8) & 31));
    }
    int64_t seed = (int64_t) (millis()) + seed_add;  // на плате нет currentTimeMillis, millis хватает
    JavaRandom rng(seed);
    uint8_t rnd[16];
    rng.next_bytes(rnd, 16);
    uint8_t h[32];
    nbcrypto::Crypto::sha256(rnd, 16, h);
    memcpy(this->pwd_, h, 16);
    this->pwd_new_ = true;
  }
  const std::string &nm = this->cands_[this->cand_i_].name;
  this->crypto_.set_key((const uint8_t *) nm.data(), nm.size(), this->auth_);
  this->phase_ = PH_SETPWD;
  auto f = nbcrypto::build_frame(T_BLE, CMD_SETPWD, 0x00, this->pwd_, 16, this->sync2_);
  this->send_plain_(f, true);
  this->status_(this->pairing_ ? "нажми кнопку питания на самокате" : "ставлю пароль");
}

void NinebotHub::send_auth_() {
  this->crypto_.set_key(this->pwd_, 16, this->auth_);
  this->phase_ = PH_AUTH;
  auto f = nbcrypto::build_frame(T_BLE, CMD_AUTH, 0x00, (const uint8_t *) this->serial_, 14, this->sync2_);
  this->send_plain_(f, true);
}

// --- обход регистров ---

void NinebotHub::start_query_() {
  this->phase_ = PH_QUERY;
  this->read_idx_ = 0;
  this->read_pending_ = false;
  this->deadline_ = millis() + SESSION_TIMEOUT_MS;
  // ручные операции из HA вперед
  this->ops_run_ = this->ops_;
  this->ops_.clear();
  this->send_read_();
}

uint8_t NinebotHub::board_for_(uint8_t group) {
  if (group == 0)
    return this->st_.vcu ? this->st_.vcu : VCU_CANDS[this->cand_idx_ % (sizeof(VCU_CANDS))];
  return this->st_.bms ? this->st_.bms : BMS_CANDS[this->cand_idx_ % (sizeof(BMS_CANDS))];
}
bool NinebotHub::board_known_(uint8_t group) { return group == 0 ? this->st_.vcu != 0 : this->st_.bms != 0; }

void NinebotHub::send_read_() {
  // сперва ручные операции
  if (!this->ops_run_.empty()) {
    auto op = this->ops_run_.front();
    this->ops_run_.erase(this->ops_run_.begin());
    uint8_t cmd = op.cmd;
    auto f = nbcrypto::build_frame(op.board, cmd, op.index, op.data.empty() ? nullptr : op.data.data(), op.data.size(),
                                   this->sync2_);
    this->send_plain_(f, true);
    this->noreply_ = (cmd == CMD_WRITE_NR);
    this->read_pending_ = !this->noreply_;
    this->deadline_ = millis() + SESSION_TIMEOUT_MS;
    return;
  }
  size_t total = sizeof(REGS) / sizeof(REGS[0]);
  if (this->read_idx_ >= total) {
    this->close_("обход завершен", false);
    return;
  }
  const Reg &r = REGS[this->read_idx_];
  // BMS спит: не долбим его регистры, если первый не ответил
  if (r.group == 1 && this->bms_dead_) {
    this->read_idx_++;
    this->send_read_();
    return;
  }
  if (r.group == 0 && this->vcu_dead_) {
    this->read_idx_++;
    this->send_read_();
    return;
  }
  uint8_t len = r.len;
  auto f = nbcrypto::build_frame(this->board_for_(r.group), CMD_READ, r.index, &len, 1, this->sync2_);
  this->send_plain_(f, true);
  this->read_pending_ = true;
}

void NinebotHub::next_read_(bool answered) {
  const Reg &r = REGS[std::min(this->read_idx_, sizeof(REGS) / sizeof(REGS[0]) - 1)];
  if (!answered && this->ops_run_.empty()) {
    // первый регистр группы не ответил: может, адрес платы другой
    if (!this->board_known_(r.group) && this->read_idx_ == 0 && r.group == 0) {
      this->cand_idx_++;
      if (this->cand_idx_ < sizeof(VCU_CANDS)) {
        this->send_read_();
        return;
      }
      this->cand_idx_ = 0;
      this->vcu_dead_ = true;
    } else if (r.group == 1 && !this->bms_ok_) {
      // bms молчит: спит (самокат не на зарядке)
      this->bms_dead_ = true;
      this->publish_bin_(BINARY_BMS_AWAKE, false);
    }
  }
  if (!this->ops_run_.empty()) {
    this->send_read_();
    return;
  }
  this->read_idx_++;
  this->next_tx_at_ = millis() + 40;
  this->set_timeout("nextread", 40, [this]() {
    if (this->phase_ == PH_QUERY)
      this->send_read_();
  });
}

void NinebotHub::on_read_(const std::vector<uint8_t> &p) {
  if (p.size() < 7)
    return;
  uint8_t index = p[6];
  const uint8_t *d = p.data() + 7;
  size_t dlen = p.size() > 7 ? p.size() - 7 : 0;
  if (!this->ops_run_.empty()) {
    // ответ на ручную операцию: в лог
    char b[80];
    snprintf(b, sizeof b, "idx %02X: %s", index, hex_str(d, std::min(dlen, (size_t) 20)).c_str());
    this->status_(b);
    ESP_LOGI(TAG, "ручной ответ %s", b);
    this->next_read_(true);
    return;
  }
  const Reg &r = REGS[std::min(this->read_idx_, sizeof(REGS) / sizeof(REGS[0]) - 1)];
  if (r.group == 0 && !this->board_known_(0)) {
    this->st_.vcu = this->board_for_(0);
    this->save_();
    this->vcu_ok_ = true;
  }
  if (r.group == 1 && !this->board_known_(1)) {
    this->st_.bms = this->board_for_(1);
    this->save_();
    this->bms_ok_ = true;
    this->publish_bin_(BINARY_BMS_AWAKE, true);
  }
  this->parse_value_(r.group, r.index, d, dlen);
  this->next_read_(true);
}

void NinebotHub::parse_value_(uint8_t group, uint8_t index, const uint8_t *d, size_t n) {
  const Reg &r = REGS[std::min(this->read_idx_, sizeof(REGS) / sizeof(REGS[0]) - 1)];
  // Масштабы части регистров G3 не подтверждены на живом самокате: публикуем
  // сырое число, калибровку уточнить по первым данным (см. PROTOCOL.md).
  switch (r.kind) {
    case SENSOR_BATTERY:
    case SENSOR_BMS_SOC:
      if (n >= 2)
        this->publish_(r.kind, u16(d));
      break;
    case SENSOR_RANGE:
    case SENSOR_TIME_FULL:
    case SENSOR_REMAIN_CAP:
      if (n >= 2)
        this->publish_(r.kind, u16(d));
      break;
    case SENSOR_ODOMETER:
      if (n >= 4)
        this->publish_(r.kind, u32(d) / 1000.0f);  // 0.001 км по E-серии
      break;
    case SENSOR_BODY_TEMP:
    case SENSOR_BMS_TEMP:
      if (n >= 2)
        this->publish_(r.kind, (int16_t) u16(d) / 10.0f);
      break;
    case SENSOR_ERROR:
    case SENSOR_WARNING:
    case SENSOR_CYCLES:
      if (n >= 2)
        this->publish_(r.kind, u16(d));
      break;
    case SENSOR_VOLTAGE:
      if (n >= 2)
        this->publish_(r.kind, u16(d) / 100.0f);  // сотые вольта, уточнить
      break;
    case SENSOR_CURRENT:
      if (n >= 2)
        this->publish_(r.kind, (int16_t) u16(d) / 100.0f);
      break;
    case SENSOR_CELL_MIN: {
      // пачка банок по 2 байта мВ
      if (n < 2)
        break;
      int lo = 0xFFFF, hi = 0;
      size_t cells = n / 2;
      for (size_t i = 0; i < cells; i++) {
        int mv = u16(d + i * 2);
        if (mv < 1000 || mv > 5000)
          continue;
        lo = std::min(lo, mv);
        hi = std::max(hi, mv);
      }
      if (hi > 0) {
        this->publish_(SENSOR_CELL_MIN, lo / 1000.0f);
        this->publish_(SENSOR_CELL_MAX, hi / 1000.0f);
        this->publish_(SENSOR_CELL_DELTA, (hi - lo) / 1000.0f);
      }
      break;
    }
    default:
      break;
  }
}

void NinebotHub::finish_query_() {
  this->sessions_++;
  if (this->rssi_valid_)
    this->publish_(SENSOR_RSSI, this->rssi_);
  if (this->session_ok_)
    this->status_(this->bms_dead_ ? "ок, BMS спит" : "ок");
}

// --- управление из HA ---

void NinebotHub::poll_now() {
  if (this->phase_ == PH_IDLE)
    this->update();
  else
    this->poll_after_ = true;
}

void NinebotHub::start_pairing() {
  this->pairing_ = true;
  this->pwd_new_ = false;
  this->pairing_until_ = millis() + PAIR_WINDOW_MS;
  this->st_.has_pwd = 0;  // заставим пройти SET_PWD
  this->status_("сопряжение: нажми кнопку питания");
  if (this->phase_ == PH_IDLE)
    this->update();
}

void NinebotHub::forget_pairing() {
  memset(this->st_.pwd, 0, 16);
  this->st_.has_pwd = 0;
  this->save_();
  this->publish_bin_(BINARY_PAIRED, false);
  this->status_("сопряжение сброшено");
}

void NinebotHub::queue_read(int board, int index, int len) {
  Op op;
  op.board = board;
  op.cmd = CMD_READ;
  op.index = index;
  op.data.push_back(len);
  this->ops_.push_back(op);
  this->poll_now();
}

void NinebotHub::queue_write(int board, int index, int value, int len, bool no_reply) {
  Op op;
  op.board = board;
  op.cmd = no_reply ? CMD_WRITE_NR : CMD_WRITE;
  op.index = index;
  for (int i = 0; i < len; i++)
    op.data.push_back((value >> (8 * i)) & 0xFF);  // little-endian
  this->ops_.push_back(op);
  this->poll_now();
}

void NinebotHub::set_paused(bool paused) {
  this->paused_ = paused;
  if (paused && this->phase_ != PH_IDLE && this->phase_ != PH_CLOSE)
    this->close_("пауза (охрана)", false);
}

void NinebotHub::restart_ble_(const char *why) {
  NBLOG("перезапуск BLE: %s", why);
  this->phase_ = PH_IDLE;
  this->ble_restarting_ = true;
  esp32_ble::global_ble->disable();
  this->set_timeout("ble_up", BLE_OFF_MS, [this]() {
    esp32_ble::global_ble->enable();
    this->set_timeout("ble_settle", BLE_SETTLE_MS, [this]() { this->ble_restarting_ = false; });
  });
}

void NinebotHub::status_(const std::string &s) {
  this->log_ = s;
  if (this->texts_[TEXT_STATUS])
    this->texts_[TEXT_STATUS]->publish_state(s);
}
void NinebotHub::publish_(uint8_t kind, float v) {
  if (this->sensors_[kind])
    this->sensors_[kind]->publish_state(v);
}
void NinebotHub::publish_bin_(uint8_t kind, bool v) {
  if (this->binaries_[kind])
    this->binaries_[kind]->publish_state(v);
}

}  // namespace esphome::ninebot

#endif
