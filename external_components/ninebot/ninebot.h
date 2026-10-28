#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <string>
#include <vector>

#ifdef USE_OTA_STATE_LISTENER
#include "esphome/components/ota/ota_backend.h"
#endif

#ifdef USE_ESP32

#include <esp_gattc_api.h>

#include "nb_crypto.h"

namespace esphome::ninebot {

namespace espbt = esphome::esp32_ble_tracker;

// Самокат Segway-Ninebot по BLE, протокол Encryption2. Разбор протокола:
// C:\Users\Alex\Claude\Ninebot\research\PROTOCOL.md и документация NootNooot
// (research/repos/segway-ninebot-ble/docs). Таблица регистров MAX G3:
// docs/devices/segway-ekickscooter-ninebot-max-g3max-g3-plus.md.
//
// Сеанс: подключение, подписка на 0004, имя из GAP 0x2A00 (оно ключ первой
// фазы), PRE_COMM, при надобности SET_PWD, AUTH, чтения, отключение.
// Пароль сессии самокат хранит один. ESP придумывает его сам при сопряжении
// (кнопка Pair в HA плюс кнопка питания на самокате) и держит во флеше, дальше
// подключается без кнопки. Телефон, сопряженный после ESP, этот пароль
// перетирает, тогда статус скажет, что нужно сопрягаться снова.
static const char *const SERVICE_UUID = "6e400001-0000-0000-006e-696e65626f74";
static const char *const WRITE_UUID = "6e400002-0000-0000-006e-696e65626f74";
static const char *const NOTIFY_UUID = "6e400004-0000-0000-006e-696e65626f74";

enum SensorKind : uint8_t {
  SENSOR_BATTERY = 0,     // vcu 0x55, общий заряд, %
  SENSOR_RANGE,           // vcu 0x5F, остаток хода
  SENSOR_ODOMETER,        // vcu 0x62, пробег
  SENSOR_BODY_TEMP,       // vcu 0x6B
  SENSOR_ERROR,           // vcu 0x58
  SENSOR_WARNING,         // vcu 0x59
  SENSOR_BMS_SOC,         // bms 0x8F, заряд пакета, %
  SENSOR_VOLTAGE,         // bms 0x8C, или сумма банок
  SENSOR_CURRENT,         // bms 0x8D
  SENSOR_BMS_TEMP,        // bms 0x96
  SENSOR_CELL_MIN,        // bms 0xA0, банки
  SENSOR_CELL_MAX,
  SENSOR_CELL_DELTA,
  SENSOR_CYCLES,          // bms 0x59
  SENSOR_REMAIN_CAP,      // bms 0x8A
  SENSOR_TIME_FULL,       // bms 0x94
  SENSOR_RSSI,
  SENSOR_COUNT,
};

enum TextKind : uint8_t {
  TEXT_STATUS = 0,
  TEXT_SERIAL,
  TEXT_FIRMWARE,
  TEXT_LOG,
  TEXT_PASSWORD,  // пароль сессии hex, чтобы перенести ключ на другую плату
  TEXT_COUNT,
};

enum BinaryKind : uint8_t {
  BINARY_ONLINE = 0,
  BINARY_PAIRED,
  BINARY_BMS_AWAKE,
  BINARY_COUNT,
};

// Что хранится во флеше. Номер версии в ключе: при смене формата старое не читается.
struct Stored {
  uint32_t magic;
  uint64_t mac;
  uint8_t variant;   // вариант ключа PRE_COMM, 0xFF не известен
  uint8_t vcu;       // адреса плат, 0 не известен
  uint8_t bms;
  uint8_t has_pwd;
  uint8_t pwd[16];
  char name[32];     // BLE-имя из GAP
} __attribute__((packed));

// Ручная команда из HA: чтение или запись регистра, ответ в лог.
struct Op {
  uint8_t board;
  uint8_t cmd;
  uint8_t index;
  std::vector<uint8_t> data;
};

class NinebotHub : public PollingComponent,
                   public ble_client::BLEClientNode,
                   public espbt::ESPBTDeviceListener
#ifdef USE_OTA_STATE_LISTENER
    ,
                   public ota::OTAGlobalStateListener
#endif
{
 public:
#ifdef USE_OTA_STATE_LISTENER
  void on_ota_global_state(ota::OTAState state, float progress, uint8_t error,
                           ota::OTAComponent *component) override;
#endif

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  bool parse_device(const espbt::ESPBTDevice &device) override;
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

  void set_mac(uint64_t mac) { this->cfg_mac_ = mac; }
  void set_mac_suffix(uint16_t suffix) { this->mac_suffix_ = suffix; this->has_suffix_ = true; }
  void set_ble_name(const std::string &name) { this->cfg_name_ = name; }
  // Готовый пароль сессии (32 hex): плата зайдет по нему без кнопки.
  void set_password(const std::string &hex) { this->cfg_pwd_hex_ = hex; }
  void set_sensor(uint8_t kind, sensor::Sensor *s) { this->sensors_[kind] = s; }
  void set_text(uint8_t kind, text_sensor::TextSensor *t) { this->texts_[kind] = t; }
  void set_binary(uint8_t kind, binary_sensor::BinarySensor *b) { this->binaries_[kind] = b; }

  // Кнопки и действия из HA.
  void poll_now();
  void start_pairing();
  void forget_pairing();
  void queue_read(int board, int index, int len);
  void queue_write(int board, int index, int value, int len, bool no_reply);
  // На время отсчета охраны сеансы не нужны: эфир отдан меткам снятия.
  void set_paused(bool paused);

 protected:
  enum Phase : uint8_t {
    PH_IDLE,
    PH_WAIT,
    PH_CONNECT,
    PH_SETUP,
    PH_NAME,
    PH_PRECOMM,
    PH_SETPWD,
    PH_AUTH,
    PH_QUERY,
    PH_CLOSE,
  };

  // Одно чтение из обхода: группа платы, регистр, длина, куда разбирать.
  struct Read {
    uint8_t group;  // 0 vcu, 1 bms, 2 ручная команда
    uint8_t index;
    uint8_t len;
  };

  // настройки
  uint64_t cfg_mac_{0};
  uint16_t mac_suffix_{0};
  bool has_suffix_{false};
  std::string cfg_name_;
  std::string cfg_pwd_hex_;

  // что знаем о самокате
  uint64_t address_{0};
  uint8_t address_type_{BLE_ADDR_TYPE_PUBLIC};
  std::string adv_name_;
  std::string gap_name_;  // имя из GAP 0x2A00, может отличаться от рекламы
  int rssi_{0};
  bool rssi_valid_{false};
  uint32_t last_seen_{0};
  Stored st_{};
  ESPPreferenceObject pref_;

  // сеанс
  Phase phase_{PH_IDLE};
  uint8_t attempt_{0};
  uint32_t deadline_{0};
  uint32_t wait_until_{0};
  uint16_t write_handle_{0};
  uint16_t notify_handle_{0};
  uint16_t name_handle_{0};
  bool write_no_rsp_{true};
  bool write_busy_{false};
  uint32_t setup_at_{0};
  std::vector<std::vector<uint8_t>> tx_;
  uint32_t next_tx_at_{0};
  std::vector<uint8_t> rx_;
  nbcrypto::Crypto crypto_;
  uint32_t counter_{0};
  uint8_t auth_[16]{};
  uint8_t pwd_[16]{};       // пароль этого сеанса: сохраненный или новый при сопряжении
  bool pwd_new_{false};
  uint8_t auth_tries_{0};
  char serial_[15]{};
  std::vector<std::string> names_;   // кандидаты имени для ключа PRE_COMM
  uint8_t try_no_{0};                // номер попытки PRE_COMM: имя и вариант
  uint32_t sent_at_{0};
  uint8_t sends_{0};
  bool pairing_{false};
  uint32_t pairing_until_{0};
  bool pair_after_{false};
  bool session_ok_{false};
  bool paused_{false};
  bool ota_active_{false};
  bool ble_restarting_{false};
  bool tx_power_set_{false};
  uint16_t sessions_{0};

  // обход регистров
  std::vector<Read> reads_;
  size_t read_idx_{0};
  uint8_t cand_idx_{0};   // какой адрес платы пробуем, пока он не известен
  bool read_pending_{false};
  bool vcu_ok_{false};
  bool bms_ok_{false};
  bool bms_dead_{false};
  bool vcu_dead_{false};
  bool noreply_{false};     // ручная запись без ответа: ждем паузу и идем дальше
  bool retry_{false};       // закрыли из-за связи, а не из-за протокола: можно повторить
  bool poll_after_{false};
  std::vector<Op> ops_;
  std::vector<Op> ops_run_;
  bool serial_done_{false};
  // банки и напряжение пакета из сеанса, для выбора единиц напряжения
  float cells_sum_{NAN};
  uint32_t volt_raw_{0};
  bool volt_have_{false};

  std::string log_;

  sensor::Sensor *sensors_[SENSOR_COUNT]{};
  text_sensor::TextSensor *texts_[TEXT_COUNT]{};
  binary_sensor::BinarySensor *binaries_[BINARY_COUNT]{};

  void start_attempt_();
  void close_(const char *why, bool retry);
  void after_close_();
  void status_(const std::string &s);
  void publish_(uint8_t kind, float v);
  void publish_bin_(uint8_t kind, bool v);
  void save_();

  void send_plain_(const std::vector<uint8_t> &plain, bool sn);
  void pump_tx_();
  void on_bytes_(const uint8_t *d, size_t n);
  void on_frame_(const uint8_t *wire, size_t n);

  void send_precomm_();
  void on_precomm_(const uint8_t *wire, size_t n);
  void send_setpwd_();
  void send_auth_();
  void start_query_();
  void send_read_();
  void next_read_(bool answered);
  void on_read_(const std::vector<uint8_t> &p);
  void parse_value_(uint8_t group, uint8_t index, const uint8_t *d, size_t n);
  uint8_t board_for_(uint8_t group);
  bool board_known_(uint8_t group);
  void finish_query_();
  void restart_ble_(const char *why);
};

}  // namespace esphome::ninebot

#endif
