#pragma once

#include "esphome/core/component.h"
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

namespace esphome::battair {

namespace espbt = esphome::esp32_ble_tracker;

// Датчики ISDT BattAir на балансировочном разъеме. Протокол снят с Arduino-прошивки
// BattAirMonitor v2.2 (D:\MegaSync\Arduino\BattAirMonitor_v2_2\BattAirClass.cpp).
// Сервис 0xBA00, характеристика 0xBA01: запись без ответа и notify.
// Каждый байт в обе стороны ксорится с последним байтом MAC датчика.
// После подписки шлется рукопожатие на 11 байт, потом команды по 4 байта,
// ответ приходит с кодом команда+1.
static const uint16_t BATTAIR_SERVICE_UUID = 0xBA00;
static const uint16_t BATTAIR_CHAR_UUID = 0xBA01;

enum SensorKind : uint8_t {
  SENSOR_VOLTAGE = 0,
  SENSOR_CELL_1,
  SENSOR_CELL_2,
  SENSOR_CELL_3,
  SENSOR_CELL_4,
  SENSOR_CELL_DELTA,
  SENSOR_CYCLES,
  SENSOR_TEMPERATURE,
  SENSOR_CHARGE_CURRENT,
  SENSOR_DISCHARGE_CURRENT,
  SENSOR_LEVEL,
  SENSOR_RSSI,
  SENSOR_ERRORS,
  SENSOR_OUTAGES,
  SENSOR_OVERHEATS,
  SENSOR_OVERCHARGES,
  SENSOR_OVERDISCHARGES,
  SENSOR_COUNT,
};

enum TextKind : uint8_t {
  TEXT_LABEL = 0,
  TEXT_TYPE,
  TEXT_FIRMWARE,
  TEXT_COUNT,
};

// Что удалось вытащить за один сеанс связи с датчиком.
struct Session {
  bool hw{false};
  bool cells{false};
  bool stats{false};
  bool realtime{false};
  bool level{false};
  uint8_t battery_type{0xFF};
  uint8_t cell_count{4};
  uint8_t hw_major{0}, hw_minor{0}, sw_major{0}, sw_minor{0};
  uint8_t production[4]{};
  std::string maker;
  int32_t cell_mv[6]{};
  uint16_t cycles{0};
  uint16_t errors{0};
  uint8_t outages{0};
  uint16_t overheats{0};
  uint16_t overcharges{0};
  uint16_t overdischarges{0};
  int8_t temperature{0};
  uint32_t charge_current{0};
  uint32_t discharge_current{0};
  uint8_t level_raw{0};
  std::string name;
};

struct Battery {
  std::string ble_name;  // имя из эфира без префикса BATAIR, по нему ищем MAC
  uint64_t address{0};
  bool address_from_config{false};
  uint8_t address_type{BLE_ADDR_TYPE_PUBLIC};
  int rssi{0};
  bool rssi_valid{false};
  uint32_t last_adv_log{0};
  uint8_t attempt{0};
  bool ok{false};
  Session s;
  // Фильтр мусора: последнее принятое и ждущее подтверждения напряжения банок, циклы.
  int32_t accepted_mv[6]{};
  bool have_accepted{false};
  int32_t pending_mv[6]{};
  bool have_pending{false};
  uint8_t pending_count{0};
  int last_cycles{-1};
  sensor::Sensor *sensors[SENSOR_COUNT]{};
  text_sensor::TextSensor *texts[TEXT_COUNT]{};
  binary_sensor::BinarySensor *online{nullptr};
};

class BattAirHub : public PollingComponent,
                   public ble_client::BLEClientNode,
                   public espbt::ESPBTDeviceListener
#ifdef USE_OTA_STATE_LISTENER
    ,
                   public ota::OTAGlobalStateListener
#endif
{
 public:
#ifdef USE_OTA_STATE_LISTENER
  // Трекер на старте OTA рвет BLE-соединения. Без этого обход принял бы обрыв за сбой
  // датчика и полез подключаться заново посреди прошивки: так OTA и срывалось 08.10.2026.
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

  void add_battery(const std::string &ble_name, uint64_t address);
  void set_sensor(uint8_t bat, uint8_t kind, sensor::Sensor *s) { this->bats_[bat].sensors[kind] = s; }
  void set_text(uint8_t bat, uint8_t kind, text_sensor::TextSensor *t) { this->bats_[bat].texts[kind] = t; }
  void set_online(uint8_t bat, binary_sensor::BinarySensor *b) { this->bats_[bat].online = b; }
  void set_found_sensor(sensor::Sensor *s) { this->found_sensor_ = s; }
  void set_counter_sensor(sensor::Sensor *s) { this->counter_sensor_ = s; }
  void set_log_sensor(text_sensor::TextSensor *t) { this->log_sensor_ = t; }
  void set_attempts(uint8_t n) { this->attempts_ = n; }

  // Опросить сейчас, не дожидаясь update_interval (кнопка в HA).
  void poll_now() { this->update(); }

 protected:
  enum Phase : uint8_t { PH_IDLE, PH_WAIT, PH_CONNECT, PH_SETUP, PH_QUERY, PH_CLOSE };

  std::vector<Battery> bats_;
  int cur_{-1};
  Phase phase_{PH_IDLE};
  uint16_t handle_{0};
  uint32_t deadline_{0};
  uint32_t wait_until_{0};
  uint32_t setup_at_{0};
  bool handshake_sent_{false};
  uint32_t handshake_at_{0};
  size_t cmd_idx_{0};
  bool cmd_pending_{false};
  uint32_t cmd_at_{0};
  uint32_t next_cmd_at_{0};
  uint8_t attempts_{3};
  bool ota_active_{false};
  bool tx_power_set_{false};
  uint16_t counter_{0};
  uint8_t found_{0};
  std::string log_;
  std::vector<uint64_t> unknown_logged_;

  sensor::Sensor *found_sensor_{nullptr};
  sensor::Sensor *counter_sensor_{nullptr};
  text_sensor::TextSensor *log_sensor_{nullptr};

  Battery &bat_() { return this->bats_[this->cur_]; }
  uint8_t key_() { return this->bat_().address & 0xFF; }

  void next_battery_();
  void start_attempt_();
  void close_(bool ok);
  void after_close_();
  void finish_cycle_();
  void send_handshake_();
  void send_command_();
  void write_(const uint8_t *data, size_t len);
  void on_packet_(const uint8_t *raw, size_t len);
  void publish_battery_(Battery &b);
  bool confirms_pending_(Battery &b);
};

}  // namespace esphome::battair

#endif
