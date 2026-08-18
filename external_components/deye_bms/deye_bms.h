#pragma once

#include "esphome/core/component.h"
#include "esphome/components/ble_client/ble_client.h"
#include "esphome/components/sensor/sensor.h"

#include <vector>
#include <string>

#ifdef USE_ESP32

#include <esp_gattc_api.h>

namespace esphome {
namespace deye_bms {

namespace espbt = esphome::esp32_ble_tracker;

static const uint16_t DEYE_SERVICE_UUID = 0xFFF1;
static const uint16_t DEYE_CHAR_UUID = 0xFFF2;

// Реверс протокола батареи Deye SE-F12-C (LVESS15). См. memory:
// project_deye_battery_ble_reverse. Ключ сессии = SHA256(bleName + random)[:16],
// AES-128-CBC, IV="0000000000000000", Pkcs7. Кадры AA55 <addr> <func> ... CRC 0D0A.
class DeyeBMS : public esphome::ble_client::BLEClientNode, public PollingComponent {
 public:
  void setup() override;
  void loop() override;
  void update();
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

  // конфигурация имени батареи (localName из advertising, напр. BAT25212000E3050050)
  void set_ble_name(const std::string &n) { this->ble_name_ = n; }

  // сенсоры
  void set_cell(int i, sensor::Sensor *s) { this->cell_[i] = s; }
  void set_cell_max(sensor::Sensor *s) { this->cell_max_ = s; }
  void set_cell_min(sensor::Sensor *s) { this->cell_min_ = s; }
  void set_cell_diff(sensor::Sensor *s) { this->cell_diff_ = s; }
  void set_cell_max_no(sensor::Sensor *s) { this->cell_max_no_ = s; }
  void set_cell_min_no(sensor::Sensor *s) { this->cell_min_no_ = s; }
  void set_total_voltage(sensor::Sensor *s) { this->total_voltage_ = s; }
  void set_current(sensor::Sensor *s) { this->current_ = s; }
  void set_soc(sensor::Sensor *s) { this->soc_ = s; }
  void set_soh(sensor::Sensor *s) { this->soh_ = s; }
  void set_temp_cell(sensor::Sensor *s) { this->temp_cell_ = s; }
  void set_temp_mos(sensor::Sensor *s) { this->temp_mos_ = s; }
  void set_temp_env(sensor::Sensor *s) { this->temp_env_ = s; }

 protected:
  std::string ble_name_;

  uint16_t char_handle_{0};
  bool notify_registered_{false};

  // состояние сессии
  enum State { IDLE, HANDSHAKE_SENT, RESULT_SENT, READY } state_{IDLE};
  std::string random_;               // 16 hex-символов, генерим сами
  uint8_t key_[16];                  // сессионный ключ
  bool have_key_{false};

  // буфер сборки зашифрованного ответа
  std::vector<uint8_t> rx_buf_;
  // очередь опроса: индексы команд
  uint8_t poll_idx_{0};

  // сенсоры
  sensor::Sensor *cell_[16]{};
  sensor::Sensor *cell_max_{nullptr};
  sensor::Sensor *cell_min_{nullptr};
  sensor::Sensor *cell_diff_{nullptr};
  sensor::Sensor *cell_max_no_{nullptr};
  sensor::Sensor *cell_min_no_{nullptr};
  sensor::Sensor *total_voltage_{nullptr};
  sensor::Sensor *current_{nullptr};
  sensor::Sensor *soc_{nullptr};
  sensor::Sensor *soh_{nullptr};
  sensor::Sensor *temp_cell_{nullptr};
  sensor::Sensor *temp_mos_{nullptr};
  sensor::Sensor *temp_env_{nullptr};

  // --- протокол ---
  void start_handshake_();
  void send_result_();
  void derive_key_();
  void send_next_poll_();
  void write_char_(const uint8_t *data, size_t len);
  void handle_notify_(const uint8_t *data, size_t len);
  void try_parse_response_();
  void parse_frame_(const std::vector<uint8_t> &f);
  void reset_session_();

  // крипта/утилиты
  static uint16_t crc16_(const uint8_t *d, size_t len);
  void aes_encrypt_(const uint8_t *in, size_t len, std::vector<uint8_t> &out);
  bool aes_decrypt_(const uint8_t *in, size_t len, std::vector<uint8_t> &out);
};

}  // namespace deye_bms
}  // namespace esphome

#endif
