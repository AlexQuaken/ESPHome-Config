#pragma once
// Расшифровка ответа BLE-монитора аккумулятора BM6 (он же Ancel BM200, Sealey BT2020).
// Протокол: служба FFF0, в FFF3 пишется зашифрованная команда d1550700..., в ответ
// FFF4 шлет уведомления по 16 байт. Шифр AES-128 CBC, IV из нулей, ключ из
// приложения: "leagend" FF FE "0100009". Разбор по tarball.ca и JeffWDH/bm6-battery-monitor.
//   байт 0-2  d1 55 07, признак ответа на эту команду
//   байт 3    1 значит температура ниже нуля
//   байт 4    температура, °C
//   байт 5    назначение неизвестно (в примерах 01)
//   байт 6    заряд, %
//   байт 7-8  напряжение, сотые вольта, старший байт первым
#include <vector>
#include "mbedtls/aes.h"

namespace bm6 {

// Команда "d1550700000000000000000000000000", уже зашифрованная тем же ключом
static const std::vector<uint8_t> CMD = {0x69, 0x7e, 0xa0, 0xb5, 0xd5, 0x4c, 0xf0, 0x24,
                                         0xe7, 0x94, 0x77, 0x23, 0x55, 0x55, 0x41, 0x14};

inline bool decode(const std::vector<uint8_t> &x, float &volts, float &temp, float &soc) {
  if (x.size() < 16) return false;
  static const uint8_t key[16] = {'l', 'e', 'a', 'g', 'e', 'n', 'd', 0xff,
                                  0xfe, '0', '1', '0', '0', '0', '0', '9'};
  uint8_t iv[16] = {0};
  uint8_t out[16];
  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_dec(&aes, key, 128);
  mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, 16, iv, x.data(), out);
  mbedtls_aes_free(&aes);

  char hex[33];
  for (int i = 0; i < 16; i++) snprintf(hex + i * 2, 3, "%02x", out[i]);
  ESP_LOGD("bm6", "raw %s", hex);

  if (out[0] != 0xd1 || out[1] != 0x55 || out[2] != 0x07) return false;
  volts = ((out[7] << 8) | out[8]) / 100.0f;
  if (volts <= 0.0f || volts > 30.0f) return false;
  temp = out[3] == 1 ? -(float) out[4] : (float) out[4];
  soc = out[6];
  return true;
}

}  // namespace bm6
