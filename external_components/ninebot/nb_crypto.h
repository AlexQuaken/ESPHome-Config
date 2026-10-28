#pragma once

// Шифрование Segway-Ninebot Encryption2 (BLE, заголовок 5A A5). Разобрано по
// research/repos/segway-ninebot-ble/docs/encryption.md и authentication.md.
// Модуль без зависимостей от ESPHome: собирается и на ПК, там его сверяет
// с эталоном из документации tools/test_nb_crypto.
//
// AES-128 в своем режиме, похожем на CCM. Ключ это SHA-1(key1 ‖ key2)[0:16],
// каждый ключ добит нулями до 16 байт. Кадр на проводе: 5A A5 LEN, шифртекст
// (BT_ID TARGET CMD INDEX DATA), хвост 6 байт. Со счетчиком (SN-режим) хвост это
// зашифрованный тег 4 байта и счетчик 2 байта big-endian; без счетчика
// 00 00 сумма_lo сумма_hi 00 00, а шифр это один и тот же блок AES на весь кадр.

#include <cstddef>
#include <cstdint>
#include <vector>

namespace nbcrypto {

// Константа протокола Gen2 (fw_data в libnbcrypto.so), общая для всех устройств.
// Документация описывает ее подстановку в первой фазе двумя фразами, которые
// можно прочитать по-разному, поэтому вариантов четыре, см. set_precomm_variant().
extern const uint8_t FW_DATA[16];

enum Result : int {
  OK = 0,
  FAILED = -1,
  FAILED_AUTH = -2,  // тег не сошелся
  FAILED_SUM = -4,   // не сошлась сумма в кадре без счетчика
};

class Crypto {
 public:
  // key1 обрезается или добивается нулями до 16 байт, key2 == nullptr это 16 нулей.
  void set_key(const uint8_t *key1, size_t key1_len, const uint8_t *key2);
  // Блок, который шифруется ключом в режиме без счетчика: нули или FW_DATA.
  void set_static_block(const uint8_t *block16);
  // Ключ первой фазы PRE_COMM по варианту: бит 0 это FW_DATA вместо key2,
  // бит 1 это FW_DATA вместо нулевого блока в режиме без счетчика.
  void set_precomm_variant(const uint8_t *name, size_t name_len, uint8_t variant);
  void set_auth(const uint8_t *auth16);

  // plain это открытый кадр 5A A5 LEN 3E ... Счетчик 0 значит режим без счетчика.
  std::vector<uint8_t> encrypt(const std::vector<uint8_t> &plain, uint32_t counter) const;
  // Режим берется из хвоста: счетчик 0 значит без счетчика. В counter отдается
  // счетчик из кадра.
  int decrypt(const uint8_t *wire, size_t len, std::vector<uint8_t> &plain, uint16_t *counter) const;

  const uint8_t *key() const { return this->key_; }

  static void derive_key(const uint8_t *key1, size_t key1_len, const uint8_t *key2, uint8_t out[16]);
  static void sha256(const uint8_t *data, size_t len, uint8_t out[32]);

 protected:
  void aes_(const uint8_t in[16], uint8_t out[16]) const;
  void nonce_(uint32_t counter, uint8_t out[13]) const;
  void cbc_mac_(const uint8_t *plain, size_t len, const uint8_t nonce[13], uint8_t tag[4]) const;
  void ctr_(const uint8_t *in, size_t len, const uint8_t nonce[13], uint8_t *out) const;

  uint8_t key_[16]{};
  uint8_t auth_[16]{};
  uint8_t static_block_[16]{};
};

// Открытый кадр: 5A A5 LEN 3E target cmd index data.
std::vector<uint8_t> build_frame(uint8_t target, uint8_t cmd, uint8_t index, const uint8_t *data, size_t len,
                                 uint8_t sync2 = 0xA5);

}  // namespace nbcrypto
