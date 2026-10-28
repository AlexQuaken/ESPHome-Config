#include "nb_crypto.h"

#include <cstring>

#include <mbedtls/aes.h>
#include <mbedtls/sha1.h>
#include <mbedtls/sha256.h>

namespace nbcrypto {

const uint8_t FW_DATA[16] = {0x97, 0xCF, 0xB8, 0x02, 0x84, 0x41, 0x43, 0xDE,
                             0x56, 0x00, 0x2B, 0x3B, 0x34, 0x78, 0x0A, 0x5D};

void Crypto::derive_key(const uint8_t *key1, size_t key1_len, const uint8_t *key2, uint8_t out[16]) {
  uint8_t buf[32] = {0};
  memcpy(buf, key1, key1_len < 16 ? key1_len : 16);
  if (key2 != nullptr)
    memcpy(buf + 16, key2, 16);
  uint8_t hash[20];
  mbedtls_sha1(buf, sizeof(buf), hash);
  memcpy(out, hash, 16);
}

void Crypto::sha256(const uint8_t *data, size_t len, uint8_t out[32]) { mbedtls_sha256(data, len, out, 0); }

void Crypto::set_key(const uint8_t *key1, size_t key1_len, const uint8_t *key2) {
  derive_key(key1, key1_len, key2, this->key_);
}

void Crypto::set_static_block(const uint8_t *block16) {
  if (block16 == nullptr)
    memset(this->static_block_, 0, 16);
  else
    memcpy(this->static_block_, block16, 16);
}

void Crypto::set_precomm_variant(const uint8_t *name, size_t name_len, uint8_t variant) {
  this->set_key(name, name_len, (variant & 1) ? FW_DATA : nullptr);
  this->set_static_block((variant & 2) ? FW_DATA : nullptr);
}

void Crypto::set_auth(const uint8_t *auth16) { memcpy(this->auth_, auth16, 16); }

void Crypto::aes_(const uint8_t in[16], uint8_t out[16]) const {
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  mbedtls_aes_setkey_enc(&ctx, this->key_, 128);
  mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT, in, out);
  mbedtls_aes_free(&ctx);
}

// counter_BE[4] ‖ auth[0:8] ‖ 00
void Crypto::nonce_(uint32_t counter, uint8_t out[13]) const {
  out[0] = counter >> 24;
  out[1] = counter >> 16;
  out[2] = counter >> 8;
  out[3] = counter;
  memcpy(out + 4, this->auth_, 8);
  out[12] = 0;
}

// B_0 = 59 ‖ nonce ‖ 00 ‖ длина после заголовка, затем заголовок из 3 байт с нулями
// до 16, затем блоки полезной части. Тег это первые 4 байта.
void Crypto::cbc_mac_(const uint8_t *plain, size_t len, const uint8_t nonce[13], uint8_t tag[4]) const {
  uint8_t x[16], blk[16];
  blk[0] = 0x59;
  memcpy(blk + 1, nonce, 13);
  blk[14] = 0;
  blk[15] = (len - 3) & 0xFF;
  this->aes_(blk, x);

  memset(blk, 0, 16);
  memcpy(blk, plain, 3);
  for (int i = 0; i < 16; i++)
    blk[i] ^= x[i];
  this->aes_(blk, x);

  for (size_t off = 3; off < len; off += 16) {
    size_t n = len - off < 16 ? len - off : 16;
    memset(blk, 0, 16);
    memcpy(blk, plain + off, n);
    for (int i = 0; i < 16; i++)
      blk[i] ^= x[i];
    this->aes_(blk, x);
  }
  memcpy(tag, x, 4);
}

// A_i = 01 ‖ nonce ‖ 00 ‖ i, данные с i = 1.
void Crypto::ctr_(const uint8_t *in, size_t len, const uint8_t nonce[13], uint8_t *out) const {
  uint8_t a[16], ks[16];
  a[0] = 0x01;
  memcpy(a + 1, nonce, 13);
  a[14] = 0;
  for (size_t off = 0, i = 1; off < len; off += 16, i++) {
    a[15] = i & 0xFF;
    this->aes_(a, ks);
    size_t n = len - off < 16 ? len - off : 16;
    for (size_t j = 0; j < n; j++)
      out[off + j] = in[off + j] ^ ks[j];
  }
}

std::vector<uint8_t> Crypto::encrypt(const std::vector<uint8_t> &plain, uint32_t counter) const {
  size_t n = plain.size();
  std::vector<uint8_t> out(n + 6, 0);
  memcpy(out.data(), plain.data(), 3);

  if (counter == 0) {
    uint16_t sum = 0;
    for (size_t i = 3; i < n; i++)
      sum += plain[i];
    sum = ~sum;
    uint8_t ks[16];
    this->aes_(this->static_block_, ks);
    for (size_t i = 3; i < n; i++)
      out[i] = plain[i] ^ ks[(i - 3) % 16];
    out[n + 2] = sum & 0xFF;
    out[n + 3] = sum >> 8;
    return out;
  }

  uint8_t nonce[13], tag[4], a0[16], ks[16];
  this->nonce_(counter, nonce);
  this->cbc_mac_(plain.data(), n, nonce, tag);
  this->ctr_(plain.data() + 3, n - 3, nonce, out.data() + 3);
  a0[0] = 0x01;
  memcpy(a0 + 1, nonce, 13);
  a0[14] = 0;
  a0[15] = 0;
  this->aes_(a0, ks);
  for (int i = 0; i < 4; i++)
    out[n + i] = tag[i] ^ ks[i];
  out[n + 4] = (counter >> 8) & 0xFF;
  out[n + 5] = counter & 0xFF;
  return out;
}

int Crypto::decrypt(const uint8_t *wire, size_t len, std::vector<uint8_t> &plain, uint16_t *counter) const {
  if (len < 9)
    return FAILED;
  size_t n = len - 6;
  uint16_t cnt = (wire[len - 2] << 8) | wire[len - 1];
  if (counter != nullptr)
    *counter = cnt;
  plain.assign(wire, wire + n);

  if (cnt == 0) {
    uint8_t ks[16];
    this->aes_(this->static_block_, ks);
    uint16_t sum = 0;
    for (size_t i = 3; i < n; i++) {
      plain[i] = wire[i] ^ ks[(i - 3) % 16];
      sum += plain[i];
    }
    sum = ~sum;
    uint16_t got = wire[n + 2] | (wire[n + 3] << 8);
    return got == sum ? OK : FAILED_SUM;
  }

  uint8_t nonce[13], tag[4], a0[16], ks[16];
  this->nonce_(cnt, nonce);
  this->ctr_(wire + 3, n - 3, nonce, plain.data() + 3);
  this->cbc_mac_(plain.data(), n, nonce, tag);
  a0[0] = 0x01;
  memcpy(a0 + 1, nonce, 13);
  a0[14] = 0;
  a0[15] = 0;
  this->aes_(a0, ks);
  for (int i = 0; i < 4; i++)
    if ((uint8_t) (wire[n + i] ^ ks[i]) != tag[i])
      return FAILED_AUTH;
  return OK;
}

std::vector<uint8_t> build_frame(uint8_t target, uint8_t cmd, uint8_t index, const uint8_t *data, size_t len,
                                 uint8_t sync2) {
  std::vector<uint8_t> f = {0x5A, sync2, (uint8_t) len, 0x3E, target, cmd, index};
  if (len)
    f.insert(f.end(), data, data + len);
  return f;
}

}  // namespace nbcrypto
