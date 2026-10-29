// Host stub of rweather Crypto's AES128 (single-block ECB) on macOS CommonCrypto.
#pragma once
#include <CommonCrypto/CommonCryptor.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

class AES128 {
  uint8_t key_[16] = {0};

 public:
  bool setKey(const uint8_t *key, size_t len) {
    if (len != 16) return false;
    memcpy(key_, key, 16);
    return true;
  }
  void decryptBlock(uint8_t *out, const uint8_t *in) {
    size_t moved = 0;
    CCCrypt(kCCDecrypt, kCCAlgorithmAES, kCCOptionECBMode, key_, 16, nullptr, in, 16, out, 16, &moved);
  }
};
