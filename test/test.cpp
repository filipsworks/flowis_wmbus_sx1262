// Host test for ../wmbus_sx1262.cpp, compiled unmodified against stubs/.
// Run from this folder (macOS / Linux with clang++ or g++):
//   clang++ -std=c++17 -I stubs -Wall test.cpp -o /tmp/wmbus_test && /tmp/wmbus_test
// FLOWIS+ cases are real captures from merbanan/rtl_433#2317; wM-Bus cases are
// wmbusmeters simulation telegrams re-encoded (CRC + 3-of-6) by an independent
// reference encoder below. Arduino ignores this folder when building the sketch.
#include <stdio.h>
#include <vector>
#include <string>

#include "../wmbus_sx1262.cpp"
#define SERIAL_OF(p) (p).serial

typedef std::vector<uint8_t> Bytes;
static int fails = 0, passes = 0;

static void check(bool cond, const std::string &name, const std::string &detail = "") {
  printf("%s  %s%s%s\n", cond ? "PASS" : "FAIL", name.c_str(), detail.empty() ? "" : "  -- ", detail.c_str());
  (cond ? passes : fails)++;
}

static Bytes hex(std::string h) {
  Bytes b;
  std::string c;
  for (char ch : h) if (isxdigit((unsigned char)ch)) c += ch;
  for (size_t i = 0; i + 1 < c.size(); i += 2) b.push_back((uint8_t)strtoul(c.substr(i, 2).c_str(), 0, 16));
  return b;
}

// --- independent reference implementations (not taken from the file) ------
static uint16_t refCrc(const Bytes &d, uint16_t poly, uint16_t init) {  // rtl_433 bit_util.c crc16
  uint16_t r = init;
  for (uint8_t b : d) {
    r ^= (uint16_t)b << 8;
    for (int k = 0; k < 8; k++) r = (r & 0x8000) ? (uint16_t)((r << 1) ^ poly) : (uint16_t)(r << 1);
  }
  return r;
}
static uint16_t refCrcEN(const Bytes &d) { return (uint16_t)~refCrc(d, 0x3D65, 0); }

static Bytes addCrcA(const Bytes &t) {  // EN 13757-4 frame format A
  Bytes out;
  size_t i = 0;
  auto blk = [&](size_t n) {
    Bytes b(t.begin() + i, t.begin() + i + n);
    uint16_t c = refCrcEN(b);
    out.insert(out.end(), b.begin(), b.end());
    out.push_back(c >> 8);
    out.push_back(c & 0xFF);
    i += n;
  };
  blk(10);
  while (i < t.size()) blk(std::min<size_t>(16, t.size() - i));
  return out;
}

static Bytes addCrcB(const Bytes &t) {  // format B, total <= 128: L counts the CRC
  Bytes out = t;
  out[0] = (uint8_t)(t.size() - 1 + 2);
  uint16_t c = refCrcEN(out);
  out.push_back(c >> 8);
  out.push_back(c & 0xFF);
  return out;
}

static const uint8_t enc6[16] = {0x16, 0x0D, 0x0E, 0x0B, 0x1C, 0x19, 0x1A, 0x13,
                                 0x2C, 0x25, 0x26, 0x23, 0x34, 0x31, 0x32, 0x29};
static Bytes encode3of6(const Bytes &in) {
  Bytes out;
  uint32_t acc = 0;
  int bits = 0;
  for (uint8_t b : in)
    for (int h = 0; h < 2; h++) {
      acc = (acc << 6) | enc6[h == 0 ? b >> 4 : b & 15];
      bits += 6;
      while (bits >= 8) { out.push_back((acc >> (bits - 8)) & 0xFF); bits -= 8; }
    }
  if (bits) {  // pad the last byte with postamble chips 0101
    int pad = 8 - bits;
    out.push_back(((acc << pad) | (0x55 & ((1 << pad) - 1))) & 0xFF);
  }
  return out;
}

static uint32_t lcg = 12345;
static uint8_t noise() { lcg = lcg * 1103515245u + 12345u; return (uint8_t)(lcg >> 16); }

// Feed one capture through wmbusPoll(); `total` = what getPacketLength reports.
static WmbusPollResult feed(const Bytes &capture, size_t total, WmbusPacket &pkt, int tail = -1) {
  for (size_t i = 0; i < sizeof(radiostub::rxBuf); i++)
    radiostub::rxBuf[i] = i < capture.size() ? capture[i] : (tail < 0 ? noise() : (uint8_t)tail);
  radiostub::pktLen = total;
  rxFlag = true;
  return wmbusPoll(pkt);
}

static std::string hexs(const uint8_t *b, int n) {
  std::string s;
  char t[3];
  for (int i = 0; i < n; i++) { snprintf(t, sizeof t, "%02X", b[i]); s += t; }
  return s;
}

static void wmbusCase(const std::string &name, const Bytes &clean, const Bytes &onAir, size_t total,
                      uint8_t mode, int tail = -1) {
  WmbusPacket pkt;
  WmbusPollResult r = feed(onAir, total, pkt, tail);
  bool same = pkt.len == (int)clean.size() && memcmp(pkt.telegram, clean.data(), clean.size()) == 0;
  check(r == WMBUS_OK && pkt.mode == mode && same, name,
        "result=" + std::to_string(r) + " mode=" + wmbusModeName(pkt.mode) + " len=" + std::to_string(pkt.len) +
            "/" + std::to_string(clean.size()) + (same ? "" : " telegram=" + hexs(pkt.telegram, pkt.len)));
}

int main() {
  for (int i = 0; i < 16; i++)
    check(kDecode3of6[enc6[i]] == i, "3-of-6 table: reference encoder nibble " + std::to_string(i));

  // ---- radio setup -------------------------------------------------------
  wmbusBegin(RADIO_FLOWIS);
  auto order = [](const char *a, const char *b) {
    int ia = -1, ib = -1;
    for (int i = 0; i < (int)radiostub::calls.size(); i++) {
      if (radiostub::calls[i] == a && ia < 0) ia = i;
      if (radiostub::calls[i] == b) ib = i;  // last
    }
    return ia >= 0 && ib >= 0 && ia < ib;
  };
  check(radiostub::rxEn == 38 && radiostub::txEn == RADIOLIB_NC, "RXEN: setRfSwitchPins(38, NC)");
  check(radiostub::dio2, "DIO2 as RF switch");
  check(order("setRfSwitchPins", "startReceive"), "RXEN configured before startReceive");
  check(radiostub::freq == 868.00f && radiostub::rxBw == 312.0f, "FLOWIS profile: 868.00 MHz, 312 kHz");
  check(radiostub::syncLen == 4 && memcmp(radiostub::sync, "\xD3\x91\xD3\x91", 4) == 0, "FLOWIS profile: sync D391D391");
  check(radiostub::preamble == 16 && radiostub::pktLen == 64, "FLOWIS profile: preamble 16, fixed length 64");
  check(order("setSyncWord", "setPreambleLength"), "setSyncWord before setPreambleLength (detector cap)");
  check(order("beginFSK", "calibrateImage"), "image calibration after every beginFSK");
  radiostub::calls.clear();
  wmbusSetProfile(RADIO_WMBUS);
  check(radiostub::freq == 868.95f && radiostub::rxBw == 234.3f && radiostub::syncLen == 2 &&
            memcmp(radiostub::sync, "\x54\x3D", 2) == 0 && radiostub::preamble == 8 && radiostub::pktLen == 255,
        "wM-Bus profile: 868.95 MHz, 234.3 kHz, sync 543D, preamble 8, length 255");
  check(radiostub::calls.back() == "startReceive" && radiostub::calls.front() == "standby",
        "profile switch: standby ... startReceive");

  // ---- FLOWIS+ : real captures from merbanan/rtl_433#2317 ---------------
  wmbusSetProfile(RADIO_FLOWIS);
  struct { const char *name, *frame; bool good; uint32_t id; double m3; } fl[] = {
      {"FLOWIS g010 type 1", "1201b64f510e81c374605cef000000450000a55ff7", true, 240209846, 0.239},
      {"FLOWIS g012 type 1", "1201b64f510e81da74605cef000000450000a51b8d", true, 240209846, 0.239},
      {"FLOWIS g014 type 1", "1201b64f510e81f074605cef000000450000a5977d", true, 240209846, 0.239},
      {"FLOWIS meter 240234918 type 1", "1201a6b1510e81f996625cf1000000110000a57168", true, 240234918, 0.241},
      {"FLOWIS meter 240234918 type 1 #2", "1201a6b1510e811e97625cf1000000110000a50a87", true, 240234918, 0.241},
      {"FLOWIS g011 type 2", "1302b64f510e0000000000000000000000000000fcb2", true, 240209846, -1},
      {"FLOWIS mis-transcribed capture (bad CRC)", "1302a6b1510e1c000000000000000000000002015ff9", false, 240234918, -1},
      {"FLOWIS flipped volume bit (bad CRC)", "1201b64f510e81c374605cee000000450000a55ff7", false, 240209846, -1},
  };
  for (auto &c : fl) {
    WmbusPacket pkt;
    Bytes f = hex(c.frame);
    WmbusPollResult r = feed(f, 64, pkt);
    bool okShape = pkt.mode == WMBUS_MODE_FLOWIS && pkt.len == (int)f.size() && SERIAL_OF(pkt) == c.id;
    check(okShape && (r == WMBUS_OK) == c.good, c.name,
          "result=" + std::to_string(r) + " serial=" + std::to_string(SERIAL_OF(pkt)) + " len=" + std::to_string(pkt.len));
    if (c.good && c.m3 >= 0) {
      WmbusValues v;
      bool d = wmbusDecode(pkt, (const uint8_t *)"0123456789abcdef", false, v);
      check(d && v.hasVolume && fabs(v.totalM3 - c.m3) < 1e-9, std::string(c.name) + ": decode without key",
            "volume=" + std::to_string(v.totalM3));
    }
    if (c.good && c.m3 < 0) {
      WmbusValues v;
      check(!wmbusDecode(pkt, (const uint8_t *)"0123456789abcdef", false, v), std::string(c.name) + ": no reading (type 2)");
    }
  }
  {
    WmbusPacket pkt;
    Bytes f = hex("1201b64f510e81c374605cef000000450000a55ff7");
    WmbusPollResult r = feed(f, 64, pkt, 0x00);
    check(r == WMBUS_OK, "FLOWIS frame followed by zero tail");
    WmbusValues v;
    wmbusDecode(pkt, (const uint8_t *)"0123456789abcdef", false, v);
    check(v.status == 0x45, "FLOWIS alarm byte b[15] -> status");
    check(pkt.rssiDbm == -60, "RSSI taken at sync (RssiSync), not averaged over the capture",
          std::to_string(pkt.rssiDbm));
    r = feed(hex("40"), 64, pkt);  // L = 64 cannot fit a 64-byte capture
    check(r != WMBUS_OK && pkt.len == 0, "FLOWIS impossible length treated as noise");
  }

  wmbusSetProfile(RADIO_WMBUS);

  // ---- wM-Bus: real telegrams from wmbusmeters simulations --------------
  Bytes t1odd = hex("1E44AE4C9956341268077A36001000 2F2F0413181E0000023B00002F2F2F2F");      // L=30, framelen 37
  Bytes t1odd2 = hex("1844AE4C4455223368077A55000000 041389E20100023B0000");                 // L=24, framelen 29
  Bytes t1even = hex("1F44AE4C9956341268077A36001000 2F2F0413181E0000023B00002F2F2F2F2F");  // L=31, framelen 38
  Bytes t1long = hex("4E4401061010101002027A00004005 2F2F0E035040691500000B2B300300066D00790C7423400C78371204860BABC8FC100000000E833C8074000000000BAB3C0000000AFDC9FC0136022F2F2F2F2F");
  Bytes c1 = hex("2A442D2C998734761B168D2091D37CAC21576C78 02FF207100041308190000441308190000615B7F616713");

  auto t1air = [](const Bytes &t) { return encode3of6(addCrcA(t)); };
  wmbusCase("T1 even frame length + noise tail (255 B capture)", t1even, t1air(t1even), 255, WMBUS_MODE_T1);
  wmbusCase("T1 odd frame length + noise tail (255 B capture)", t1odd, t1air(t1odd), 255, WMBUS_MODE_T1);
  wmbusCase("T1 odd #2 + noise tail", t1odd2, t1air(t1odd2), 255, WMBUS_MODE_T1);
  wmbusCase("T1 L=0x4E (91 B frame) + noise tail", t1long, t1air(t1long), 255, WMBUS_MODE_T1);
  wmbusCase("T1 even + 0x55 tail", t1even, t1air(t1even), 255, WMBUS_MODE_T1, 0x55);
  wmbusCase("T1 even, exact length, no tail", t1even, t1air(t1even), t1air(t1even).size(), WMBUS_MODE_T1);
  wmbusCase("T1 odd, exact length, no tail", t1odd, t1air(t1odd), t1air(t1odd).size(), WMBUS_MODE_T1);

  Bytes a = {0x54, 0xCD};
  Bytes fa = addCrcA(c1);
  a.insert(a.end(), fa.begin(), fa.end());
  wmbusCase("C1 format A + noise tail", c1, a, 255, WMBUS_MODE_C1A);
  Bytes b = {0x54, 0x3D};
  Bytes fb = addCrcB(c1);
  b.insert(b.end(), fb.begin(), fb.end());
  wmbusCase("C1 format B + noise tail", c1, b, 255, WMBUS_MODE_C1B);

  {  // unencrypted wM-Bus must decode without a key
    WmbusPacket pkt;
    WmbusPollResult r = feed(t1air(t1odd), 255, pkt);
    WmbusValues v;
    bool d = wmbusDecode(pkt, (const uint8_t *)"0123456789abcdef", false, v);
    check(r == WMBUS_OK && d && v.hasVolume && fabs(v.totalM3 - 7.704) < 1e-9 && SERIAL_OF(pkt) == 12345699,
          "T1 unencrypted: volume 7.704 m3, serial 12345699, no key",
          "volume=" + std::to_string(v.totalM3) + " serial=" + std::to_string(SERIAL_OF(pkt)));
  }

  {  // non-DIF/VIF CIs must never be parsed as records (ELL/AFL/manufacturer)
    int bogus = 0;
    for (uint8_t ci : {0x8C, 0x8D, 0x90, 0xA0, 0xA2, 0x5A, 0x7F}) {
      Bytes t = hex("1E44AE4C9956341268077A36001000 2F2F0413181E0000023B00002F2F2F2F");
      t[10] = ci;
      WmbusPacket pkt;
      feed(t1air(t), 255, pkt);
      WmbusValues v;
      if (wmbusDecode(pkt, (const uint8_t *)"0123456789abcdef", false, v)) bogus++;
    }
    check(bogus == 0, "CI 8C/8D/90/A0/A2/5A/7F: not decoded as DIF/VIF", std::to_string(bogus) + " decoded");
    Bytes t = hex("1444AE4C995634126807780413181E0000023B0000");  // CI 0x78, no TPL
    t[0] = (uint8_t)(t.size() - 1);
    WmbusPacket pkt;
    feed(t1air(t), 255, pkt);
    WmbusValues v;
    check(wmbusDecode(pkt, (const uint8_t *)"0123456789abcdef", false, v) && fabs(v.totalM3 - 7.704) < 1e-9,
          "CI 0x78 (no TPL header) still decoded", std::to_string(v.totalM3));
  }

  int noiseOk = 0;
  for (int i = 0; i < 2000; i++) {
    Bytes n;
    for (int k = 0; k < 255; k++) n.push_back(noise());
    WmbusPacket pkt;
    if (feed(n, 255, pkt) == WMBUS_OK) noiseOk++;
  }
  check(noiseOk == 0, "2000 pure-noise captures: none accepted", std::to_string(noiseOk) + " accepted");

  printf("\n%d passed, %d failed\n", passes, fails);
  return fails ? 1 : 0;
}
