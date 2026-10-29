// wmbus_sx1262.cpp  — see wmbus_sx1262.h for the overview.
//
// Radio layer: RadioLib SX1262 in FSK mode, switchable between two profiles:
//
//  RADIO_FLOWIS — Apator FLOWIS+ overlays (serials 2401..2405, fw v24.x). These
//    do NOT speak wM-Bus: proprietary frame at ~868.0 MHz, 100 kbps FSK,
//    preamble 0xAA.., sync 0xD391D391, then L + L bytes + CRC-16 (poly 0x8005,
//    init 0xFFFF), unencrypted. Reverse engineered in rtl_433 (decoder 237,
//    src/devices/flowis.c, issue merbanan/rtl_433#2317).
//  RADIO_WMBUS  — standard wM-Bus T1/C1 at 868.95 MHz (newer "Flow Meters PRO"
//    OMS modules, and every other wM-Bus meter). The SX1262 packet engine
//    cannot undo T-mode 3-out-of-6 coding, so we capture a fixed block after
//    the 0x543D sync word and do the framing in software.

#include "wmbus_sx1262.h"

#include <RadioLib.h>
#include <SPI.h>
#include <math.h>
#include <string.h>

// AES-128 block cipher from the "Crypto" library (CBC done manually).
#include <AES.h>

// ---------------------------------------------------------------------------
//  Wio-SX1262 for XIAO ESP32-S3 — pin map (B2B connector).
//  Source: RadioBoards/SeeedStudio/XIAO_ESP32S3.h + RadioLib discussion #1361.
// ---------------------------------------------------------------------------
#define SX_NSS    41   // SPI chip select
#define SX_DIO1   39   // IRQ
#define SX_RST    42   // reset
#define SX_BUSY   40   // busy
#define SX_SCK     7
#define SX_MISO    8
#define SX_MOSI    9
#define SX_RXEN   38   // RF switch RX enable — MUST be driven high to receive.
                       // Meshtastic variant: SX126X_RXEN 38 (PR #5517: "a bit
                       // deaf ... now 20-40 dB stronger"); Seeed OCH BSP sets
                       // antenna_sw = 38 high at init.
#define SX_TCXO_V  1.8 // Wio-SX1262 TCXO on DIO3 (Meshtastic: 1.8 V)

// Both profiles are 100 kbps 2-FSK with ~50 kHz deviation.
#define WMBUS_BITRATE   100.0
#define WMBUS_DEV       50.0    // frequency deviation (kHz) — tune if needed

// wM-Bus T1/C1 profile.
#define WMBUS_FREQ      868.95
#define WMBUS_RXBW      234.3   // RX bandwidth (kHz), nearest valid SX126x value

// FLOWIS+ proprietary profile. rtl_433 users receive it with -f 868M; issue
// #2317 measured "a very strong, stable signal on 868.01 MHz". 312 kHz RX BW
// matches the 325 kHz a forum user ran on a CC1101 and tolerates the offset.
#define FLOWIS_FREQ     868.00
#define FLOWIS_RXBW     312.0
#define FLOWIS_RX_LEN   64      // L byte + payload + CRC fit easily (L is ~0x12)
// Fixed coded bytes captured per telegram. 255 = SX126x hardware maximum and
// the same value the SzczepanLeon esphome driver uses (payload length 0xFF).
// 255 coded bytes -> 170 decoded -> supports telegrams up to L ~ 150 (typical
// water meters send L < 100).
#define WMBUS_RX_LEN    255

// getRSSI() returns RssiAvg, averaged over the whole fixed-length capture —
// mostly noise after a short frame (FLOWIS: ~21 of 64 bytes). RssiSync is
// latched when the sync word matches, i.e. while the transmitter is on air.
struct SX1262Rx : public SX1262 {
  using SX1262::SX1262;
  float rssiSync() { return -(float)((getPacketStatus() >> 8) & 0xFF) / 2.0f; }
};
static SX1262Rx radio = new Module(SX_NSS, SX_DIO1, SX_RST, SX_BUSY);
static volatile bool rxFlag = false;
static bool radioOk = false;
static uint8_t profile = RADIO_FLOWIS;
static uint32_t c1FramesSeen = 0;

uint32_t wmbusC1Detected() { return c1FramesSeen; }

IRAM_ATTR static void onRxIsr() { rxFlag = true; }

// ===========================================================================
//  Software wM-Bus decode (identical to the CC1101 build)
// ===========================================================================

// --- 3-out-of-6 ---
static const uint8_t kDecode3of6[] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0x03, 0xFF, 0x01, 0x02, 0xFF,
    0xFF, 0xFF, 0xFF, 0x07, 0xFF, 0xFF, 0x00, 0xFF,
    0xFF, 0x05, 0x06, 0xFF, 0x04, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0x0B, 0xFF, 0x09, 0x0A, 0xFF,
    0xFF, 0x0F, 0xFF, 0xFF, 0x08, 0xFF, 0xFF, 0xFF,
    0xFF, 0x0D, 0x0E, 0xFF, 0x0C, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Decode n bytes (2n six-bit codes, MSB-first chip stream) from a T-mode
// capture. Bit-serial on purpose: an odd byte count ends mid-triplet, and only
// the codes that belong to the frame may be checked — the chips after it are
// postamble or, on the SX1262's fixed-length capture, receiver noise.
// Needs inLen >= (3n+1)/2. Returns the number of invalid codes, -1 if short.
static int decode3of6Bytes(const uint8_t *in, int inLen, uint8_t *out, int n) {
  if ((3 * n + 1) / 2 > inLen) return -1;
  int errors = 0;
  for (int i = 0; i < 2 * n; i++) {
    int bit = i * 6, byte = bit >> 3, off = bit & 7;
    uint16_t w = (in[byte] << 8) | (byte + 1 < inLen ? in[byte + 1] : 0);
    uint8_t nib = kDecode3of6[(w >> (10 - off)) & 0x3F];
    if (nib == 0xFF) { errors++; nib = 0; }
    if (i & 1) out[i >> 1] |= nib;
    else       out[i >> 1] = nib << 4;
  }
  return errors;
}

// --- CRC-16, MSB-first (wM-Bus: poly 0x3D65 init 0 inverted; FLOWIS: 0x8005 / 0xFFFF) ---
static uint16_t crc16(const uint8_t *data, int len, uint16_t poly, uint16_t crc) {
  for (int i = 0; i < len; i++) {
    uint8_t b = data[i];
    for (int k = 0; k < 8; k++) {
      if (((crc & 0x8000) >> 8) ^ (b & 0x80)) crc = (crc << 1) ^ poly;
      else crc = (crc << 1);
      b <<= 1;
    }
  }
  return crc;
}

static uint16_t crcEN13757(const uint8_t *data, int len) {
  return ~crc16(data, len, 0x3D65, 0x0000);
}

static bool crcBlockOk(const uint8_t *data, int len, const uint8_t *crcPtr) {
  uint16_t got = ((uint16_t)crcPtr[0] << 8) | crcPtr[1];
  return crcEN13757(data, len) == got;
}

const char *wmbusModeName(uint8_t mode) {
  switch (mode) {
    case WMBUS_MODE_T1:  return "T1";
    case WMBUS_MODE_C1A: return "C1-A";
    case WMBUS_MODE_C1B: return "C1-B";
    case WMBUS_MODE_FLOWIS: return "FLOWIS";
    default:             return "?";
  }
}

static int frameLenWithCrc(uint8_t L) {
  int data = (int)L - 9;
  if (data < 0) data = 0;
  int full = data / 16;
  int rem = data % 16;
  int len = 12 + full * 18;
  if (rem) len += rem + 2;
  return len;
}

static int dechainAndCheck(const uint8_t *in, int frameLen, uint8_t *out,
                           int outMax) {
  if (frameLen < 12 || 10 > outMax) return -1;
  if (!crcBlockOk(in, 10, in + 10)) return -1;
  memcpy(out, in, 10);
  int o = 10, i = 12;
  while (i < frameLen) {
    int dataLen = frameLen - i - 2;
    if (dataLen > 16) dataLen = 16;
    if (dataLen <= 0) break;
    if (o + dataLen > outMax) return -1;
    if (!crcBlockOk(in + i, dataLen, in + i + dataLen)) return -1;
    memcpy(out + o, in + i, dataLen);
    o += dataLen;
    i += dataLen + 2;
  }
  return o;
}

// Frame format B: the L field counts the CRCs. One CRC at the end for frames up
// to 128 bytes, otherwise one at offset 126 and one at the end. Strips them and
// rewrites L, mirroring wmbusmeters' trimCRCsFrameFormatB.
static int trimFormatB(const uint8_t *in, int len, uint8_t *out, int outMax) {
  if (len < 12 || len > outMax) return -1;
  int crc1 = (len <= 128) ? len - 2 : 126;
  int crc2 = (len <= 128) ? 0 : len - 2;

  if (crcEN13757(in, crc1) != (uint16_t)((in[crc1] << 8) | in[crc1 + 1])) return -1;
  memcpy(out, in, crc1);
  int o = crc1;

  if (crc2 > 0) {
    int len2 = crc2 - crc1 - 2;
    if (len2 < 0) return -1;
    if (crcEN13757(in + crc1 + 2, len2) != (uint16_t)((in[crc2] << 8) | in[crc2 + 1])) return -1;
    memcpy(out + o, in + crc1 + 2, len2);
    o += len2;
  }
  out[0] = o - 1;  // rewrite L so it matches the trimmed frame
  return o;
}

static uint32_t bcd4ToDecimal(const uint8_t *b) {
  uint32_t v = 0, mul = 1;
  for (int k = 0; k < 4; k++) {
    v += ((b[k] & 0x0F) + 10 * (b[k] >> 4)) * mul;
    mul *= 100;
  }
  return v;
}

// --- DIF/VIF ---
static int difLen(uint8_t df, bool *bcd) {
  *bcd = false;
  switch (df) {
    case 0x00: return 0;
    case 0x01: return 1;
    case 0x02: return 2;
    case 0x03: return 3;
    case 0x04: return 4;
    case 0x05: return 4;
    case 0x06: return 6;
    case 0x07: return 8;
    case 0x08: return 0;
    case 0x09: *bcd = true; return 1;
    case 0x0A: *bcd = true; return 2;
    case 0x0B: *bcd = true; return 3;
    case 0x0C: *bcd = true; return 4;
    case 0x0D: return -1;
    case 0x0E: *bcd = true; return 6;
    default: return -1;
  }
}

static long long readValue(const uint8_t *d, int len, bool bcd) {
  long long v = 0;
  if (bcd) {
    long long mul = 1;
    for (int k = 0; k < len; k++) {
      v += ((d[k] & 0x0F) + 10 * (d[k] >> 4)) * mul;
      mul *= 100;
    }
  } else {
    for (int k = 0; k < len; k++) v |= ((long long)d[k]) << (8 * k);
    if (len < 8 && (d[len - 1] & 0x80)) v |= (~0LL) << (8 * len);
  }
  return v;
}

static void parseRecords(const uint8_t *d, int len, WmbusValues &v) {
  int i = 0;
  while (i < len) {
    uint8_t dif = d[i++];
    if (dif == 0x0F || dif == 0x1F) break;
    if (dif == 0x2F) continue;

    uint8_t dataField = dif & 0x0F;
    uint32_t storage = (dif >> 6) & 0x01;
    uint8_t prev = dif;
    int difeCount = 0;
    while (prev & 0x80) {
      if (i >= len) return;
      uint8_t dife = d[i++];
      storage |= ((uint32_t)(dife & 0x0F)) << (1 + 4 * difeCount);
      difeCount++;
      prev = dife;
    }

    if (i >= len) return;
    uint8_t vif = d[i++];
    uint8_t vp = vif & 0x7F;
    prev = vif;
    while (prev & 0x80) {
      if (i >= len) return;
      prev = d[i++];
    }

    bool bcd;
    int dlen = difLen(dataField, &bcd);
    if (dlen < 0) {
      if (i >= len) return;
      uint8_t lvar = d[i++];
      i += lvar;
      continue;
    }
    if (i + dlen > len) return;
    long long value = readValue(d + i, dlen, bcd);
    i += dlen;

    if (vp >= 0x10 && vp <= 0x17) {
      double m3 = (double)value * pow(10.0, (int)(vp & 0x07) - 6);
      if (storage == 0) {
        if (!v.hasVolume) { v.totalM3 = m3; v.hasVolume = true; }
      } else {
        if (!v.hasTarget) { v.targetM3 = m3; v.hasTarget = true; }
      }
    } else if (vp >= 0x38 && vp <= 0x3F) {
      if (!v.hasFlow) {
        v.flowM3h = (double)value * pow(10.0, (int)(vp & 0x07) - 6);
        v.hasFlow = true;
      }
    } else if (vp >= 0x58 && vp <= 0x5B) {
      if (!v.hasTemp) {
        v.flowTempC = (double)value * pow(10.0, (int)(vp & 0x03) - 3);
        v.hasTemp = true;
      }
    } else if (vp >= 0x64 && vp <= 0x67) {
      if (!v.hasTemp) {
        v.flowTempC = (double)value * pow(10.0, (int)(vp & 0x03) - 3);
        v.hasTemp = true;
      }
    }
  }
}

bool wmbusDecode(const WmbusPacket &pkt, const uint8_t key[16], bool keySet,
                 WmbusValues &out) {
  memset(&out, 0, sizeof(out));

  if (pkt.mode == WMBUS_MODE_FLOWIS) {
    // rtl_433 flowis.c (b[0] = L): only message type 1 carries the reading.
    // Volume = 24-bit little-endian litres at b[11..13]; b[14] backflow,
    // b[15] alarm. Not encrypted, so no key is needed.
    const uint8_t *b = pkt.telegram;
    if (pkt.len < 16 || b[1] != 1) return false;
    out.ci = b[1];
    out.totalM3 = (b[11] | (b[12] << 8) | ((uint32_t)b[13] << 16)) / 1000.0;
    out.hasVolume = true;
    out.status = b[15];
    return true;
  }

  const uint8_t *f = pkt.telegram;
  int len = pkt.len;
  if (len < 11) return false;

  int p = 10;
  out.ci = f[p++];

  if (out.ci == 0x72) {
    if (p + 8 > len) return false;
    p += 8;
  } else if (out.ci == 0x78) {       // no TPL header: plain DIF/VIF records
    parseRecords(f + p, len - p, out);
    return out.hasVolume || out.hasTemp || out.hasFlow;
  } else if (out.ci != 0x7A) {
    return false;  // ELL/AFL/manufacturer-specific CI: not DIF/VIF, may be ciphertext
  }

  if (p + 4 > len) return false;
  out.accessNo = f[p++];
  out.status = f[p++];
  uint16_t cfg = f[p] | (f[p + 1] << 8);
  p += 2;
  out.securityMode = (cfg >> 8) & 0x1F;
  uint8_t numBlocks = (cfg >> 4) & 0x0F;

  if (out.securityMode == 0 || numBlocks == 0) {
    parseRecords(f + p, len - p, out);
    return out.hasVolume || out.hasTemp || out.hasFlow;
  }

  if (!keySet) return false;
  if (out.securityMode != 5) return false;

  int encBytes = numBlocks * 16;
  if (p + encBytes > len) encBytes = (len - p) & ~0x0F;
  if (encBytes < 16) return false;

  uint8_t iv[16];
  iv[0] = f[2];
  iv[1] = f[3];
  for (int k = 0; k < 6; k++) iv[2 + k] = f[4 + k];
  for (int k = 0; k < 8; k++) iv[8 + k] = out.accessNo;

  static uint8_t plain[WMBUS_MAX_FRAME];
  AES128 aes;
  aes.setKey(key, 16);
  uint8_t prev[16];
  memcpy(prev, iv, 16);
  for (int off = 0; off < encBytes; off += 16) {  // AES-128-CBC decrypt
    uint8_t blk[16];
    aes.decryptBlock(blk, f + p + off);
    for (int k = 0; k < 16; k++) plain[off + k] = blk[k] ^ prev[k];
    memcpy(prev, f + p + off, 16);
  }

  if (!(plain[0] == 0x2F && plain[1] == 0x2F)) {
    out.decrypted = false;
    return false;
  }
  out.decrypted = true;

  parseRecords(plain + 2, encBytes - 2, out);
  int tailStart = p + encBytes;
  if (tailStart < len) parseRecords(f + tailStart, len - tailStart, out);

  return out.hasVolume || out.hasTemp || out.hasFlow;
}

// ===========================================================================
//  SX1262 radio
// ===========================================================================

bool wmbusChipPresent() { return radioOk; }

const char *wmbusProfileName(uint8_t p) {
  return p == RADIO_FLOWIS ? "FLOWIS+ 868.00 MHz" : "wM-Bus T1/C1 868.95 MHz";
}

uint8_t wmbusProfile() { return profile; }

// (Re)configure what differs between the profiles; both share bitrate,
// deviation, NRZ, no whitening, no hardware CRC and fixed-length capture.
static int applyProfile(uint8_t p) {
  static uint8_t syncWmbus[]  = {0x54, 0x3D};
  static uint8_t syncFlowis[] = {0xD3, 0x91, 0xD3, 0x91};
  bool fl = (p == RADIO_FLOWIS);
  int st;
  radio.standby();
  rxFlag = false;
  if ((st = radio.setFrequency(fl ? FLOWIS_FREQ : WMBUS_FREQ)) != RADIOLIB_ERR_NONE) return st;
  if ((st = radio.setRxBandwidth(fl ? FLOWIS_RXBW : WMBUS_RXBW)) != RADIOLIB_ERR_NONE) return st;
  if ((st = radio.setSyncWord(fl ? syncFlowis : syncWmbus, fl ? 4 : 2)) != RADIOLIB_ERR_NONE) return st;
  // Preamble detector: 8 bits for wM-Bus T-mode (short preamble, as in the
  // SzczepanLeon driver), 16 bits for FLOWIS (0xAAAAAAA). Must follow
  // setSyncWord: RadioLib caps the detector at the sync-word length.
  if ((st = radio.setPreambleLength(fl ? 16 : 8)) != RADIOLIB_ERR_NONE) return st;
  if ((st = radio.fixedPacketLengthMode(fl ? FLOWIS_RX_LEN : WMBUS_RX_LEN)) != RADIOLIB_ERR_NONE) return st;
  profile = p;
  return radio.startReceive();
}

bool wmbusSetProfile(uint8_t p) {
  int st = applyProfile(p);
  if (st != RADIOLIB_ERR_NONE) Serial.printf("SX1262 profile %u failed: %d\n", p, st);
  return st == RADIOLIB_ERR_NONE;
}

bool wmbusBegin(uint8_t p) {
  SPI.begin(SX_SCK, SX_MISO, SX_MOSI, SX_NSS);

  // beginFSK(freq, bitRate, freqDev, rxBw, power, preambleLen, tcxoVoltage)
  int st = radio.beginFSK(WMBUS_FREQ, WMBUS_BITRATE, WMBUS_DEV, WMBUS_RXBW, 10,
                          16, SX_TCXO_V);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("SX1262 beginFSK failed: %d\n", st);
    radioOk = false;
    return false;
  }

  // beginFSK resets the chip (image calibration back to 902-928 MHz) but
  // setFrequency() skips recalibration when the cached frequency is < 20 MHz
  // away — true on every re-init. The 863-870 MHz preset covers both profiles.
  radio.calibrateImage(WMBUS_FREQ);
  radio.setCRC(0);                            // CRCs are checked in software
  radio.setWhitening(false);
  radio.setEncoding(RADIOLIB_ENCODING_NRZ);
  radio.setRxBoostedGainMode(true);
  // RF switch: DIO2 drives the TX side, GPIO38 must be high for RX. RadioLib
  // raises it on every startReceive() and drops it in standby/TX.
  radio.setDio2AsRfSwitch(true);
  radio.setRfSwitchPins(SX_RXEN, RADIOLIB_NC);
  radio.setPacketReceivedAction(onRxIsr);

  radioOk = wmbusSetProfile(p);
  if (radioOk) Serial.printf("SX1262 ready: %s\n", wmbusProfileName(profile));
  return radioOk;
}

static void fillHeader(WmbusPacket &pkt) {
  if (pkt.len >= 4)  { pkt.mfct[0] = pkt.telegram[2]; pkt.mfct[1] = pkt.telegram[3]; }
  if (pkt.len >= 8)  pkt.serial = bcd4ToDecimal(pkt.telegram + 4);
  if (pkt.len >= 9)  pkt.version = pkt.telegram[8];
  if (pkt.len >= 10) pkt.devType = pkt.telegram[9];
}

// Classify a capture as T1 or C1 and decode it. T1 is 3-out-of-6 coded and
// always frame format A; C1 is uncoded and announces its format with the two
// marker bytes 0x54 0xCD (A) / 0x54 0x3D (B) that follow the shared sync word.
// Both modes share the same radio settings, so one receiver handles both.
static WmbusPollResult wmbusClassify(const uint8_t *coded, int codedLen,
                                     uint8_t *decoded, WmbusPacket &pkt) {
  const uint8_t *frame;
  int frameLen;
  bool formatB = false;

  if (coded[0] == 0x54 && (coded[1] == 0xCD || coded[1] == 0x3D)) {
    formatB = (coded[1] == 0x3D);
    pkt.mode = formatB ? WMBUS_MODE_C1B : WMBUS_MODE_C1A;
    c1FramesSeen++;
    frame = coded + 2;
    int avail = codedLen - 2;
    if (avail < 12 || frame[0] < 10 || frame[0] > 250) return WMBUS_ERR;
    frameLen = formatB ? (frame[0] + 1) : frameLenWithCrc(frame[0]);
    if (frameLen > avail) frameLen = avail;
  } else {
    // T1: read L from the first 12 chips, then decode exactly the frame and
    // nothing after it. Any invalid code inside the frame means noise.
    if (decode3of6Bytes(coded, codedLen, decoded, 1) != 0) return WMBUS_ERR;
    if (decoded[0] < 10 || decoded[0] > 250) return WMBUS_ERR;
    frameLen = frameLenWithCrc(decoded[0]);
    if (frameLen > WMBUS_MAX_FRAME) return WMBUS_ERR;
    if (decode3of6Bytes(coded, codedLen, decoded, frameLen) != 0) return WMBUS_ERR;
    pkt.mode = WMBUS_MODE_T1;
    frame = decoded;
  }

  // Best-effort copy first, so a CRC failure still leaves something to show.
  pkt.len = frameLen < WMBUS_MAX_FRAME ? frameLen : WMBUS_MAX_FRAME - 1;
  memcpy(pkt.telegram, frame, pkt.len);
  fillHeader(pkt);

  int clean = formatB ? trimFormatB(frame, frameLen, pkt.telegram, WMBUS_MAX_FRAME)
                      : dechainAndCheck(frame, frameLen, pkt.telegram, WMBUS_MAX_FRAME);
  if (clean < 0) {                       // CRC failed — restore the raw copy
    memcpy(pkt.telegram, frame, pkt.len);
    fillHeader(pkt);
    return WMBUS_ERR;
  }
  pkt.len = clean;
  fillHeader(pkt);
  return WMBUS_OK;
}

// FLOWIS+ frame straight after the D391D391 sync: L, L payload bytes, then a
// big-endian CRC-16 (poly 0x8005, init 0xFFFF) over L + payload (rtl_433
// flowis.c). payload[0] = message type, payload[1..4] = meter id, little
// endian — the number printed on the meter.
static WmbusPollResult flowisClassify(const uint8_t *b, int n, WmbusPacket &pkt) {
  pkt.mode = WMBUS_MODE_FLOWIS;
  int L = b[0];
  if (L < 5 || L + 3 > n) return WMBUS_ERR;   // impossible length: noise
  pkt.len = L + 3;                             // keep the CRC, as rtl_433 shows it
  memcpy(pkt.telegram, b, pkt.len);
  pkt.devType = b[1];
  pkt.serial = b[2] | (b[3] << 8) | (b[4] << 16) | ((uint32_t)b[5] << 24);
  uint16_t got = (b[L + 1] << 8) | b[L + 2];
  return crc16(b, L + 1, 0x8005, 0xFFFF) == got ? WMBUS_OK : WMBUS_ERR;
}

WmbusPollResult wmbusPoll(WmbusPacket &pkt) {
  if (!rxFlag) return WMBUS_NONE;
  rxFlag = false;

  pkt.len = 0;
  pkt.serial = 0;
  pkt.mfct[0] = pkt.mfct[1] = 0;
  pkt.version = pkt.devType = 0;
  pkt.mode = WMBUS_MODE_UNKNOWN;

  static uint8_t coded[WMBUS_MAX_FRAME * 2];
  static uint8_t decoded[WMBUS_MAX_FRAME];

  int rxLen = radio.getPacketLength();
  if (rxLen <= 0 || rxLen > (int)sizeof(coded))
    rxLen = (profile == RADIO_FLOWIS) ? FLOWIS_RX_LEN : WMBUS_RX_LEN;
  int st = radio.readData(coded, rxLen);
  pkt.rssiDbm = (int)lround(radio.rssiSync());
  // Re-arm. A failed re-arm leaves the chip in standby with RXEN low: report
  // it through wmbusChipPresent() so the sketch re-initialises the radio.
  if (radio.startReceive() != RADIOLIB_ERR_NONE) radioOk = false;

  if (st != RADIOLIB_ERR_NONE && st != RADIOLIB_ERR_CRC_MISMATCH) return WMBUS_ERR;

  if (rxLen < 3) return WMBUS_ERR;

  return profile == RADIO_FLOWIS ? flowisClassify(coded, rxLen, pkt)
                                 : wmbusClassify(coded, rxLen, decoded, pkt);
}
