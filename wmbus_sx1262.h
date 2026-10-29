// wmbus_sx1262.h
// Meter receiver for XIAO ESP32-S3 + Wio-SX1262 (RadioLib, FSK mode).
//
// Two radio profiles (one SX1262 cannot listen on both frequencies at once):
//   RADIO_FLOWIS — Apator FLOWIS+ overlays (serials 2401..2405, fw v24.x):
//                  proprietary, NOT wM-Bus, 868.0 MHz, sync 0xD391D391,
//                  unencrypted (rtl_433 decoder 237).
//   RADIO_WMBUS  — wM-Bus T1 + C1 (format A/B) at 868.95 MHz, e.g. the newer
//                  "Flow Meters PRO" OMS modules (encrypted, AES mode 5).
// All framing, CRC, 3-of-6, AES and DIF/VIF parsing is done in software.

#ifndef WMBUS_SX1262_H
#define WMBUS_SX1262_H

#include <Arduino.h>

#define WMBUS_MAX_FRAME 256

enum WmbusPollResult {
  WMBUS_NONE = 0,   // nothing received
  WMBUS_OK   = 1,   // a CRC-valid telegram was captured
  WMBUS_ERR  = 2    // bytes received but decode / CRC failed (partial copy in telegram)
};

// Link mode / frame format of a capture.
enum WmbusMode {
  WMBUS_MODE_UNKNOWN = 0,  // noise, or not a frame we recognise
  WMBUS_MODE_T1      = 1,  // T1, 3-out-of-6 coded, frame format A
  WMBUS_MODE_C1A     = 2,  // C1, uncoded, frame format A
  WMBUS_MODE_C1B     = 3,  // C1, uncoded, frame format B
  WMBUS_MODE_FLOWIS  = 4   // Apator FLOWIS+ proprietary frame
};

// Radio profile = frequency + sync word the SX1262 is listening for.
enum RadioProfile {
  RADIO_FLOWIS = 0,
  RADIO_WMBUS  = 1
};

struct WmbusPacket {
  uint8_t  telegram[WMBUS_MAX_FRAME];  // wM-Bus: L,C,M,M,A(6),CI,... (CRCs stripped)
                                       // FLOWIS: L, payload, CRC (as rtl_433 shows it)
  int      len;
  uint32_t serial;     // number printed on the meter (wM-Bus: BCD A-field, FLOWIS: LE id)
  uint8_t  mfct[2];
  uint8_t  version;
  uint8_t  devType;
  int      rssiDbm;
  uint8_t  mode;                       // WmbusMode of this capture
};

// Human-readable name of a WmbusMode value.
const char *wmbusModeName(uint8_t mode);

struct WmbusValues {
  bool    decrypted;
  uint8_t ci;
  uint8_t securityMode;
  uint8_t status;
  uint8_t accessNo;

  bool    hasVolume;  double totalM3;
  bool    hasTarget;  double targetM3;
  bool    hasTemp;    double flowTempC;
  bool    hasFlow;    double flowM3h;
};

// Initialise the SX1262 and start RX on the given RadioProfile.
bool wmbusBegin(uint8_t profile);

// Switch the running receiver to another RadioProfile.
bool wmbusSetProfile(uint8_t profile);
uint8_t wmbusProfile();
const char *wmbusProfileName(uint8_t profile);

// True if the SX1262 last initialised successfully (used for retry/recovery).
bool wmbusChipPresent();

// How many mode-C1 frames were seen so far (diagnostics).
uint32_t wmbusC1Detected();

// Poll for a received packet (interrupt-driven; this just checks a flag).
WmbusPollResult wmbusPoll(WmbusPacket &pkt);

// Decode a clean telegram (decrypts if keySet and mode == AES-CBC mode 5).
bool wmbusDecode(const WmbusPacket &pkt, const uint8_t key[16], bool keySet,
                 WmbusValues &out);

#endif  // WMBUS_SX1262_H
