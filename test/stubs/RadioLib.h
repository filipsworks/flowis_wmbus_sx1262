// Host stub of the RadioLib 7.7.1 SX1262 API used by wmbus_sx1262.cpp.
// Records every configuration call so tests can assert the radio setup; the
// receive buffer / packet length / readData status are test-controlled.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <string>
#include <vector>

#define RADIOLIB_ERR_NONE (0)
#define RADIOLIB_ERR_CRC_MISMATCH (-7)
#define RADIOLIB_ENCODING_NRZ (0x00)
#define RADIOLIB_NC (0xFFFFFFFF)

namespace radiostub {
inline uint8_t rxBuf[512];
inline size_t pktLen = 255;
inline int16_t readState = RADIOLIB_ERR_NONE;
inline void (*isr)() = nullptr;
inline float freq = 0, rxBw = 0;
inline uint8_t sync[8]; inline size_t syncLen = 0;
inline size_t preamble = 0;
inline uint32_t rxEn = 0, txEn = 0;
inline bool dio2 = false;
inline std::vector<std::string> calls;   // call order
}  // namespace radiostub

class Module { public: Module(int, int, int, int) {} };

class SX1262 {
  static int16_t ok(const char *c) { radiostub::calls.push_back(c); return RADIOLIB_ERR_NONE; }
 public:
  SX1262(Module *) {}
  int16_t beginFSK(float f = 434.0, float = 4.8, float = 5.0, float bw = 156.2, int8_t = 10,
                   uint16_t = 16, float = 1.6, bool = false) { radiostub::freq = f; radiostub::rxBw = bw; return ok("beginFSK"); }
  int16_t standby() { return ok("standby"); }
  int16_t setFrequency(float f) { radiostub::freq = f; return ok("setFrequency"); }
  int16_t setRxBandwidth(float bw) { radiostub::rxBw = bw; return ok("setRxBandwidth"); }
  int16_t setSyncWord(uint8_t *s, size_t n) { memcpy(radiostub::sync, s, n); radiostub::syncLen = n; return ok("setSyncWord"); }
  int16_t setCRC(uint8_t, uint16_t = 0x1D0F, uint16_t = 0x1021, bool = true) { return ok("setCRC"); }
  int16_t setWhitening(bool, uint16_t = 0x01FF) { return ok("setWhitening"); }
  int16_t setEncoding(uint8_t) { return ok("setEncoding"); }
  int16_t setPreambleLength(size_t n) { radiostub::preamble = n; return ok("setPreambleLength"); }
  int16_t fixedPacketLengthMode(uint8_t n = 0xFF) { radiostub::pktLen = n; return ok("fixedPacketLengthMode"); }
  int16_t setRxBoostedGainMode(bool, bool = true) { return ok("setRxBoostedGainMode"); }
  int16_t setDio2AsRfSwitch(bool e = true) { radiostub::dio2 = e; return ok("setDio2AsRfSwitch"); }
  void setRfSwitchPins(uint32_t rx, uint32_t tx) { radiostub::rxEn = rx; radiostub::txEn = tx; ok("setRfSwitchPins"); }
  void setPacketReceivedAction(void (*f)(void)) { radiostub::isr = f; ok("setPacketReceivedAction"); }
  int16_t startReceive() { return ok("startReceive"); }
  size_t getPacketLength(bool = true) { return radiostub::pktLen; }
  int16_t readData(uint8_t *data, size_t len) {
    size_t n = radiostub::pktLen;
    if (len != 0 && len < n) n = len;
    memcpy(data, radiostub::rxBuf, n);
    return radiostub::readState;
  }
  float getRSSI() { return -110.0f; }   // RssiAvg: diluted by the post-frame noise
  int16_t calibrateImage(float) { return ok("calibrateImage"); }
 protected:
  uint32_t getPacketStatus() { return (0x00u << 16) | (120u << 8) | 220u; }  // RssiSync -60, RssiAvg -110
};
