// flowis_wmbus_sx1262.ino
// ---------------------------------------------------------------------------
// XIAO ESP32-S3 + Wio-SX1262 wireless M-Bus receiver for Apator FLOWIS water
// meters, with Home Assistant MQTT discovery and a built-in web UI listing the
// last 10 raw telegrams and the state of every meter in range (/meters).
//
// This is the SX1262 (RadioLib, FSK) variant of the CC1101 sketch. The radio
// layer lives in wmbus_sx1262.cpp; everything else (decode/AES/DIF-VIF, MQTT,
// web UI) is the same logic ported to ESP32.
//
// Apator FLOWIS+ overlays (serials 2401..2405) do NOT transmit wM-Bus: they use
// a proprietary, unencrypted frame at 868.0 MHz (rtl_433 decoder 237). The
// radio mode "auto" alternates between that and wM-Bus T1/C1 at 868.95 MHz.
// The AES key is only needed for encrypted wM-Bus meters.
//
// Required libraries (Arduino Library Manager):
//   - esp32 board package (Espressif)         -> board "XIAO_ESP32S3"
//   - WiFiManager       (tzapu)
//   - PubSubClient      (Nick O'Leary)
//   - ArduinoJson v7    (Benoit Blanchon)
//   - RadioLib          (Jan Gromes)
//   - Crypto            (Rhys Weatherley)      -> AES.h
//   LittleFS + WebServer come with the ESP32 core.
//
// Hardware: Wio-SX1262 plugs onto the XIAO ESP32-S3 via the B2B connector.
// Pins (NSS 41 / DIO1 39 / RST 42 / BUSY 40, SPI 7/8/9) are set in
// wmbus_sx1262.cpp. Just fit an 868 MHz antenna.
// ---------------------------------------------------------------------------

#include <FS.h>
#include <LittleFS.h>

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoOTA.h>
#include <algorithm>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "wmbus_sx1262.h"

#define DEVICE_NAME  "Flowis wM-Bus (SX1262)"
#define HOSTNAME     "flowis-sx1262"
#define CONFIG_FILE  "/config.json"
#define FW_VERSION   "1.0.0-sx1262"

const unsigned long MQTT_RETRY_MS = 5000UL;
const int           TELEGRAM_LOG_SIZE = 10;

// ---- runtime configuration ----
char mqttServer[40] = "";
char mqttPort[6]    = "1883";
char mqttUser[32]   = "";
char mqttPass[32]   = "";
char mqttPrefix[24] = "flowis";
char meterIdStr[16] = "";
char aesKeyStr[33]  = "";
char ntfyUrl[96]    = "";   // e.g. http://ntfy.sh/moj-temat; empty = no notifications
char radioMode[8]   = "auto"; // "auto" | "flowis" | "wmbus"
char otaPass[33]    = "";   // Arduino IDE network upload; empty = OTA disabled

uint32_t meterFilter = 0;
uint8_t  aesKey[16];
bool     keySet = false;
bool     shouldSaveConfig = false;

// ---- objects ----
WiFiClient    espClient;
PubSubClient  mqtt(espClient);
WiFiManager   wm;
WebServer     server(80);

unsigned long lastMqttRetry = 0;
bool          radioOk = false;
unsigned long lastRadioCheck = 0;
unsigned long lastNoiseNote = 0;

// Radio mode "auto": one SX1262 cannot listen on 868.00 and 868.95 MHz at
// once, so hop between the profiles. 17 s dwell (34 s cycle) on purpose: it
// shares no factor with typical meter intervals (10/30/60/120 s), so the phase
// drifts and a periodic transmitter cannot keep landing in the other window.
const unsigned long AUTO_DWELL_MS = 17000UL;
unsigned long lastHop = 0;
bool          autoLocked = false;   // auto found the configured meter; stop hopping

// ---- HA discovery state ----
bool discTotal = false, discTarget = false, discTemp = false,
     discFlow = false, discRssi = false, discStatus = false;

// ---- telegram ring buffer ----
struct LogEntry {
  unsigned long ts;
  int           rssi;
  uint32_t      serial;
  bool          ok;
  bool          mine;
  String        hex;
};
LogEntry logBuf[TELEGRAM_LOG_SIZE];
int  logHead = 0;
int  logCount = 0;
unsigned long totalReceived = 0;

// ---- every meter heard (web UI /meters only — never published to HA) -----
const int METER_TABLE_SIZE = 32;
struct MeterEntry {
  uint32_t      serial;
  uint8_t       mode;        // WmbusMode of the last frame
  uint8_t       mfct[2];
  bool          hasVolume;   // a reading was decoded at least once
  double        totalM3;
  uint8_t       status;      // FLOWIS+: alarm byte, wM-Bus: TPL status
  bool          encrypted;   // last frame encrypted and not decodable (no / wrong key)
  int           rssi;
  unsigned long lastSeen;
  uint32_t      frames;      // 0 = free slot
};
MeterEntry meters[METER_TABLE_SIZE];

// ===========================================================================
//  Helpers
// ===========================================================================

String bytesToHex(const uint8_t *data, int len) {
  static const char *h = "0123456789ABCDEF";
  String s;
  s.reserve(len * 2);
  for (int i = 0; i < len; i++) { s += h[data[i] >> 4]; s += h[data[i] & 0x0F]; }
  return s;
}

int hexToBytes(const char *hex, uint8_t *out, int maxBytes) {
  int n = strlen(hex);
  if (n % 2) return 0;
  int bytes = n / 2;
  if (bytes > maxBytes) return 0;
  for (int i = 0; i < bytes; i++) {
    auto nib = [](char c) -> int {
      if (c >= '0' && c <= '9') return c - '0';
      if (c >= 'a' && c <= 'f') return c - 'a' + 10;
      if (c >= 'A' && c <= 'F') return c - 'A' + 10;
      return -1;
    };
    int a = nib(hex[i * 2]), b = nib(hex[i * 2 + 1]);
    if (a < 0 || b < 0) return 0;
    out[i] = (a << 4) | b;
  }
  return bytes;
}

String mfctToString(const uint8_t *m) {
  uint16_t v = m[0] | (m[1] << 8);
  char s[4];
  s[0] = ((v >> 10) & 0x1F) + 64;
  s[1] = ((v >> 5) & 0x1F) + 64;
  s[2] = (v & 0x1F) + 64;
  s[3] = 0;
  return String(s);
}

bool autoMode() { return strcmp(radioMode, "flowis") && strcmp(radioMode, "wmbus"); }
uint8_t fixedProfile() { return strcmp(radioMode, "wmbus") ? RADIO_FLOWIS : RADIO_WMBUS; }

// OTA only with a password: without one anyone on the LAN could flash firmware.
// The partition table (default_8MB: app0/app1 3.3 MB each) already has room.
bool otaRunning = false;
void applyOta() {
  if (strlen(otaPass) == 0) {
    if (otaRunning) { ArduinoOTA.end(); otaRunning = false; }
    return;
  }
  ArduinoOTA.setPassword(otaPass);
  if (!otaRunning) {
    ArduinoOTA.setHostname(HOSTNAME);  // Arduino IDE port: "flowis-sx1262 at <ip>"
    ArduinoOTA.begin();
    otaRunning = true;
  }
}

void applyCrypto() {
  meterFilter = strtoul(meterIdStr, nullptr, 10);
  keySet = (hexToBytes(aesKeyStr, aesKey, 16) == 16);
}

// ===========================================================================
//  Configuration persistence
// ===========================================================================

void loadConfig() {
  if (!LittleFS.begin(true)) { Serial.println(F("LittleFS mount failed")); return; }
  if (!LittleFS.exists(CONFIG_FILE)) { Serial.println(F("No saved config")); return; }
  File f = LittleFS.open(CONFIG_FILE, "r");
  if (!f) return;
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) { Serial.print(F("Config parse failed: ")); Serial.println(err.c_str()); return; }
  strlcpy(mqttServer, doc["mqtt_server"] | "",       sizeof(mqttServer));
  strlcpy(mqttPort,   doc["mqtt_port"]   | "1883",   sizeof(mqttPort));
  strlcpy(mqttUser,   doc["mqtt_user"]   | "",       sizeof(mqttUser));
  strlcpy(mqttPass,   doc["mqtt_pass"]   | "",       sizeof(mqttPass));
  strlcpy(mqttPrefix, doc["mqtt_prefix"] | "flowis", sizeof(mqttPrefix));
  strlcpy(meterIdStr, doc["meter_id"]    | "",       sizeof(meterIdStr));
  strlcpy(aesKeyStr,  doc["aes_key"]     | "",       sizeof(aesKeyStr));
  strlcpy(ntfyUrl,    doc["ntfy_url"]    | "",       sizeof(ntfyUrl));
  strlcpy(radioMode,  doc["radio_mode"]  | "auto",   sizeof(radioMode));
  strlcpy(otaPass,    doc["ota_pass"]    | "",       sizeof(otaPass));
  Serial.println(F("Config loaded"));
}

void saveConfig() {
  JsonDocument doc;
  doc["mqtt_server"] = mqttServer;
  doc["mqtt_port"]   = mqttPort;
  doc["mqtt_user"]   = mqttUser;
  doc["mqtt_pass"]   = mqttPass;
  doc["mqtt_prefix"] = mqttPrefix;
  doc["meter_id"]    = meterIdStr;
  doc["aes_key"]     = aesKeyStr;
  doc["ntfy_url"]    = ntfyUrl;
  doc["radio_mode"]  = radioMode;
  doc["ota_pass"]    = otaPass;
  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) { Serial.println(F("Config save failed")); return; }
  serializeJson(doc, f);
  f.close();
  Serial.println(F("Config saved"));
}

void saveConfigCallback() { shouldSaveConfig = true; }

// ===========================================================================
//  MQTT / Home Assistant discovery
// ===========================================================================

String deviceInfoJson() {
  return String("{\"manufacturer\":\"Apator\",\"model\":\"FLOWIS\",")
       + "\"name\":\"" DEVICE_NAME "\",\"sw_version\":\"" FW_VERSION "\","
       + "\"identifiers\":[\"" + String(HOSTNAME) + "-" + String(mqttPrefix) + "\"]}";
}

String stateTopic(const char *key) { return String(mqttPrefix) + "/" + key; }

void publishDiscovery(const char *key, const char *name, const char *devClass,
                      const char *unit, const char *stateClass, const char *icon,
                      int precision, bool diagnostic) {
  String cfg = String("{\"name\":\"") + name + "\",";
  cfg += "\"state_topic\":\"" + stateTopic(key) + "\",";
  cfg += "\"unique_id\":\"" + String(mqttPrefix) + "_" + key + "\",";
  if (devClass && devClass[0])     cfg += "\"device_class\":\"" + String(devClass) + "\",";
  if (stateClass && stateClass[0]) cfg += "\"state_class\":\"" + String(stateClass) + "\",";
  if (unit && unit[0])             cfg += "\"unit_of_measurement\":\"" + String(unit) + "\",";
  if (icon && icon[0])             cfg += "\"icon\":\"" + String(icon) + "\",";
  if (precision >= 0)              cfg += "\"suggested_display_precision\":" + String(precision) + ",";
  if (diagnostic)                  cfg += "\"entity_category\":\"diagnostic\",";
  cfg += "\"availability_topic\":\"" + stateTopic("status") + "\",";
  cfg += "\"payload_available\":\"online\",\"payload_not_available\":\"offline\",";
  cfg += "\"device\":" + deviceInfoJson() + "}";

  String topic = String("homeassistant/sensor/") + mqttPrefix + "_" + key + "/config";
  mqtt.publish(topic.c_str(), cfg.c_str(), true);
}

bool mqttConnect() {
  if (strlen(mqttServer) == 0) return false;
  mqtt.setBufferSize(1024);
  mqtt.setServer(mqttServer, atoi(mqttPort));

  String mac = WiFi.macAddress(); mac.replace(":", "");
  String clientId = String(HOSTNAME) + "-" + mac;
  String availTopic = stateTopic("status");

  bool ok;
  if (strlen(mqttUser) > 0)
    ok = mqtt.connect(clientId.c_str(), mqttUser, mqttPass, availTopic.c_str(), 0, true, "offline");
  else
    ok = mqtt.connect(clientId.c_str(), availTopic.c_str(), 0, true, "offline");

  if (ok) {
    Serial.println(F("MQTT connected"));
    mqtt.publish(availTopic.c_str(), "online", true);
    publishDiscovery("rssi", "Sygnał RSSI", "signal_strength", "dBm",
                     "measurement", "mdi:radio-tower", 0, true);
    discRssi = true;
  } else {
    Serial.printf("MQTT failed, rc=%d\n", mqtt.state());
  }
  return ok;
}

void publishValues(const WmbusPacket &pkt, const WmbusValues &v) {
  if (!mqtt.connected()) return;

  if (v.hasVolume) {
    if (!discTotal) {
      publishDiscovery("total_m3", "Zużycie wody", "water", "m³",
                       "total_increasing", "mdi:water", 3, false);
      discTotal = true;
    }
    mqtt.publish(stateTopic("total_m3").c_str(), String(v.totalM3, 3).c_str(), true);
  }
  if (v.hasTarget) {
    if (!discTarget) {
      publishDiscovery("target_m3", "Zużycie (data rozliczeniowa)", "water", "m³",
                       "", "mdi:water-outline", 3, false);
      discTarget = true;
    }
    mqtt.publish(stateTopic("target_m3").c_str(), String(v.targetM3, 3).c_str(), true);
  }
  if (v.hasTemp) {
    if (!discTemp) {
      publishDiscovery("flow_temp", "Temperatura wody", "temperature", "°C",
                       "measurement", "mdi:thermometer-water", 1, false);
      discTemp = true;
    }
    mqtt.publish(stateTopic("flow_temp").c_str(), String(v.flowTempC, 1).c_str(), true);
  }
  if (v.hasFlow) {
    if (!discFlow) {
      publishDiscovery("flow", "Przepływ", "volume_flow_rate", "m³/h",
                       "measurement", "mdi:water-pump", 3, false);
      discFlow = true;
    }
    mqtt.publish(stateTopic("flow").c_str(), String(v.flowM3h, 3).c_str(), true);
  }

  if (!discStatus) {
    publishDiscovery("meter_status", "Status licznika", "", "",
                     "", "mdi:alert-circle-outline", -1, true);
    discStatus = true;
  }
  char st[8];
  snprintf(st, sizeof(st), "0x%02X", v.status);
  mqtt.publish(stateTopic("meter_status").c_str(), st, true);

  mqtt.publish(stateTopic("rssi").c_str(), String(pkt.rssiDbm).c_str(), true);
}

// ===========================================================================
//  Telegram log
// ===========================================================================

void logTelegram(const WmbusPacket &pkt, bool ok, bool mine) {
  LogEntry &e = logBuf[logHead];
  e.ts = millis();
  e.rssi = pkt.rssiDbm;
  e.serial = pkt.serial;
  e.ok = ok;
  e.mine = mine;
  e.hex = bytesToHex(pkt.telegram, pkt.len);
  logHead = (logHead + 1) % TELEGRAM_LOG_SIZE;
  if (logCount < TELEGRAM_LOG_SIZE) logCount++;
  totalReceived++;
}

// Remember the latest state of every meter with a CRC-valid frame. FLOWIS+
// and wM-Bus ids live in separate spaces, so the protocol is part of the key.
void trackMeter(const WmbusPacket &pkt, bool decoded, const WmbusValues &v) {
  bool fl = pkt.mode == WMBUS_MODE_FLOWIS;
  MeterEntry *slot = nullptr;
  for (auto &m : meters)
    if (m.frames && m.serial == pkt.serial && (m.mode == WMBUS_MODE_FLOWIS) == fl) { slot = &m; break; }
  if (!slot) {  // new meter: a free slot, or evict the one heard longest ago
    slot = &meters[0];
    for (auto &m : meters) {
      if (!m.frames) { slot = &m; break; }
      if (millis() - m.lastSeen > millis() - slot->lastSeen) slot = &m;
    }
    *slot = MeterEntry();
    slot->serial = pkt.serial;
  }
  slot->mode = pkt.mode;
  slot->mfct[0] = pkt.mfct[0];
  slot->mfct[1] = pkt.mfct[1];
  slot->rssi = pkt.rssiDbm;
  slot->lastSeen = millis();
  slot->frames++;
  // FLOWIS+ type-2 frames carry no reading: keep the last one we had.
  if (decoded && v.hasVolume) {
    slot->hasVolume = true;
    slot->totalM3 = v.totalM3;
    slot->status = v.status;
  }
  slot->encrypted = !decoded && v.securityMode != 0;
}

// ===========================================================================
//  NTFY notification
// ===========================================================================

// Push a notification about a received telegram. `v` may be nullptr when the
// telegram could not be decoded (no key / wrong key) — we still report the
// reception, since knowing the meter is heard at all is the useful part.
void notifyNtfy(const WmbusPacket &pkt, const WmbusValues *v) {
  if (strlen(ntfyUrl) == 0 || WiFi.status() != WL_CONNECTED) return;

  bool flowis = (pkt.mode == WMBUS_MODE_FLOWIS);
  String body = "Licznik: " + String(pkt.serial) + " (" +
                (flowis ? String("FLOWIS+") : mfctToString(pkt.mfct)) +
                ", typ 0x" + String(pkt.devType, HEX) + ")\n";
  body += "Sygnal: " + String(pkt.rssiDbm) + " dBm\n";
  if (v && v->hasVolume) body += "Zuzycie: " + String(v->totalM3, 3) + " m3\n";
  if (v && v->hasTarget) body += "Data rozlicz.: " + String(v->targetM3, 3) + " m3\n";
  if (v && v->hasTemp)   body += "Temperatura: " + String(v->flowTempC, 1) + " C\n";
  if (v && v->hasFlow)   body += "Przeplyw: " + String(v->flowM3h, 3) + " m3/h\n";
  if (v) {
    body += "Status: 0x" + String(v->status, HEX) +
            (flowis ? " (bez szyfrowania)" : v->decrypted ? " (odszyfrowany)" : " (nieodszyfrowany)") + "\n";
  } else {
    body += flowis ? "Ramka bez odczytu (typ != 1)\n" : "Nieodszyfrowany (brak klucza AES)\n";
  }
  body += "Telegram: " + bytesToHex(pkt.telegram, pkt.len);

  HTTPClient http;
  if (!http.begin(ntfyUrl)) return;
  // Headers must stay ASCII — ntfy sends them verbatim.
  http.addHeader("Title", (v && v->hasVolume) ? "Flowis: odczyt wodomierza"
                                              : "Flowis: telegram odebrany");
  http.addHeader("Tags", "droplet");
  int code = http.POST(body);
  if (code <= 0) Serial.printf("ntfy blad: %s\n", http.errorToString(code).c_str());
  http.end();
}

// ===========================================================================
//  Web UI
// ===========================================================================

String htmlEscape(const String &in) {
  String o; o.reserve(in.length());
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '<') o += "&lt;"; else if (c == '>') o += "&gt;";
    else if (c == '&') o += "&amp;"; else if (c == '"') o += "&quot;";
    else if (c == '\'') o += "&#39;";
    else o += c;
  }
  return o;
}

String inputRow(const char *label, const char *name, const char *value, const char *type) {
  return "<label>" + String(label) + "<input type='" + type + "' name='" + name +
         "' value='" + htmlEscape(value) + "'></label>";
}

void handleRoot() {
  String p;
  p.reserve(4096);
  p += F("<!doctype html><html lang='pl'><head><meta charset='utf-8'>"
         "<meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<meta http-equiv='refresh' content='15'>"
         "<title>Flowis wM-Bus (SX1262)</title><style>"
         "body{font-family:system-ui,Arial,sans-serif;margin:0;background:#0f172a;color:#e2e8f0}"
         ".w{max-width:900px;margin:0 auto;padding:16px}"
         "h1{font-size:1.3rem}h2{font-size:1.05rem;margin-top:1.5rem;color:#93c5fd}"
         ".card{background:#1e293b;border-radius:10px;padding:14px 16px;margin:10px 0}"
         "table{width:100%;border-collapse:collapse;font-size:.85rem}"
         "th,td{text-align:left;padding:6px 8px;border-bottom:1px solid #334155}"
         "code{word-break:break-all;font-size:.78rem;color:#cbd5e1}"
         "label{display:block;margin:8px 0;font-size:.9rem}"
         "input{width:100%;padding:8px;margin-top:3px;border-radius:6px;border:1px solid #475569;"
         "background:#0f172a;color:#e2e8f0;box-sizing:border-box}"
         "button{background:#2563eb;color:#fff;border:0;border-radius:6px;padding:10px 16px;"
         "cursor:pointer;margin-top:8px}.muted{color:#94a3b8}.ok{color:#4ade80}.bad{color:#f87171}"
         "</style></head><body><div class='w'>");
  p += F("<h1>Flowis wM-Bus (SX1262) &rarr; Home Assistant</h1>");

  p += F("<div class='card'><h2 style='margin-top:0'>Status</h2><table>");
  p += "<tr><td>WiFi</td><td>" + WiFi.SSID() + " (" + WiFi.localIP().toString() +
       ", " + String(WiFi.RSSI()) + " dBm)</td></tr>";
  p += "<tr><td>Radio SX1262</td><td>" +
       String(radioOk ? "<span class='ok'>OK</span>" : "<span class='bad'>błąd init</span>") + "</td></tr>";
  p += "<tr><td>Profil radia</td><td>" + String(wmbusProfileName(wmbusProfile())) + " <span class='muted'>(" +
       (autoMode() ? (autoLocked ? String("auto — znaleziono licznik, zablokowany")
                                 : "auto — przełączanie co " + String(AUTO_DWELL_MS / 1000) + " s")
                   : String("stały")) + ")</span></td></tr>";
  p += "<tr><td>MQTT</td><td>" + String(mqtt.connected() ? "<span class='ok'>połączony</span>"
                                                        : "<span class='bad'>rozłączony</span>") +
       " (" + String(mqttServer) + ":" + String(mqttPort) + ")</td></tr>";
  p += "<tr><td>Filtr licznika</td><td>" +
       String(meterFilter ? String(meterFilter)
                          : String("<span class='bad'>brak — nic nie trafia do HA; wpisz numer z listy poniżej</span>")) +
       "</td></tr>";
  p += "<tr><td>Klucz AES</td><td>" +
       String(keySet ? "<span class='ok'>ustawiony — szyfrowany wM-Bus też dekodowany</span>"
                     : "<span class='muted'>brak — dekodowane FLOWIS+ i nieszyfrowany wM-Bus</span>") +
       "</td></tr>";
  p += "<tr><td>Odebrane telegramy</td><td>" + String(totalReceived) + "</td></tr>";
  p += "<tr><td>Ramki C1</td><td>" + String(wmbusC1Detected()) +
       " <span class='muted'>(T1 i C1 są obsługiwane jednocześnie)</span></td></tr>";
  p += "<tr><td>OTA</td><td>" +
       String(otaRunning ? "<span class='ok'>włączone</span> — w Arduino IDE port sieciowy " HOSTNAME
                         : "<span class='muted'>wyłączone (ustaw hasło OTA)</span>") + "</td></tr>";
  int heard = 0;
  for (auto &m : meters) if (m.frames) heard++;
  p += "<tr><td>Liczniki w zasięgu</td><td>" + String(heard) +
       " &nbsp; <a href='/meters'>zobacz stany wszystkich &rarr;</a></td></tr>";
  p += F("</table></div>");

  p += F("<div class='card'><h2 style='margin-top:0'>Ostatnie telegramy (surowe)</h2>");
  if (logCount == 0) {
    p += F("<p class='muted'>Brak odebranych telegramów.</p>");
  } else {
    p += F("<table><tr><th>kiedy</th><th>RSSI</th><th>nr licznika</th><th>status</th><th>telegram (hex)</th></tr>");
    for (int n = 0; n < logCount; n++) {
      int idx = (logHead - 1 - n + TELEGRAM_LOG_SIZE * 2) % TELEGRAM_LOG_SIZE;
      LogEntry &e = logBuf[idx];
      unsigned long age = (millis() - e.ts) / 1000UL;
      String tag = e.ok ? (e.mine ? "<span class='ok'>OK</span>" : "OK (inny)")
                        : "<span class='bad'>CRC/raw</span>";
      p += "<tr><td>" + String(age) + " s temu</td><td>" + String(e.rssi) +
           "</td><td>" + String(e.serial) + "</td><td>" + tag +
           "</td><td><code>" + e.hex + "</code></td></tr>";
    }
    p += F("</table>");
  }
  p += F("</div>");

  p += F("<div class='card'><h2 style='margin-top:0'>Konfiguracja</h2>"
         "<p class='muted' style='margin-top:0'>Numer licznika = dziesiętny numer z tabliczki "
         "(patrz kolumna „nr licznika” wyżej); dopóki pusty, nic nie trafia do Home Assistanta, "
         "a NTFY zgłasza każdy usłyszany licznik. Klucz AES = 32 znaki hex (16 bajtów), potrzebny "
         "tylko dla szyfrowanego wM-Bus. Lista telegramów jest ZAWSZE pełna — numer i klucz "
         "sterują tylko encjami w Home Assistant.</p>"
         "<form method='POST' action='/save'>");
  p += inputRow("Serwer MQTT", "server", mqttServer, "text");
  p += inputRow("Port MQTT", "port", mqttPort, "text");
  p += inputRow("Użytkownik MQTT", "user", mqttUser, "text");
  p += inputRow("Hasło MQTT", "pass", mqttPass, "password");
  p += inputRow("Prefiks tematów MQTT", "prefix", mqttPrefix, "text");
  p += inputRow("Numer licznika (puste = nic nie publikuj do HA)", "meter", meterIdStr, "text");
  p += inputRow("Klucz AES (32 znaki hex; tylko szyfrowany wM-Bus)", "key", aesKeyStr, "text");
  p += "<label>Tryb radia<select name='radio'>";
  const char *modes[][2] = {{"auto", "auto — przełączaj FLOWIS+ / wM-Bus (gdy nie wiesz)"},
                            {"flowis", "FLOWIS+ 868,00 MHz (nakładki 2401–2405, bez szyfrowania)"},
                            {"wmbus", "wM-Bus T1/C1 868,95 MHz (nowsze moduły OMS, szyfrowane)"}};
  for (auto &m : modes)
    p += "<option value='" + String(m[0]) + "'" + (strcmp(radioMode, m[0]) ? "" : " selected") +
         ">" + m[1] + "</option>";
  p += "</select></label>";
  p += inputRow("Hasło OTA (puste = OTA wyłączone)", "ota", otaPass, "password");
  p += inputRow("NTFY URL (np. http://ntfy.sh/moj-temat, puste = brak powiadomień)",
                "ntfy", ntfyUrl, "text");
  p += F("<button type='submit'>Zapisz</button></form>");
  p += F("<form method='POST' action='/portal' style='display:inline' "
         "onsubmit=\"return confirm('Uwaga: to WYCZYŚCI zapisaną sieć WiFi i zrestartuje "
         "urządzenie do portalu konfiguracji. Ustawienia MQTT / numer licznika / klucz AES "
         "zostaną zachowane. Kontynuować?')\">"
         "<button style='background:#475569'>Portal WiFi (reset WiFi)</button></form> ");
  p += F("<form method='POST' action='/restart' style='display:inline'>"
         "<button style='background:#475569'>Restart</button></form>");
  p += "<p class='muted'>Wersja " FW_VERSION "</p>";
  p += F("</div></div></body></html>");

  server.send(200, "text/html", p);
}

String ageText(unsigned long ms) {
  unsigned long sec = ms / 1000UL;
  if (sec < 120) return String(sec) + " s";
  if (sec < 7200) return String(sec / 60) + " min";
  return String(sec / 3600) + " h";
}

void handleMeters() {
  int idx[METER_TABLE_SIZE], n = 0;
  for (int i = 0; i < METER_TABLE_SIZE; i++) if (meters[i].frames) idx[n++] = i;
  std::sort(idx, idx + n, [](int a, int b) {
    return millis() - meters[a].lastSeen < millis() - meters[b].lastSeen;
  });

  String p;
  p.reserve(4096);
  p += F("<!doctype html><html lang='pl'><head><meta charset='utf-8'>"
         "<meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<meta http-equiv='refresh' content='15'>"
         "<title>Liczniki w zasięgu</title><style>"
         "body{font-family:system-ui,Arial,sans-serif;margin:0;background:#0f172a;color:#e2e8f0}"
         ".w{max-width:900px;margin:0 auto;padding:16px}h1{font-size:1.2rem}"
         ".card{background:#1e293b;border-radius:10px;padding:14px 16px;margin:10px 0;overflow-x:auto}"
         "table{width:100%;border-collapse:collapse;font-size:.85rem}"
         "th,td{text-align:left;padding:6px 8px;border-bottom:1px solid #334155;white-space:nowrap}"
         "td.num{text-align:right;font-variant-numeric:tabular-nums}"
         "tr.mine td{background:#14532d}"
         "a{color:#93c5fd}.muted{color:#94a3b8}"
         "</style></head><body><div class='w'>");
  p += F("<h1>Liczniki w zasięgu</h1><p class='muted'>Ostatni stan każdego licznika, od którego "
         "przyszła poprawna ramka — także sąsiadów. Tylko tutaj: do Home Assistanta trafia wyłącznie "
         "licznik o wpisanym numerze (zielony wiersz). Stan znika po restarcie urządzenia. "
         "Odświeżanie co 15 s. <a href='/'>&larr; powrót</a></p><div class='card'>");
  if (n == 0) {
    p += F("<p class='muted'>Jeszcze żaden licznik.</p>");
  } else {
    p += F("<table><tr><th>nr licznika</th><th>typ</th><th>zużycie [m³]</th><th>status</th>"
           "<th>RSSI</th><th>ostatnio</th><th>ramek</th></tr>");
    for (int k = 0; k < n; k++) {
      MeterEntry &m = meters[idx[k]];
      bool mine = meterFilter != 0 && m.serial == meterFilter;
      String type = m.mode == WMBUS_MODE_FLOWIS
                        ? String("FLOWIS+")
                        : String(wmbusModeName(m.mode)) + " " + mfctToString(m.mfct);
      String value = m.hasVolume ? String(m.totalM3, 3)
                     : m.encrypted ? String("<span class='muted'>zaszyfrowany</span>")
                                   : String("<span class='muted'>—</span>");
      char st[8];
      snprintf(st, sizeof(st), "0x%02X", m.status);
      p += String("<tr") + (mine ? " class='mine'" : "") + "><td>" + String(m.serial) +
           (mine ? " &#9733;" : "") + "</td><td>" + type + "</td><td class='num'>" + value +
           "</td><td>" + (m.hasVolume ? String(st) : String("—")) +
           "</td><td class='num'>" + String(m.rssi) + "</td><td>" +
           ageText(millis() - m.lastSeen) + " temu</td><td class='num'>" + String(m.frames) +
           "</td></tr>";
    }
    p += F("</table>");
  }
  p += F("</div></div></body></html>");
  server.send(200, "text/html", p);
}

void handleSave() {
  if (server.hasArg("server")) strlcpy(mqttServer, server.arg("server").c_str(), sizeof(mqttServer));
  if (server.hasArg("port"))   strlcpy(mqttPort,   server.arg("port").c_str(),   sizeof(mqttPort));
  if (server.hasArg("user"))   strlcpy(mqttUser,   server.arg("user").c_str(),   sizeof(mqttUser));
  if (server.hasArg("pass"))   strlcpy(mqttPass,   server.arg("pass").c_str(),   sizeof(mqttPass));
  if (server.hasArg("prefix")) strlcpy(mqttPrefix, server.arg("prefix").c_str(), sizeof(mqttPrefix));
  if (server.hasArg("meter"))  strlcpy(meterIdStr, server.arg("meter").c_str(),  sizeof(meterIdStr));
  if (server.hasArg("key"))    strlcpy(aesKeyStr,  server.arg("key").c_str(),    sizeof(aesKeyStr));
  if (server.hasArg("ntfy"))   strlcpy(ntfyUrl,    server.arg("ntfy").c_str(),   sizeof(ntfyUrl));
  if (server.hasArg("radio"))  strlcpy(radioMode,  server.arg("radio").c_str(),  sizeof(radioMode));
  if (server.hasArg("ota"))    strlcpy(otaPass,    server.arg("ota").c_str(),    sizeof(otaPass));
  saveConfig();
  applyOta();
  autoLocked = false;
  if (radioOk && !wmbusSetProfile(autoMode() ? wmbusProfile() : fixedProfile())) radioOk = false;
  applyCrypto();
  mqtt.disconnect();
  discTotal = discTarget = discTemp = discFlow = discRssi = discStatus = false;
  server.sendHeader("Location", "/");
  server.send(303);
}

void handlePortal() {
  server.send(200, "text/html",
              "<meta charset='utf-8'>Reset WiFi — urządzenie uruchomi portal po restarcie...");
  delay(300);
  wm.resetSettings();
  ESP.restart();
}
void handleRestart() { server.send(200, "text/html", "Restart..."); delay(300); ESP.restart(); }

// ===========================================================================
//  Setup / loop
// ===========================================================================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\nFlowis wM-Bus (SX1262) boot"));

  loadConfig();
  applyCrypto();

  WiFiManagerParameter pServer("server", "Serwer MQTT",     mqttServer, sizeof(mqttServer) - 1);
  WiFiManagerParameter pPort  ("port",   "Port MQTT",       mqttPort,   sizeof(mqttPort)   - 1);
  WiFiManagerParameter pUser  ("user",   "Uzytkownik MQTT", mqttUser,   sizeof(mqttUser)   - 1);
  WiFiManagerParameter pPass  ("pass",   "Haslo MQTT",      mqttPass,   sizeof(mqttPass)   - 1, "type='password'");
  WiFiManagerParameter pPrefix("prefix", "Prefiks MQTT",    mqttPrefix, sizeof(mqttPrefix) - 1);
  WiFiManagerParameter pMeter ("meter",  "Numer licznika",  meterIdStr, sizeof(meterIdStr) - 1);
  WiFiManagerParameter pKey   ("key",    "Klucz AES (hex)", aesKeyStr,  sizeof(aesKeyStr)  - 1);
  WiFiManagerParameter pNtfy  ("ntfy",   "NTFY URL",        ntfyUrl,    sizeof(ntfyUrl)    - 1);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  wm.setSaveConfigCallback(saveConfigCallback);
  wm.setConnectTimeout(30);
  wm.addParameter(&pServer);
  wm.addParameter(&pPort);
  wm.addParameter(&pUser);
  wm.addParameter(&pPass);
  wm.addParameter(&pPrefix);
  wm.addParameter(&pMeter);
  wm.addParameter(&pKey);
  wm.addParameter(&pNtfy);

  // Portal only on a fresh device; a 5 min timeout (then restart) keeps an
  // MQTT-less setup from sitting in AP mode forever after a power blip.
  bool forcePortal = (strlen(mqttServer) == 0) && !wm.getWiFiIsSaved();
  wm.setConfigPortalTimeout(300);
  bool wifiOk = forcePortal ? wm.startConfigPortal(HOSTNAME) : wm.autoConnect(HOSTNAME);
  if (!wifiOk) { Serial.println(F("WiFi failed, restarting")); delay(2000); ESP.restart(); }

  strlcpy(mqttServer, pServer.getValue(), sizeof(mqttServer));
  strlcpy(mqttPort,   pPort.getValue(),   sizeof(mqttPort));
  strlcpy(mqttUser,   pUser.getValue(),   sizeof(mqttUser));
  strlcpy(mqttPass,   pPass.getValue(),   sizeof(mqttPass));
  strlcpy(mqttPrefix, pPrefix.getValue(), sizeof(mqttPrefix));
  strlcpy(meterIdStr, pMeter.getValue(),  sizeof(meterIdStr));
  strlcpy(aesKeyStr,  pKey.getValue(),    sizeof(aesKeyStr));
  strlcpy(ntfyUrl,    pNtfy.getValue(),   sizeof(ntfyUrl));
  if (shouldSaveConfig) { saveConfig(); shouldSaveConfig = false; }
  applyCrypto();

  Serial.print(F("WiFi OK, IP: ")); Serial.println(WiFi.localIP());
  applyOta();

  radioOk = wmbusBegin(fixedProfile());   // auto starts on FLOWIS+
  lastHop = millis();
  if (!radioOk) Serial.println(F("WARNING: SX1262 init nieudany — sprawdź moduł/antenę"));

  server.on("/", handleRoot);
  server.on("/meters", handleMeters);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/portal", HTTP_POST, handlePortal);
  server.on("/restart", HTTP_POST, handleRestart);
  server.begin();
  Serial.println(F("Web UI started"));

  mqttConnect();
}

void loop() {
  server.handleClient();
  if (otaRunning) ArduinoOTA.handle();

  if (strlen(mqttServer) > 0) {
    if (mqtt.connected()) mqtt.loop();
    else if (millis() - lastMqttRetry > MQTT_RETRY_MS) { lastMqttRetry = millis(); mqttConnect(); }
  }

  // Radio — retry init if it failed at boot.
  if (!radioOk) {
    if (millis() - lastRadioCheck > 3000) {
      lastRadioCheck = millis();
      Serial.println(F("SX1262 nie działa — ponawiam init"));
      radioOk = wmbusBegin(fixedProfile());
    }
    return;
  }

  if (autoMode() && !autoLocked && millis() - lastHop > AUTO_DWELL_MS) {
    lastHop = millis();
    // A failed switch leaves the radio in standby: force a re-init instead.
    if (!wmbusSetProfile(wmbusProfile() == RADIO_FLOWIS ? RADIO_WMBUS : RADIO_FLOWIS)) radioOk = false;
  }

  WmbusPacket pkt;
  WmbusPollResult r = wmbusPoll(pkt);
  if (!wmbusChipPresent()) { radioOk = false; return; }  // re-arm failed
  if (r == WMBUS_NONE) return;

  bool ok = (r == WMBUS_OK);

  if (pkt.len == 0) {  // pure noise / nothing usable
    if (millis() - lastNoiseNote > 5000) {
      lastNoiseNote = millis();
      Serial.printf("radio żyje, szum/brak ramki (%d dBm)\n", pkt.rssiDbm);
    }
    return;
  }

  // Publish only the configured meter: FLOWIS+ is unencrypted, so without a
  // filter every neighbour's reading would overwrite the same retained,
  // total_increasing HA sensor.
  bool mine = ok && meterFilter != 0 && pkt.serial == meterFilter;
  logTelegram(pkt, ok, mine || (ok && meterFilter == 0));

  Serial.printf("RX %s [%s] serial=%lu rssi=%d len=%d %s\n",
                ok ? "OK" : "ERR", wmbusModeName(pkt.mode),
                (unsigned long)pkt.serial, pkt.rssiDbm,
                pkt.len, pkt.mode == WMBUS_MODE_FLOWIS ? "FLOWIS+" : mfctToString(pkt.mfct).c_str());

  if (!ok) return;

  // FLOWIS+ and unencrypted wM-Bus decode without a key; wmbusDecode() itself
  // refuses encrypted wM-Bus when no key is set.
  WmbusValues v;
  bool decoded = wmbusDecode(pkt, aesKey, keySet, v);
  trackMeter(pkt, decoded, v);   // web UI only, every meter

  if (mine) {
    // Lock "auto" only on the configured meter: in a block of flats a
    // neighbour's meter would otherwise stop the hopping for good.
    if (autoMode() && !autoLocked) {
      autoLocked = true;
      Serial.printf("auto: licznik %lu znaleziony — zostaję na %s\n",
                    (unsigned long)pkt.serial, wmbusProfileName(wmbusProfile()));
    }
    if (decoded) {
      if (v.hasVolume) Serial.printf("  total = %.3f m3\n", v.totalM3);
      publishValues(pkt, v);
    } else if (keySet) {
      Serial.println(F("  decode failed (zły klucz / nieobsługiwany tryb)"));
    }
  }
  // ntfy: our meter — or, while no meter is configured, any meter heard, so the
  // user learns what is on the air and which number to enter.
  if (mine || meterFilter == 0) notifyNtfy(pkt, decoded ? &v : nullptr);
}
