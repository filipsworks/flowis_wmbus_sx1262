# Flowis → Home Assistant (XIAO ESP32-S3 + Wio-SX1262)

Szkic Arduino IDE dla **Seeed XIAO ESP32-S3 + Wio-SX1262** (RadioLib, tryb FSK), który
odbiera odczyty wodomierzy **Apator FLOWIS** i wystawia je do Home Assistanta przez MQTT
discovery. Ma web UI z konfiguracją, listą 10 ostatnich telegramów, tabelą stanów wszystkich
liczników w zasięgu (`/meters`) i opcjonalne powiadomienia NTFY.

> Skompilowano dla `esp32:esp32:XIAO_ESP32S3` (rdzeń esp32 3.3.10, RadioLib 7.7.1).
> Test hosta: `test/` — 57/57 (patrz niżej).

---

## Najważniejsze: FLOWIS+ to NIE jest wM-Bus

Na rynku są dwie generacje nakładek radiowych na wodomierze Flowis — i nadają zupełnie inaczej:

| nakładka | protokół | częstotliwość | szyfrowanie | profil radia |
|---|---|---|---|---|
| **FLOWIS+** (numery 2401…2405, fw v24.x) | **własnościowy** — sync `D391 D391`, CRC-16 0x8005 | **868,00 MHz** | **brak** | `FLOWIS+` |
| nowsze „Flow Meters PRO”, moduł radiowy OMS (v1.12.x) | **wM-Bus T1** (OMS) | 868,95 MHz | AES (mode 5) — klucz od dostawcy | `wM-Bus` |

Źródła: dekoder rtl_433 [`flowis.c`](https://github.com/merbanan/rtl_433/blob/master/src/devices/flowis.c)
(„Flowis flow meters”, nr 237) i jego reverse engineering w
[merbanan/rtl_433#2317](https://github.com/merbanan/rtl_433/issues/2317) („very strong, stable signal
on 868.01 MHz”, 100 kbps FSK); wątek [forum.arturhome.pl — wmbusmeters i FLOWIS+](https://forum.arturhome.pl/t/wmbusmeters-i-flowis/6544)
(„rtl433 bez problemu czyta starsze Flowisy z customowym protokołem. Ja mam niestety nowsze, które
nadają standardowym WMBUS, ale za to szyfrowane”; wywołanie `rtl_433 -R 237 -f 868M -s 1000k`).

Poprzednie wersje tego szkicu słuchały tylko wM-Bus na 868,95 MHz — **nakładki FLOWIS+ były dla nich
niewidoczne** (inna częstotliwość i inny sync).

### Tryb radia (web UI → „Tryb radia”)

Jedno SX1262 nie słucha na 868,00 i 868,95 MHz naraz, więc:

- **`auto`** (domyślny) — przełącza FLOWIS+ ↔ wM-Bus co 17 s. Okres 34 s nie ma wspólnego dzielnika
  z typowymi interwałami liczników (10/30/60/120 s), więc faza się przesuwa i okresowy nadajnik nie
  może stale trafiać w „drugie” okno. Gdy przyjdzie poprawna ramka **od licznika o wpisanym numerze**,
  auto zostaje na tym profilu (status w web UI: „znaleziono licznik, zablokowany”). Bez wpisanego numeru
  nie blokuje się — sąsiedni licznik wM-Bus w bloku nie „przejmie” odbiornika.
- **`flowis`** — tylko FLOWIS+ 868,00 MHz.
- **`wmbus`** — tylko wM-Bus T1/C1 868,95 MHz.

Gdy już wiesz, jaki masz licznik, ustaw tryb stały — odbiór jest wtedy ciągły (a nie 50 % czasu).

---

## Sprzęt

Wio-SX1262 wpina się na XIAO przez **złącze B2B** — nic nie lutujesz. Wkręć **antenę 868 MHz**.
Piny (w `wmbus_sx1262.cpp`):

| funkcja | GPIO | źródło |
|---|---|---|
| NSS / DIO1 / RESET / BUSY | 41 / 39 / 42 / 40 | RadioBoards, BSP Seeed (one_channel_hub) |
| SCK / MISO / MOSI | 7 / 8 / 9 | jw. |
| **RXEN (przełącznik antenowy)** | **38 — musi być HIGH przy odbiorze** | Meshtastic `SX126X_RXEN 38`; BSP Seeed `antenna_sw = 38` |
| przełącznik TX | DIO2 | Meshtastic / BSP Seeed |
| TCXO | DIO3, 1,8 V | Meshtastic |

**GPIO38 to druga przyczyna „głuchoty”** wcześniejszych wersji: bez ustawienia RXEN tor odbiorczy jest
praktycznie odcięty. W Meshtastic dodanie tego pinu dało
[„20–40 dB stronger signals”](https://github.com/meshtastic/firmware/pull/5517). Teraz RadioLib ustawia go
w HIGH przy każdym `startReceive()`.

## Biblioteki (Menedżer bibliotek)

- **esp32** (Espressif) — płytka „XIAO_ESP32S3”
- **RadioLib** (Jan Gromes), **WiFiManager** (tzapu), **PubSubClient**, **ArduinoJson v7**,
  **Crypto** (Rhys Weatherley)
- LittleFS i WebServer są w rdzeniu ESP32.

## Uruchomienie

Pierwszy boot → portal WiFi `flowis-sx1262` (WiFi, MQTT, numer licznika, klucz AES) → potem web UI pod
`http://<ip>/` (tam też „Tryb radia” i NTFY).

> Przycisk **„Portal WiFi (reset WiFi)”** ma potwierdzenie — **czyści zapisaną sieć WiFi** i restartuje do
> portalu. MQTT / numer licznika / klucz AES / tryb radia (LittleFS) zostają.

## Aktualizacja przez sieć (OTA)

Pierwsze wgranie zawsze przez USB. Potem w web UI ustaw **Hasło OTA** i zapisz — w Arduino IDE pojawi się
port sieciowy **`flowis-sx1262 at <ip>`** (Narzędzia → Port). Wgrywasz normalnie, IDE zapyta o hasło.

- Puste hasło = OTA wyłączone. Bez hasła każdy w sieci mógłby wgrać własny firmware.
- Sprzętowo nic nie trzeba: domyślny układ partycji XIAO (`default_8MB`) ma dwie przegródki na aplikację
  po 3,3 MB, a firmware zajmuje ok. 1,3 MB. **Nie wybieraj** schematu „TinyUF2 … No OTA”.
- Port nie widać? Komputer i XIAO muszą być w tej samej podsieci (IDE szuka przez mDNS), a zapora nie może
  blokować przychodzących połączeń do IDE.
- Web UI nie ma logowania — kto ma dostęp do strony, zobaczy też hasło OTA. W zaufanej sieci domowej to
  wystarczy.

## Pola konfiguracji

- **Numer licznika** — numer z tabliczki (ten sam, co w kolumnie „nr licznika” w web UI). Dla FLOWIS+
  to 32-bitowe ID z ramki (np. `240234918`), dla wM-Bus numer z pola A (BCD).
  **Dopóki jest pusty, nic nie trafia do Home Assistanta** — FLOWIS+ nie jest szyfrowany, więc bez
  filtra odczyty wszystkich sąsiadów nadpisywałyby tę samą encję (i psuły statystyki `total_increasing`).
  W tym czasie NTFY zgłasza każdy usłyszany licznik, a lista telegramów pokazuje jego numer — przepisz
  swój. Dopiero z numerem tryb `auto` może się zablokować na właściwym profilu.
- **Klucz AES** — 32 znaki hex (16 bajtów). **Potrzebny tylko dla szyfrowanego wM-Bus.** FLOWIS+ i
  nieszyfrowany wM-Bus dekodują się bez klucza.
- **Tryb radia** — patrz wyżej.
- **NTFY URL** — np. `http://ntfy.sh/dluga-losowa-nazwa`; puste = brak powiadomień. Bez numeru
  licznika przychodzi powiadomienie o każdym usłyszanym liczniku (tryb „szukam”), z numerem — tylko o Twoim.

**Lista surowych telegramów pokazuje zawsze wszystko**, co przeszło ramkowanie — numer licznika i klucz
sterują tylko tym, co trafia do Home Assistanta.

## Co trafia do Home Assistanta

| encja | FLOWIS+ | wM-Bus |
|---|---|---|
| Zużycie wody (`total_m3`, m³) | ✔ (ramka typu 1, bez klucza) | ✔ (po odszyfrowaniu lub bez szyfrowania) |
| *warunek* | *wpisany numer licznika* | *wpisany numer licznika* |
| Status licznika (`meter_status`) | bajt alarmu | bajt statusu TPL |
| RSSI (`rssi`, dBm) | ✔ | ✔ |
| Temperatura / przepływ / data rozliczeniowa | — | jeśli licznik nadaje |

FLOWIS+ nadaje na zmianę ramki typu 1 (z odczytem) i typu 2 (bez odczytu — w liście telegramów, bez
encji). Format ramki FLOWIS+ (za syncem): `L | typ | ID(4, LE) | ? | czas(4) | objętość(3, LE, litry) |
backflow | alarm | … | CRC(2, BE)`.

## Formaty na liście telegramów

- **FLOWIS+**: `L`, ładunek i CRC — ten sam zapis, co w rtl_433 `-X` / issue #2317, np.
  `1201A6B1510E81F996625CF1000000110000A57168` → licznik 240234918, 0,241 m³.
- **wM-Bus**: warstwa łącza + aplikacji po usunięciu CRC bloków, od pola L — format do wklejenia w
  analizator [wmbusmeters](https://wmbusmeters.org/analyze/).

## Liczniki w zasięgu: `/meters`

Link „zobacz stany wszystkich” na stronie głównej. Jeden wiersz na licznik, od którego przyszła ramka z
poprawnym CRC — **także sąsiadów, niezależnie od wpisanego numeru**: numer, typ (`FLOWIS+`, `T1 APA` …),
ostatnie zużycie w m³, status, RSSI, kiedy ostatnio i ile ramek. Twój licznik jest na zielono z ★.

- **Tylko na stronie.** Do Home Assistanta (i NTFY, gdy numer jest wpisany) dalej trafia wyłącznie
  licznik o wpisanym numerze.
- Szyfrowany wM-Bus bez klucza: „zaszyfrowany” zamiast zużycia. FLOWIS+ ramka typu 2 (bez odczytu) nie
  kasuje ostatniego zużycia, tylko odświeża „ostatnio”.
- Do 32 liczników; gdy jest pełno, nowy wypiera najdawniej słyszany. Stan jest w RAM — znika po restarcie.
- W trybie `auto` licznik słychać tylko w oknie jego profilu, więc „ostatnio” rośnie skokami po ~17 s.

## Strojenie radia

`#define` na górze `wmbus_sx1262.cpp`:

- `FLOWIS_FREQ` (868,00 MHz), `FLOWIS_RXBW` (312 kHz) — pomiar w issue #2317: 868,01 MHz.
- `WMBUS_FREQ` (868,95), `WMBUS_RXBW` (234,3) — jak w sterowniku SzczepanLeona.
- `WMBUS_DEV` (50 kHz) — oba profile.
- `FLOWIS_RX_LEN` (64) / `WMBUS_RX_LEN` (255) — ile bajtów łapiemy po syncu.
- `AUTO_DWELL_MS` (17 s) w `.ino` — czas na jednym profilu w trybie `auto`.

## Test hosta (`test/`)

Kompiluje **niezmieniony** `wmbus_sx1262.cpp` ze stubami RadioLib i sprawdza (57 przypadków):
konfigurację obu profili (częstotliwość, sync, preambuła, RXEN=38 przed `startReceive`, kalibracja
obrazu, RSSI z chwili syncu), **prawdziwe ramki FLOWIS+ z issue #2317**
(CRC, ID, objętość, ramki z błędem CRC odrzucone), telegramy wM-Bus z symulacji wmbusmeters
przekodowane niezależnym koderem (T1 parzyste/nieparzyste, z ogonem szumu i bez, C1 format A i B,
dekodowanie bez klucza, CI inne niż DIF/VIF nie są parsowane) oraz 2000 przechwyceń czystego szumu
(żadne nie przyjęte).

```
cd test && clang++ -std=c++17 -I stubs -Wall test.cpp -o /tmp/wmbus_test && /tmp/wmbus_test
```

Poprzedni dekoder na tych samych danych: **każda** ramka T1 z 255-bajtowego przechwycenia SX1262
była odrzucana jako szum (błędy 3-of-6 liczone w szumie za ramką), a ramki o nieparzystej długości
traciły ostatni bajt. Obie rzeczy są poprawione (dekodowanie 3-of-6 bit po bicie, tylko tyle, ile ma ramka).

## Status / uczciwie

- ✅ Protokół FLOWIS+ i jego CRC sprawdzone na prawdziwych przechwyceniach; RXEN zgodny z Meshtastic
  i BSP Seeeda; dekoder wM-Bus sprawdzony testem hosta.
- ✅ Odbiór FLOWIS+ potwierdzony na żywym liczniku (bez szyfrowania).
- Gdyby po zmianie anteny / miejsca nic nie przychodziło: w Serialu widać `radio żyje, szum/brak ramki`;
  dalej `FLOWIS_FREQ` (spróbuj 868,05 / 867,95) i `FLOWIS_RXBW` 373,6.
- Klucza AES nie da się „wydobyć” radiowo — dla szyfrowanego wM-Bus musi go dać dostawca.

## Ostrzeżenia kompilatora

Własny kod kompiluje się bez ostrzeżeń. Szum z WiFiManagera i rdzenia ESP32 wycisza
**Preferencje → „Kompiluj ostrzeżenia” → `Domyślne`**. Zostaje jeden celowy `#warning` RadioLib o USB CDC —
nieszkodliwy (dotyczy deep-sleep), nie wyłączaj USB CDC, bo stracisz Serial Monitor.

## Diagnostyka (Serial @115200)

- `SX1262 ready: FLOWIS+ 868.00 MHz` — radio wystartowało (w trybie `auto` zawsze od FLOWIS+).
- `SX1262 beginFSK failed: N` / `SX1262 profile … failed` — błąd RadioLib (kod N); firmware ponawia init.
- `RX OK [FLOWIS] serial=… len=21 FLOWIS+` + `total = … m3` — odebrany i zdekodowany odczyt.
- `auto: licznik … znaleziony — zostaję na …` — auto zablokowane na właściwym profilu.
- `radio żyje, szum/brak ramki` — fałszywe dopasowanie syncu, nic do zdekodowania.
