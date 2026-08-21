#include <Arduino.h>
#include <esp_system.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <time.h>
#include <math.h>
#include "secrets.h"

/*
  ESP32 + MAX485 + ADJ Saber Spot RGBW
  -------------------------------------
  配線（送信専用）
    ESP32 GPIO18       -> MAX485 DI
    ESP32 GPIO4        -> MAX485 DE と /RE（短絡）
    ESP32 GND          -> MAX485 GND -> XLR pin 1
    MAX485 B           -> XLR pin 2（DMX-）
    MAX485 A           -> XLR pin 3（DMX+）

  ADJ Saber Spot RGBW 側
    DMX address : 001
    Channel mode: 5CH

  5CH割り当て
    CH1 Red
    CH2 Green
    CH3 Blue
    CH4 White
    CH5 Master Dimmer

  BREAK / MAB生成
    DEをLOWにはしない。
    UARTを一度停止し、GPIO18を直接LOWにして120usのBREAKを作る。
    続けてGPIO18をHIGHにして20us以上のMABを作り、
    UARTを250,000 baud / 8N2で再開してDMXデータを送る。
    実機ではUART再開時間を含むMABが約300usとなり、正常に認識済み。
*/

// -----------------------------------------------------------------------------
// Wi-Fi設定は secrets.h に分離する。
// -----------------------------------------------------------------------------

WebServer server(80);

// -----------------------------------------------------------------------------
// NTP / 日本時間
// -----------------------------------------------------------------------------

const char* NTP_SERVER = "ntp.nict.jp";
constexpr long GMT_OFFSET_SEC = 9L * 60L * 60L;
constexpr int DAYLIGHT_OFFSET_SEC = 0;

// -----------------------------------------------------------------------------
// DMX
// -----------------------------------------------------------------------------

constexpr int DMX_UART_NUMBER = 1;
constexpr int DMX_TX_PIN = 18;
constexpr int DMX_EN_PIN = 4;

constexpr uint32_t DMX_DATA_BAUD = 250000;
constexpr uint32_t DMX_BREAK_US = 120;
constexpr uint32_t DMX_MAB_US = 20;
constexpr uint32_t DMX_FRAME_INTERVAL_MS = 25;

// 実機との互換性を優先し、Start Code + 512 slotを送る。
// Saber Spot（address 001 / 5CH）に必要なCH1～CH5を含む。
constexpr size_t DMX_SLOT_COUNT = 512;
constexpr size_t DMX_PACKET_BYTES = DMX_SLOT_COUNT + 1;

HardwareSerial DMXSerial(DMX_UART_NUMBER);
uint8_t dmxData[DMX_PACKET_BYTES] = {0};
bool dmxUartStarted = false;

// 起動時は動作確認しやすいよう「赤100%、Master 100%」の手動モード。
uint8_t red = 255;
uint8_t green = 0;
uint8_t blue = 0;
uint8_t white = 0;
uint8_t masterDimmer = 255;

// mode: 0=AUTO, 1=MANUAL
int mode = 1;

// weather: 0=Sunny, 1=Cloudy, 2=Rain
int weather = 0;

// GPIO25 = 5V fan control (via external MOSFET)
constexpr int FAN_PIN = 25;
constexpr int FAN_PWM_CHANNEL = 0;
constexpr int FAN_PWM_FREQUENCY = 50;
constexpr int FAN_PWM_RESOLUTION = 8;
constexpr uint8_t FAN_DEFAULT_SPEED = 0;
uint8_t fanSpeed = FAN_DEFAULT_SPEED;

// GPIO26 = 5V dosing pump control (via external MOSFET)
constexpr int DOSING_PIN = 26;
constexpr uint32_t DOSING_MAX_RUN_MS = 10000;
constexpr uint32_t DOSING_STEP_MS = 500;
constexpr uint32_t DOSING_DEFAULT_MS = 3000;

bool dosingActive = false;
uint32_t dosingStartedMs = 0;
uint32_t dosingStopMs = 0;
uint32_t dosingDurationMs = DOSING_DEFAULT_MS;

String phase = "手動";
String timeStr = "--:--";
String dateStr = "----/--/-- (-)";

constexpr uint8_t MAX_SAVED_NETWORKS = 5;
constexpr uint32_t WIFI_RECONNECT_RETRY_MS = 45000;
constexpr uint32_t WIFI_SAVED_NETWORK_TIMEOUT_MS = 10000;
constexpr uint32_t WIFI_SCAN_TIMEOUT_MS = 15000;
constexpr uint8_t WIFI_SETUP_AP_CHANNEL = 1;
const char* SETUP_AP_PASSWORD = "paludarium";

// 同じプログラムを複数台へ書き込んでも名前が衝突しないよう、
// setup()でESP32固有MACの末尾6桁から自動生成する。
// 例:
//   deviceId    = honma-paludarium-a1b2c3（将来のAWS IoT Thing名）
//   hostname    = paludarium-a1b2c3      （mDNS）
//   setupApSsid = Paludarium-A1B2C3-Setup（初期設定用Wi-Fi）
String deviceId = "";
String hostname = "";
String setupApSsid = "";

struct SavedNetwork {
  String ssid;
  String password;
};

SavedNetwork savedNetworks[MAX_SAVED_NETWORKS];
uint8_t savedNetworkCount = 0;
uint8_t savedNetworkAttemptIndex = 0;
bool apMode = false;
bool wifiConnected = false;
String currentWifiSsid = "";
int currentWifiRssi = 0;
String currentIp = "";
uint32_t lastWifiRetryMs = 0;
uint32_t lastSavedScanMs = 0;
uint32_t lastDmxMs = 0;
uint32_t lastTimeMs = 0;
uint32_t lastLightMs = 0;
uint32_t pendingWifiReconnectMs = 0;
String pendingWifiReconnectSsid = "";
String pendingWifiReconnectPassword = "";
DNSServer dnsServer;
wl_status_t previousWifiStatus = WL_IDLE_STATUS;

struct ScanResult {
  String ssid;
  int32_t rssi;
  bool secure;
};

enum WifiState {
  WIFI_STATE_IDLE,
  WIFI_STATE_SCAN_START,
  WIFI_STATE_SCANNING,
  WIFI_STATE_SELECT_NETWORK,
  WIFI_STATE_CONNECTING,
  WIFI_STATE_CONNECTED,
  WIFI_STATE_FAILED
};

WifiState wifiState = WIFI_STATE_IDLE;
String wifiConnectSsid = "";
String wifiConnectPass = "";
String wifiScanJsonCache = "[]";
bool wifiScanPending = false;
uint32_t wifiStateChangedMs = 0;

void initializeDeviceIdentity() {
  const uint64_t mac = ESP.getEfuseMac();
  const uint32_t suffixValue = static_cast<uint32_t>(mac & 0xFFFFFFULL);

  char suffixUpper[7];
  char suffixLower[7];
  snprintf(suffixUpper, sizeof(suffixUpper), "%06lX", static_cast<unsigned long>(suffixValue));
  snprintf(suffixLower, sizeof(suffixLower), "%06lx", static_cast<unsigned long>(suffixValue));

  deviceId = "honma-paludarium-" + String(suffixLower);
  hostname = "paludarium-" + String(suffixLower);
  setupApSsid = "Paludarium-" + String(suffixUpper) + "-Setup";

  Serial.println("Device identity initialized");
  Serial.print("Device ID: ");
  Serial.println(deviceId);
  Serial.print("Local URL after Wi-Fi connection: http://");
  Serial.print(hostname);
  Serial.println(".local/");
  Serial.print("Setup AP SSID: ");
  Serial.println(setupApSsid);
}

String jsonEscape(const String& input);

// -----------------------------------------------------------------------------
// DMX処理
// -----------------------------------------------------------------------------

void updateDMXBuffer() {
  dmxData[0] = 0x00;          // DMX NULL Start Code
  dmxData[1] = red;           // CH1 Red
  dmxData[2] = green;         // CH2 Green
  dmxData[3] = blue;          // CH3 Blue
  dmxData[4] = white;         // CH4 White
  dmxData[5] = masterDimmer;  // CH5 Master Dimmer

  // CH6～CH512は0のままにする。
  for (size_t i = 6; i < DMX_PACKET_BYTES; ++i) {
    dmxData[i] = 0;
  }
}

void sendDMXFrame() {
  // DE=HIGH：BREAK、MAB、データ送信中を通してMAX485を送信可能にする。
  digitalWrite(DMX_EN_PIN, HIGH);

  // 初回以外は、前回フレームの送信完了を待ってUARTを停止する。
  if (dmxUartStarted) {
    DMXSerial.flush();
    DMXSerial.end();
    dmxUartStarted = false;
  }

  // TXをGPIOとして直接操作し、確実なBREAKを作る。
  pinMode(DMX_TX_PIN, OUTPUT);
  digitalWrite(DMX_TX_PIN, LOW);
  delayMicroseconds(DMX_BREAK_US);

  // BREAK後にHIGHへ戻してMABを作る。
  digitalWrite(DMX_TX_PIN, HIGH);
  delayMicroseconds(DMX_MAB_US);

  // RXは使わず、TX=GPIO18で250,000 baud / 8N2を再開する。
  // begin()中もTXはアイドルHIGHなので、その時間はMABに加算される。
  DMXSerial.begin(DMX_DATA_BAUD, SERIAL_8N2, -1, DMX_TX_PIN);
  dmxUartStarted = true;

  // Start Code + CH1～CH512を送信する。
  DMXSerial.write(dmxData, DMX_PACKET_BYTES);
  DMXSerial.flush();
}

void setupDMX() {
  pinMode(DMX_EN_PIN, OUTPUT);
  digitalWrite(DMX_EN_PIN, HIGH);

  updateDMXBuffer();
  sendDMXFrame();
  lastDmxMs = millis();
}

// -----------------------------------------------------------------------------
// 時刻・自動照明
// -----------------------------------------------------------------------------

struct LightKeyframe {
  int minute;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t w;
  uint8_t master;
};

// AUTOモードの光量を簡単に調整できるよう、時刻ごとのキーフレームをまとめて管理する。
// 各値は 00:00〜24:00 の一日内で線形補間される。
const LightKeyframe AUTO_KEYFRAMES[] = {
  { 0,   20,  20,  80,   0, 150 },
  { 330, 20,  20,  80,   0, 150 },
  { 360,  80,  90,  90,  25, 180 },
  { 420, 200, 180, 160, 120, 220 },
  { 600, 255, 255, 255, 255, 255 },
  { 960, 255, 228, 190, 160, 255 },
  { 1080, 255, 120,  80,  40, 255 },
  { 1170, 180,  60,  20,  12, 180 },
  { 1260,  40,  30,  80,   8, 180 },
  { 1440, 20,  20,  80,   0, 150 },
};
constexpr size_t AUTO_KEYFRAME_COUNT = sizeof(AUTO_KEYFRAMES) / sizeof(AUTO_KEYFRAMES[0]);

const uint32_t SETTINGS_SAVE_DELAY_MS = 2000;

Preferences preferences;
bool settingsDirty = false;
uint32_t lastSettingsChangeMs = 0;

uint8_t interpolate8(uint8_t from, uint8_t to, float t) {
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  return static_cast<uint8_t>(lroundf(from + (to - from) * t));
}

uint8_t scale8(uint8_t value, float factor) {
  int result = static_cast<int>(lroundf(value * factor));
  if (result < 0) result = 0;
  if (result > 255) result = 255;
  return static_cast<uint8_t>(result);
}

void getAutoLightFromKeyframes(int totalMinutes, uint8_t& outR, uint8_t& outG, uint8_t& outB, uint8_t& outW, uint8_t& outMaster) {
  if (totalMinutes < 0) totalMinutes = 0;
  if (totalMinutes >= 1440) totalMinutes = 1440;

  for (size_t i = 0; i < AUTO_KEYFRAME_COUNT - 1; ++i) {
    const LightKeyframe& from = AUTO_KEYFRAMES[i];
    const LightKeyframe& to = AUTO_KEYFRAMES[i + 1];

    if (totalMinutes >= from.minute && totalMinutes <= to.minute) {
      const float span = static_cast<float>(to.minute - from.minute);
      const float t = (span > 0.0f) ? (static_cast<float>(totalMinutes - from.minute) / span) : 0.0f;

      outR = interpolate8(from.r, to.r, t);
      outG = interpolate8(from.g, to.g, t);
      outB = interpolate8(from.b, to.b, t);
      outW = interpolate8(from.w, to.w, t);
      outMaster = interpolate8(from.master, to.master, t);
      return;
    }
  }

  const LightKeyframe& last = AUTO_KEYFRAMES[AUTO_KEYFRAME_COUNT - 1];
  outR = last.r;
  outG = last.g;
  outB = last.b;
  outW = last.w;
  outMaster = last.master;
}

bool readLocalTime(struct tm& timeinfo) {
  // NTP未同期時にloopを長時間止めない。
  return getLocalTime(&timeinfo, 10);
}

void updateTimeDisplay() {
  struct tm timeinfo;

  if (!readLocalTime(timeinfo)) {
    timeStr = "--:--";
    dateStr = "----/--/-- (-)";
    if (mode == 0) phase = "時刻未同期";
    return;
  }

  char buffer[6];
  snprintf(buffer, sizeof(buffer), "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
  timeStr = buffer;

  static const char* WEEKDAYS[] = {"日", "月", "火", "水", "木", "金", "土"};
  char dateBuffer[24];
  snprintf(
    dateBuffer,
    sizeof(dateBuffer),
    "%04d/%02d/%02d (%s)",
    timeinfo.tm_year + 1900,
    timeinfo.tm_mon + 1,
    timeinfo.tm_mday,
    WEEKDAYS[timeinfo.tm_wday]
  );
  dateStr = dateBuffer;

  if (mode == 1) {
    phase = "手動";
  } else if (timeinfo.tm_hour < 6) {
    phase = "夜";
  } else if (timeinfo.tm_hour == 6 && timeinfo.tm_min < 30) {
    phase = "夜明け";
  } else if (timeinfo.tm_hour < 18) {
    phase = "昼";
  } else if (timeinfo.tm_hour == 18 && timeinfo.tm_min < 30) {
    phase = "夕焼け";
  } else if (timeinfo.tm_hour < 21) {
    phase = "夕方";
  } else {
    phase = "夜";
  }
}

void updateAutomaticLight() {
  if (mode == 1) return;

  struct tm timeinfo;
  if (!readLocalTime(timeinfo)) return;

  const int currentMinute = timeinfo.tm_hour * 60 + timeinfo.tm_min;

  uint8_t baseR = 0;
  uint8_t baseG = 0;
  uint8_t baseB = 0;
  uint8_t baseW = 0;
  uint8_t baseMaster = 255;

  getAutoLightFromKeyframes(currentMinute, baseR, baseG, baseB, baseW, baseMaster);

  if (weather == 1) {
    // Cloudy：約50秒周期で明るさを 60～100% の範囲で緩やかに変化させる。
    const float cloudFactor = sinf(millis() / 8000.0f) * 0.20f + 0.80f;
    baseR = scale8(baseR, cloudFactor);
    baseG = scale8(baseG, cloudFactor);
    baseB = scale8(baseB, cloudFactor);
    baseW = scale8(baseW, cloudFactor);
    baseMaster = scale8(baseMaster, cloudFactor);
  } else if (weather == 2) {
    // Rain：キーフレーム値に対して暗くし、やや青寄りにする。
    baseR = scale8(baseR, 0.55f);
    baseG = scale8(baseG, 0.70f);
    baseB = scale8(baseB, 0.92f);
    baseW = scale8(baseW, 0.60f);
    baseMaster = scale8(baseMaster, 0.80f);
  }

  red = baseR;
  green = baseG;
  blue = baseB;
  white = baseW;
  masterDimmer = baseMaster;

  updateDMXBuffer();
}

void markSettingsDirty() {
  settingsDirty = true;
  lastSettingsChangeMs = millis();
}

void loadSettings() {
  preferences.begin("paludarium", false);

  const bool hasSavedValues = preferences.isKey("mode") || preferences.isKey("weather") ||
                              preferences.isKey("manual_r") || preferences.isKey("manual_g") ||
                              preferences.isKey("manual_b") || preferences.isKey("manual_w") ||
                              preferences.isKey("manual_d") || preferences.isKey("fan_speed") ||
                              preferences.isKey("dosing_ms");

  if (hasSavedValues) {
    mode = preferences.getUChar("mode", mode);
    weather = preferences.getUChar("weather", weather);
    red = preferences.getUChar("manual_r", red);
    green = preferences.getUChar("manual_g", green);
    blue = preferences.getUChar("manual_b", blue);
    white = preferences.getUChar("manual_w", white);
    masterDimmer = preferences.getUChar("manual_d", masterDimmer);
    fanSpeed = preferences.getUChar("fan_speed", fanSpeed);
    dosingDurationMs = preferences.getULong("dosing_ms", dosingDurationMs);
  }

  preferences.end();
}

void saveSettings() {
  if (!settingsDirty) {
    return;
  }

  if (millis() - lastSettingsChangeMs < SETTINGS_SAVE_DELAY_MS) {
    return;
  }

  preferences.begin("paludarium", false);
  preferences.putUChar("mode", static_cast<uint8_t>(mode));
  preferences.putUChar("weather", static_cast<uint8_t>(weather));
  preferences.putUChar("manual_r", red);
  preferences.putUChar("manual_g", green);
  preferences.putUChar("manual_b", blue);
  preferences.putUChar("manual_w", white);
  preferences.putUChar("manual_d", masterDimmer);
  preferences.putUChar("fan_speed", fanSpeed);
  preferences.putULong("dosing_ms", dosingDurationMs);
  preferences.end();

  settingsDirty = false;
}

void updateFanOutput() {
  const uint16_t duty = map(fanSpeed, 0, 100, 0, 255);
  ledcWrite(FAN_PIN, duty);
}

void setFanSpeed(uint8_t speed) {
  fanSpeed = constrain(speed, 0, 100);
  updateFanOutput();
  markSettingsDirty();
}

void setupFan() {
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);

  ledcAttach(FAN_PIN, FAN_PWM_FREQUENCY, FAN_PWM_RESOLUTION);
  ledcWrite(FAN_PIN, 0);

  preferences.begin("paludarium", false);
  const bool hasSavedFanSpeed = preferences.isKey("fan_speed");
  if (hasSavedFanSpeed) {
    fanSpeed = preferences.getUChar("fan_speed", FAN_DEFAULT_SPEED);
  } else {
    fanSpeed = FAN_DEFAULT_SPEED;
  }
  preferences.end();

  updateFanOutput();
}

void setupDosing() {
  pinMode(DOSING_PIN, OUTPUT);
  digitalWrite(DOSING_PIN, LOW);
  dosingActive = false;
  dosingStartedMs = 0;
  dosingStopMs = 0;
  // 再起動時は設定時間だけ復元し、ポンプは必ず OFF で開始する。
}

bool startDosing(uint32_t ms) {
  if (ms < DOSING_STEP_MS || ms > DOSING_MAX_RUN_MS) {
    return false;
  }

  if ((ms % DOSING_STEP_MS) != 0) {
    return false;
  }

  dosingDurationMs = ms;
  dosingStartedMs = millis();
  dosingStopMs = dosingStartedMs + ms;
  dosingActive = true;
  digitalWrite(DOSING_PIN, HIGH);
  return true;
}

void stopDosing() {
  dosingActive = false;
  dosingStartedMs = 0;
  dosingStopMs = 0;
  digitalWrite(DOSING_PIN, LOW);
}

void updateDosing() {
  // Wi‑Fi切断時でも DMX 送信を継続し、ドージングは指定時間の自動停止のみ行う。
  if (!dosingActive) {
    return;
  }

  if ((millis() - dosingStartedMs) >= dosingDurationMs) {
    stopDosing();
  }
}

uint32_t getDosingRemainingMs() {
  if (!dosingActive) {
    return 0;
  }

  uint32_t elapsed = millis() - dosingStartedMs;
  if (elapsed >= dosingDurationMs) {
    return 0;
  }

  return dosingDurationMs - elapsed;
}

// -----------------------------------------------------------------------------
// Web API
// -----------------------------------------------------------------------------

void loadSavedNetworks() {
  savedNetworkCount = 0;
  preferences.begin("paludarium", false);
  const uint8_t count = preferences.getUChar("wifi_count", 0);
  for (uint8_t i = 0; i < MAX_SAVED_NETWORKS && i < count; ++i) {
    const String keySsid = "wifi_ssid_" + String(i);
    const String keyPass = "wifi_pass_" + String(i);
    if (preferences.isKey(keySsid.c_str()) && preferences.isKey(keyPass.c_str())) {
      savedNetworks[savedNetworkCount].ssid = preferences.getString(keySsid.c_str(), "");
      savedNetworks[savedNetworkCount].password = preferences.getString(keyPass.c_str(), "");
      if (savedNetworks[savedNetworkCount].ssid.length() > 0) {
        savedNetworkCount++;
      }
    }
  }
  preferences.end();
}

void saveNetwork(const String& ssid, const String& password) {
  if (ssid.length() == 0) return;

  preferences.begin("paludarium", false);
  const uint8_t existingCount = preferences.getUChar("wifi_count", 0);
  for (uint8_t i = 0; i < existingCount; ++i) {
    const String keySsid = "wifi_ssid_" + String(i);
    if (preferences.getString(keySsid.c_str(), "") == ssid) {
      preferences.putString(("wifi_pass_" + String(i)).c_str(), password);
      preferences.end();
      loadSavedNetworks();
      return;
    }
  }

  if (existingCount < MAX_SAVED_NETWORKS) {
    const uint8_t index = existingCount;
    preferences.putString(("wifi_ssid_" + String(index)).c_str(), ssid);
    preferences.putString(("wifi_pass_" + String(index)).c_str(), password);
    preferences.putUChar("wifi_count", existingCount + 1);
  } else {
    // 5件上限。最も古いものを置き換える。
    for (uint8_t i = 1; i < MAX_SAVED_NETWORKS; ++i) {
      const String keySsid = "wifi_ssid_" + String(i);
      const String prevKeySsid = "wifi_ssid_" + String(i - 1);
      const String keyPass = "wifi_pass_" + String(i);
      const String prevKeyPass = "wifi_pass_" + String(i - 1);
      preferences.putString(prevKeySsid.c_str(), preferences.getString(keySsid.c_str(), ""));
      preferences.putString(prevKeyPass.c_str(), preferences.getString(keyPass.c_str(), ""));
    }
    preferences.putString(("wifi_ssid_" + String(MAX_SAVED_NETWORKS - 1)).c_str(), ssid);
    preferences.putString(("wifi_pass_" + String(MAX_SAVED_NETWORKS - 1)).c_str(), password);
    preferences.putUChar("wifi_count", MAX_SAVED_NETWORKS);
  }
  preferences.end();
  loadSavedNetworks();
}

void deleteNetworkByIndex(uint8_t index) {
  if (index >= savedNetworkCount) return;

  preferences.begin("paludarium", false);
  const uint8_t count = preferences.getUChar("wifi_count", 0);
  for (uint8_t i = index; i < count - 1; ++i) {
    preferences.putString(("wifi_ssid_" + String(i)).c_str(), preferences.getString(("wifi_ssid_" + String(i + 1)).c_str(), ""));
    preferences.putString(("wifi_pass_" + String(i)).c_str(), preferences.getString(("wifi_pass_" + String(i + 1)).c_str(), ""));
  }
  preferences.putString(("wifi_ssid_" + String(count - 1)).c_str(), "");
  preferences.putString(("wifi_pass_" + String(count - 1)).c_str(), "");
  preferences.putUChar("wifi_count", count > 0 ? count - 1 : 0);
  preferences.end();
  loadSavedNetworks();
}

String buildScanJson() {
  const int found = WiFi.scanComplete();
  if (found <= 0) {
    return "[]";
  }

  struct UniqueWifiEntry {
    String ssid;
    int32_t rssi;
    bool secure;
  };

  UniqueWifiEntry unique[32];
  uint8_t uniqueCount = 0;

  for (int i = 0; i < found; ++i) {
    const String ssid = WiFi.SSID(i);
    if (ssid.length() == 0) continue;

    int matchIndex = -1;
    for (uint8_t j = 0; j < uniqueCount; ++j) {
      if (unique[j].ssid == ssid) {
        matchIndex = j;
        break;
      }
    }

    const int32_t rssi = WiFi.RSSI(i);
    const bool secure = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;

    if (matchIndex >= 0) {
      if (rssi > unique[matchIndex].rssi) {
        unique[matchIndex].rssi = rssi;
        unique[matchIndex].secure = secure;
      }
      continue;
    }

    if (uniqueCount < 32) {
      unique[uniqueCount].ssid = ssid;
      unique[uniqueCount].rssi = rssi;
      unique[uniqueCount].secure = secure;
      uniqueCount++;
    }
  }

  String json = "[";
  for (uint8_t i = 0; i < uniqueCount; ++i) {
    if (i > 0) json += ",";
    json += "{\"ssid\":\"" + jsonEscape(unique[i].ssid) + "\",\"rssi\":" + String(unique[i].rssi) + ",\"secure\":" + (unique[i].secure ? "true" : "false") + "}";
  }
  json += "]";
  return json;
}

bool trySavedNetworks() {
  if (savedNetworkCount == 0) {
    return false;
  }

  WiFi.mode(WIFI_AP_STA);
  WiFi.disconnect(false);

  if (savedNetworkAttemptIndex >= savedNetworkCount) {
    savedNetworkAttemptIndex = 0;
  }

  const uint8_t currentIndex = savedNetworkAttemptIndex;
  wifiConnectSsid = savedNetworks[currentIndex].ssid;
  wifiConnectPass = savedNetworks[currentIndex].password;
  WiFi.begin(wifiConnectSsid.c_str(), wifiConnectPass.c_str());
  wifiState = WIFI_STATE_CONNECTING;
  wifiStateChangedMs = millis();
  return true;
}

void triggerSetupMode() {
  if (apMode && WiFi.getMode() == WIFI_AP_STA) {
    return;
  }

  apMode = false;

  Serial.println("[A1] WiFi.mode AP_STA start");
  Serial.flush();
  WiFi.mode(WIFI_AP_STA);
  Serial.println("[A1] WiFi.mode AP_STA OK");
  Serial.flush();

  if (!startSetupAP()) {
    Serial.println("ERROR: Setup AP start failed.");
  }
  wifiState = WIFI_STATE_FAILED;
  wifiStateChangedMs = millis();
}

bool startSetupAP() {
  Serial.println("[A2] softAPConfig start");
  Serial.flush();
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  Serial.println("[A2] softAPConfig OK");
  Serial.flush();

  Serial.println("[A3] softAP start");
  Serial.flush();
  const bool started = WiFi.softAP(setupApSsid.c_str(), SETUP_AP_PASSWORD, WIFI_SETUP_AP_CHANNEL);
  Serial.println("[A3] softAP OK");
  Serial.flush();

  if (!started) {
    apMode = false;
    Serial.print("ERROR: WiFi.softAP() failed for ");
    Serial.println(setupApSsid);
    return false;
  }

  apMode = true;
  currentIp = WiFi.softAPIP().toString();

  Serial.println("[A4] dnsServer start");
  Serial.flush();
  dnsServer.start(53, "*", WiFi.softAPIP());
  Serial.println("[A4] dnsServer OK");
  Serial.flush();

  Serial.println();
  Serial.println("Setup AP started");
  Serial.print("SSID: ");
  Serial.println(setupApSsid);
  Serial.print("AP IP: ");
  Serial.println(currentIp);
  return true;
}

void stopSetupAP() {
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  apMode = false;
  WiFi.hostname(hostname.c_str());
}

void startMDNS() {
  if (!WiFi.isConnected()) return;
  if (!MDNS.begin(hostname.c_str())) {
    Serial.println("mDNS start failed");
    return;
  }
  MDNS.addService("http", "tcp", 80);
  Serial.print("mDNS ready: http://");
  Serial.print(hostname);
  Serial.println(".local");
}

void handleStatus() {
  String json;
  json.reserve(256);

  wifiConnected = WiFi.status() == WL_CONNECTED;
  if (wifiConnected) {
    currentWifiSsid = WiFi.SSID();
    currentWifiRssi = WiFi.RSSI();
    currentIp = WiFi.localIP().toString();
  } else if (apMode) {
    currentIp = WiFi.softAPIP().toString();
    currentWifiSsid = setupApSsid;
    currentWifiRssi = 0;
  } else {
    currentIp = "";
    currentWifiSsid = "";
    currentWifiRssi = 0;
  }

  json = "{";
  json += "\"r\":" + String(red) + ",";
  json += "\"g\":" + String(green) + ",";
  json += "\"b\":" + String(blue) + ",";
  json += "\"w\":" + String(white) + ",";
  json += "\"d\":" + String(masterDimmer) + ",";
  json += "\"fan\":" + String(fanSpeed) + ",";
  json += "\"mode\":" + String(mode) + ",";
  json += "\"weather\":" + String(weather) + ",";
  json += "\"dosing\":" + String(dosingActive ? "true" : "false") + ",";
  json += "\"dosingDurationMs\":" + String(dosingDurationMs) + ",";
  json += "\"dosingRemainingMs\":" + String(getDosingRemainingMs()) + ",";
  json += "\"time\":\"" + timeStr + "\",";
  json += "\"date\":\"" + dateStr + "\",";
  json += "\"phase\":\"" + phase + "\",";
  json += "\"wifiConnected\":" + String(wifiConnected ? "true" : "false") + ",";
  json += "\"ssid\":\"" + currentWifiSsid + "\",";
  json += "\"wifi\":\"" + (wifiConnected ? WiFi.localIP().toString() : (apMode ? WiFi.softAPIP().toString() : "未接続")) + "\",";
  json += "\"rssi\":" + String(currentWifiRssi) + ",";
  json += "\"ip\":\"" + currentIp + "\",";
  json += "\"apMode\":" + String(apMode ? "true" : "false") + ",";
  json += "\"deviceId\":\"" + deviceId + "\",";
  json += "\"setupApSsid\":\"" + setupApSsid + "\",";
  json += "\"hostname\":\"" + hostname + ".local\"";
  json += "}";
  server.send(200, "application/json; charset=utf-8", json);
}

void handleWifiScan() {
  const int scanComplete = WiFi.scanComplete();
  if (scanComplete < 0) {
    WiFi.scanNetworks(true, true);
    wifiScanPending = true;
    wifiScanJsonCache = "[]";
    wifiState = WIFI_STATE_SCANNING;
    wifiStateChangedMs = millis();
    Serial.println("[SCAN] start");
    Serial.flush();
    server.send(200, "application/json; charset=utf-8", "{\"ok\":true,\"status\":\"scan_started\"}");
    return;
  }

  wifiScanJsonCache = buildScanJson();
  wifiScanPending = false;
  wifiState = (WiFi.status() == WL_CONNECTED) ? WIFI_STATE_CONNECTED : WIFI_STATE_FAILED;
  wifiStateChangedMs = millis();
  server.send(200, "application/json; charset=utf-8", wifiScanJsonCache);
}

void handleWifiResults() {
  const int scanComplete = WiFi.scanComplete();
  if (scanComplete < 0) {
    if (wifiState == WIFI_STATE_SCANNING && (millis() - wifiStateChangedMs) >= WIFI_SCAN_TIMEOUT_MS) {
      WiFi.scanDelete();
      wifiScanPending = false;
      wifiState = (WiFi.status() == WL_CONNECTED) ? WIFI_STATE_CONNECTED : WIFI_STATE_FAILED;
      wifiStateChangedMs = millis();
      Serial.println("[SCAN] timeout, continuing AP mode");
      Serial.flush();
    }
    server.send(200, "application/json; charset=utf-8", "{\"ok\":true,\"status\":\"scan_pending\"}");
    return;
  }

  wifiScanJsonCache = buildScanJson();
  wifiScanPending = false;
  wifiState = (WiFi.status() == WL_CONNECTED) ? WIFI_STATE_CONNECTED : WIFI_STATE_FAILED;
  wifiStateChangedMs = millis();
  server.send(200, "application/json; charset=utf-8", wifiScanJsonCache);
}

String jsonEscape(const String& input) {
  String result;
  result.reserve(input.length() + 8);

  for (size_t i = 0; i < input.length(); ++i) {
    const char ch = input.charAt(i);
    switch (ch) {
      case '\\': result += "\\\\"; break;
      case '"': result += "\\\""; break;
      case '\n': result += "\\n"; break;
      case '\r': result += "\\r"; break;
      case '\t': result += "\\t"; break;
      default: result += ch; break;
    }
  }

  return result;
}

bool hasConfiguredSecretsWifi() {
  return (strlen(WIFI_SSID) > 0) && (strcmp(WIFI_SSID, "YOUR_WIFI_SSID") != 0);
}

void attemptWifiConnection(const String& ssid, const String& password) {
  if (ssid.length() == 0) {
    return;
  }

  WiFi.mode(WIFI_AP_STA);
  WiFi.hostname(hostname.c_str());
  WiFi.disconnect(false);
  WiFi.begin(ssid.c_str(), password.c_str());
  wifiConnectSsid = ssid;
  wifiConnectPass = password;
  wifiState = WIFI_STATE_CONNECTING;
  wifiStateChangedMs = millis();
  Serial.print("Connecting to Wi‑Fi: ");
  Serial.println(ssid);
}

void handleWifiSave() {
  String ssid = server.arg("ssid");
  String password = server.arg("password");
  ssid.trim();

  if (ssid.length() == 0) {
    server.send(400, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"empty_ssid\"}");
    return;
  }

  saveNetwork(ssid, password);
  pendingWifiReconnectSsid = ssid;
  pendingWifiReconnectPassword = password;
  pendingWifiReconnectMs = millis() + 2500;
  wifiState = WIFI_STATE_CONNECTING;
  wifiStateChangedMs = millis();

  String json = "{\"ok\":true,\"message\":\"保存しました。接続を試しています。\",\"ssid\":\"" + jsonEscape(ssid) + "\"}";
  server.send(200, "application/json; charset=utf-8", json);
  delay(10);
}

void handleWifiSaved() {
  String json = "[";
  for (uint8_t i = 0; i < savedNetworkCount; ++i) {
    if (i > 0) json += ",";
    json += "{\"id\":" + String(i) + ",\"ssid\":\"" + jsonEscape(savedNetworks[i].ssid) + "\"}";
  }
  json += "]";
  server.send(200, "application/json; charset=utf-8", json);
}

void handleWifiDelete() {
  uint8_t index = server.hasArg("id") ? static_cast<uint8_t>(server.arg("id").toInt()) : 0;
  if (index >= savedNetworkCount) {
    server.send(400, "application/json; charset=utf-8", "{\"ok\":false,\"error\":\"invalid_id\"}");
    return;
  }

  const String targetSsid = savedNetworks[index].ssid;
  deleteNetworkByIndex(index);
  if (WiFi.SSID() == targetSsid) {
    WiFi.disconnect(false);
  }

  server.send(200, "application/json; charset=utf-8", "{\"ok\":true}");
}

void handleFan() {
  if (server.hasArg("speed")) {
    const int requested = server.arg("speed").toInt();
    setFanSpeed(constrain(requested, 0, 100));
  }

  server.send(200, "text/plain; charset=utf-8", "OK");
}

void handleDosingStart() {
  uint32_t requestedMs = DOSING_DEFAULT_MS;
  if (server.hasArg("ms")) {
    requestedMs = static_cast<uint32_t>(server.arg("ms").toInt());
  }

  if (!startDosing(requestedMs)) {
    server.send(400, "text/plain; charset=utf-8", "Invalid dosing time");
    return;
  }

  dosingDurationMs = requestedMs;
  markSettingsDirty();
  server.send(200, "text/plain; charset=utf-8", "OK");
}

void handleDosingStop() {
  stopDosing();
  server.send(200, "text/plain; charset=utf-8", "OK");
}

void handleDosingConfig() {
  uint32_t requestedMs = DOSING_DEFAULT_MS;
  if (server.hasArg("ms")) {
    requestedMs = static_cast<uint32_t>(server.arg("ms").toInt());
  }

  if (requestedMs < DOSING_STEP_MS || requestedMs > DOSING_MAX_RUN_MS || (requestedMs % DOSING_STEP_MS) != 0) {
    server.send(400, "text/plain; charset=utf-8", "Invalid dosing duration");
    return;
  }

  dosingDurationMs = requestedMs;
  markSettingsDirty();
  server.send(200, "text/plain; charset=utf-8", "OK");
}

void handleMode() {
  if (server.hasArg("m")) {
    mode = (server.arg("m").toInt() == 0) ? 0 : 1;
  }

  if (mode == 0) {
    updateAutomaticLight();
  } else {
    phase = "手動";
  }

  markSettingsDirty();
  updateDMXBuffer();
  server.send(200, "text/plain; charset=utf-8", "OK");
}

void handleWeather() {
  if (server.hasArg("w")) {
    weather = constrain(server.arg("w").toInt(), 0, 2);
  }

  if (mode == 0) updateAutomaticLight();
  markSettingsDirty();
  server.send(200, "text/plain; charset=utf-8", "OK");
}

void handleLevels() {
  if (server.hasArg("r")) red = constrain(server.arg("r").toInt(), 0, 255);
  if (server.hasArg("g")) green = constrain(server.arg("g").toInt(), 0, 255);
  if (server.hasArg("b")) blue = constrain(server.arg("b").toInt(), 0, 255);
  if (server.hasArg("w")) white = constrain(server.arg("w").toInt(), 0, 255);
  if (server.hasArg("d")) masterDimmer = constrain(server.arg("d").toInt(), 0, 255);

  mode = 1;
  phase = "手動";
  markSettingsDirty();
  updateDMXBuffer();

  server.send(200, "text/plain; charset=utf-8", "OK");
}

// -----------------------------------------------------------------------------
// Web UI
// -----------------------------------------------------------------------------

const char WIFI_SETUP_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ja">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <title>Paludarium Wi‑Fi Setup</title>
  <style>
    :root {
      --bg: #06121d;
      --panel: #0e1e2d;
      --card: #12273a;
      --line: rgba(165, 191, 228, 0.24);
      --text: #edf5ff;
      --muted: #9bb5d8;
      --primary: #67b6ff;
      --accent: #7f8dff;
      --success: #4de0a7;
      --danger: #ff7171;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0; background: radial-gradient(circle at top, #0d2238, var(--bg));
      color: var(--text); font-family: Arial, sans-serif; display: flex; justify-content: center; padding: 24px 14px;
    }
    .app { width: min(100%, 760px); }
    .top { display: flex; justify-content: space-between; align-items: center; gap: 12px; margin-bottom: 18px; }
    h1 { margin: 0; font-size: clamp(1.6rem, 4vw, 2.5rem); }
    .status-pill { padding: 8px 12px; border-radius: 999px; border: 1px solid var(--line); background: rgba(255,255,255,0.04); font-weight: 700; }
    .panel { background: rgba(15, 28, 42, 0.96); border: 1px solid var(--line); border-radius: 16px; padding: 16px; margin-bottom: 18px; box-shadow: 0 10px 30px rgba(0,0,0,0.18); }
    .grid { display: grid; gap: 14px; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); }
    .field { display: flex; flex-direction: column; gap: 8px; }
    label { font-weight: 700; color: var(--muted); }
    input, select, button { font: inherit; }
    input[type="text"], input[type="password"] { width: 100%; border-radius: 10px; border: 1px solid var(--line); background: rgba(8,18,26,0.9); color: var(--text); padding: 11px 12px; }
    .row { display: flex; gap: 10px; flex-wrap: wrap; align-items: center; }
    button {
      background: linear-gradient(135deg, var(--primary), var(--accent));
      color: white; border: 0; border-radius: 10px; padding: 10px 14px; font-weight: 800; cursor: pointer;
    }
    button.secondary { background: rgba(255,255,255,0.06); border: 1px solid var(--line); }
    button.danger { background: linear-gradient(135deg, #ff7272, #d95e93); }
    .muted { color: var(--muted); }
    table { width: 100%; border-collapse: collapse; }
    th, td { text-align: left; padding: 8px 6px; border-bottom: 1px solid var(--line); vertical-align: middle; }
    .strength { display: inline-block; min-width: 68px; }
    .strength[data-level="0"] { color: var(--muted); }
    .strength[data-level="1"] { color: #ffbf69; }
    .strength[data-level="2"] { color: #9ae8ff; }
    .strength[data-level="3"] { color: var(--success); }
    .hidden { display: none !important; }
    .scan-status { color: var(--muted); font-size: 0.95rem; }
    .list-btn { width: 100%; text-align: left; padding: 10px 12px; background: rgba(255,255,255,0.04); border: 1px solid var(--line); border-radius: 10px; color: var(--text); }
    .list-btn strong { display: block; }
    .footer { display: flex; justify-content: space-between; gap: 10px; margin-top: 12px; flex-wrap: wrap; }
  </style>
</head>
<body>
  <div class="app">
    <div class="top">
      <h1>Paludarium Wi‑Fi Setup</h1>
      <div class="status-pill" id="ap-status">AP モード</div>
    </div>

    <div class="panel">
      <div class="grid">
        <div class="field"><label>端末ID</label><div id="device-id">-</div></div>
        <div class="field"><label>ローカルURL</label><div id="local-hostname">-</div></div>
        <div class="field"><label>設定用Wi-Fi名</label><div id="setup-ap-ssid">-</div></div>
        <div class="field"><label>現在の接続状態</label><div id="conn-state">未接続</div></div>
        <div class="field"><label>現在接続しているSSID</label><div id="current-ssid">-</div></div>
        <div class="field"><label>現在のIPアドレス</label><div id="current-ip">-</div></div>
      </div>
      <div class="row" style="margin-top: 14px;">
        <button type="button" id="scan-btn">周辺Wi‑Fiを検索</button>
        <button type="button" class="secondary" id="refresh-saved-btn">保存済みWi‑Fiを更新</button>
      </div>
    </div>

    <div class="panel">
      <div class="field" style="margin-bottom: 12px;">
        <label>検出されたSSID一覧</label>
        <div id="scan-status" class="scan-status">検索待ち</div>
      </div>
      <div id="scan-results" class="grid"></div>
    </div>

    <div class="panel">
      <form id="wifi-form">
        <div class="field" style="margin-bottom: 12px;">
          <label for="ssid-input">SSID</label>
          <input id="ssid-input" name="ssid" type="text" placeholder="Wi‑FiのSSID" />
        </div>
        <div class="field" style="margin-bottom: 12px;">
          <label for="password-input">パスワード</label>
          <div class="row" style="align-items: stretch;">
            <input id="password-input" name="password" type="password" placeholder="Wi‑Fiパスワード" />
            <button type="button" class="secondary" id="toggle-password-btn">表示</button>
          </div>
        </div>
        <div class="row">
          <button type="submit">接続して保存</button>
        </div>
      </form>
    </div>

    <div class="panel">
      <div class="field" style="margin-bottom: 8px;">
        <label>保存済みWi‑Fi一覧</label>
      </div>
      <div id="saved-networks"></div>
    </div>

    <div class="footer">
      <button type="button" class="secondary" id="back-control-btn">制御画面へ戻る</button>
      <button type="button" class="danger" id="clear-network-btn">保存済みWi‑Fiを消去</button>
    </div>
  </div>

  <script>
    function setStatus(el, value) {
      document.getElementById(el).textContent = value || '-';
    }

    function updateStatusPanel() {
      fetch('/status').then((r) => r.json()).then((data) => {
        const connected = !!data.wifiConnected;
        setStatus('device-id', data.deviceId || '-');
        setStatus('local-hostname', data.hostname ? `http://${data.hostname}/` : '-');
        setStatus('setup-ap-ssid', data.setupApSsid || '-');
        setStatus('conn-state', connected ? '接続済み' : '未接続');
        setStatus('current-ssid', data.ssid || '-');
        setStatus('current-ip', data.ip || '-');
        document.getElementById('ap-status').textContent = data.apMode ? 'AP モード' : 'STA モード';
      }).catch(() => {
        setStatus('conn-state', '状態取得失敗');
      });
    }

    function renderScanResults(items) {
      const container = document.getElementById('scan-results');
      if (!items || !items.length) {
        container.innerHTML = '<div class="muted">検索結果がありません。</div>';
        return;
      }

      const rows = items.map((item) => {
        const level = item.rssi > -60 ? 3 : item.rssi > -75 ? 2 : item.rssi > -90 ? 1 : 0;
        return `
          <button class="list-btn" data-ssid="${item.ssid.replace(/"/g, '&quot;')}" type="button">
            <strong>${item.ssid}</strong>
            <span class="muted">RSSI: ${item.rssi} dBm / ${item.secure ? '暗号化あり' : '暗号化なし'}</span>
            <span class="strength" data-level="${level}">${item.rssi > -60 ? '強' : item.rssi > -75 ? '中' : item.rssi > -90 ? '弱' : '極弱'}</span>
          </button>
        `;
      }).join('');

      container.innerHTML = rows;
      container.querySelectorAll('[data-ssid]').forEach((btn) => {
        btn.addEventListener('click', () => {
          const ssid = btn.getAttribute('data-ssid');
          document.getElementById('ssid-input').value = ssid;
        });
      });
    }

    function loadScanResults() {
      document.getElementById('scan-status').textContent = '検索中...';
      fetch('/wifi/scan').then((r) => r.json()).then((data) => {
        const items = Array.isArray(data) ? data : [];
        renderScanResults(items);
        document.getElementById('scan-status').textContent = items.length ? `${items.length}件のSSIDを検出しました` : '検出結果がありません';
      }).catch(() => {
        document.getElementById('scan-status').textContent = '検索に失敗しました';
      });
    }

    function loadSavedNetworks() {
      fetch('/wifi/saved').then((r) => r.json()).then((data) => {
        const container = document.getElementById('saved-networks');
        if (!Array.isArray(data) || data.length === 0) {
          container.innerHTML = '<div class="muted">保存済みWi‑Fiはありません。</div>';
          return;
        }

        container.innerHTML = data.map((item) => `
          <div class="row" style="justify-content: space-between; margin-bottom: 8px; padding: 10px 12px; border: 1px solid var(--line); border-radius: 10px; background: rgba(255,255,255,0.03);">
            <span>${item.ssid}</span>
            <button type="button" class="danger" data-delete-id="${item.id}">削除</button>
          </div>
        `).join('');

        container.querySelectorAll('[data-delete-id]').forEach((btn) => {
          btn.addEventListener('click', () => {
            const id = btn.getAttribute('data-delete-id');
            fetch('/wifi/delete?id=' + id).then(() => loadSavedNetworks()).then(() => updateStatusPanel());
          });
        });
      }).catch(() => {
        document.getElementById('saved-networks').innerHTML = '<div class="muted">保存済み一覧の取得に失敗しました。</div>';
      });
    }

    document.getElementById('scan-btn').addEventListener('click', loadScanResults);
    document.getElementById('refresh-saved-btn').addEventListener('click', loadSavedNetworks);
    document.getElementById('toggle-password-btn').addEventListener('click', () => {
      const input = document.getElementById('password-input');
      const visible = input.type === 'text';
      input.type = visible ? 'password' : 'text';
      document.getElementById('toggle-password-btn').textContent = visible ? '表示' : '非表示';
    });
    document.getElementById('wifi-settings-btn')?.addEventListener('click', () => {
      window.location.href = '/wifi';
    });
    document.getElementById('back-control-btn').addEventListener('click', () => {
      window.location.href = '/';
    });
    document.getElementById('clear-network-btn').addEventListener('click', () => {
      fetch('/wifi/saved').then((r) => r.json()).then((data) => {
        if (!Array.isArray(data) || data.length === 0) return;
        Promise.all(data.map((item) => fetch('/wifi/delete?id=' + item.id))).then(() => loadSavedNetworks());
      });
    });

    document.getElementById('wifi-form').addEventListener('submit', (event) => {
      event.preventDefault();
      const ssid = document.getElementById('ssid-input').value.trim();
      const password = document.getElementById('password-input').value;
      if (!ssid) {
        alert('SSIDを入力してください');
        return;
      }

      const params = new URLSearchParams({ ssid, password });
      fetch('/wifi/save?' + params.toString()).then((r) => r.json()).then((data) => {
        alert(data.message || '保存しました。接続を試しています。');
        loadSavedNetworks();
        updateStatusPanel();
      }).catch(() => alert('保存に失敗しました'));
    });

    updateStatusPanel();
    loadSavedNetworks();
    loadScanResults();
    setInterval(updateStatusPanel, 5000);
  </script>
</body>
</html>
)rawliteral";

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ja">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <title>Paludarium Light</title>
  <style>
    :root {
      --bg-1: #040b18;
      --bg-2: #0a1426;
      --panel: rgba(17, 27, 40, 0.96);
      --panel-soft: rgba(22, 33, 47, 0.94);
      --card: rgba(16, 24, 35, 0.96);
      --border: rgba(140, 164, 193, 0.18);
      --text: #edf3ff;
      --muted: #8ea7c7;
      --purple-1: #6a5cff;
      --purple-2: #9c59ff;
      --purple-3: #b96cff;
      --green: #4de0a7;
      --cyan: #6dc5ff;
      --red: #ff6d5f;
      --red-soft: rgba(255, 109, 95, 0.16);
      --green-soft: rgba(77, 224, 167, 0.14);
      --blue-soft: rgba(86, 160, 255, 0.15);
      --white-soft: rgba(220, 232, 255, 0.15);
      --purple-soft: rgba(174, 106, 255, 0.18);
      --shadow: 0 12px 24px rgba(0,0,0,0.25);
      --track: rgba(160, 179, 206, 0.2);
      --thumb: #f4f8ff;
    }

    * { box-sizing: border-box; }

    html, body {
      margin: 0;
      min-height: 100%;
      background: radial-gradient(circle at top, #0d1d33 0%, var(--bg-1) 38%, #020915 100%);
      color: var(--text);
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }

    body {
      display: flex;
      justify-content: center;
      padding: max(12px, env(safe-area-inset-top)) 12px max(24px, env(safe-area-inset-bottom));
    }

    .app {
      width: min(100%, 440px);
      padding-bottom: 12px;
    }

    .topbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin: 4px 0 14px;
    }

    .brand {
      display: flex;
      align-items: center;
      gap: 12px;
      font-weight: 800;
      letter-spacing: 0.02em;
      font-size: clamp(1.6rem, 3.3vw, 2.3rem);
    }

    .brand-mark {
      width: 36px;
      height: 36px;
      border-radius: 10px;
      background: linear-gradient(135deg, #5d6dff, #9a61ff);
      box-shadow: inset 0 0 0 1px rgba(255,255,255,0.24), var(--shadow);
      position: relative;
      flex-shrink: 0;
    }

    .brand-mark::before,
    .brand-mark::after {
      content: "";
      position: absolute;
      border-radius: 50%;
      background: rgba(255,255,255,0.9);
    }

    .brand-mark::before {
      width: 12px;
      height: 12px;
      left: 12px;
      top: 7px;
      box-shadow: 0 0 0 3px rgba(255,255,255,0.24);
    }

    .brand-mark::after {
      width: 18px;
      height: 18px;
      left: 9px;
      top: 19px;
      clip-path: polygon(50% 0, 100% 100%, 0 100%);
      background: rgba(255,255,255,0.72);
    }

    .status-pill {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      padding: 8px 12px;
      border: 1px solid var(--border);
      border-radius: 999px;
      background: rgba(19, 28, 38, 0.9);
      color: var(--text);
      font-weight: 700;
      font-size: 0.82rem;
    }

    .status-dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: #aef3bf;
      box-shadow: 0 0 12px rgba(110, 255, 183, 0.8);
    }

    .status-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 10px;
      margin-bottom: 18px;
    }

    .card {
      background: linear-gradient(180deg, rgba(19, 28, 39, 0.98), rgba(12, 20, 30, 0.98));
      border: 1px solid var(--border);
      border-radius: 16px;
      box-shadow: var(--shadow);
    }

    .status-card {
      padding: 12px 12px 10px;
      min-height: 120px;
      display: flex;
      flex-direction: column;
      justify-content: space-between;
    }

    .status-top {
      display: flex;
      align-items: center;
      gap: 8px;
      color: var(--muted);
      font-size: 0.88rem;
      font-weight: 600;
    }

    .status-small {
      font-size: 1.02rem;
      color: var(--text);
      font-weight: 700;
      letter-spacing: 0.02em;
    }

    .status-main {
      font-weight: 800;
      font-size: clamp(1.6rem, 5vw, 2.5rem);
      letter-spacing: 0.02em;
      line-height: 1.1;
      margin-top: 4px;
    }

    .status-sub {
      font-size: 0.74rem;
      color: var(--muted);
      margin-top: 2px;
    }

    .status-value {
      font-weight: 700;
      color: var(--green);
      font-size: 1.05rem;
      margin-top: 2px;
    }

    .panel {
      background: linear-gradient(180deg, rgba(18, 28, 39, 0.96), rgba(12, 20, 31, 0.96));
      border: 1px solid var(--border);
      border-radius: 18px;
      padding: 12px 12px 10px;
      box-shadow: var(--shadow);
      margin-bottom: 18px;
    }

    .panel-title {
      display: flex;
      align-items: center;
      gap: 8px;
      margin: 0 0 12px;
      font-size: 1.05rem;
      font-weight: 800;
      color: var(--text);
    }

    .panel-title .icon {
      font-size: 1.15rem;
      opacity: 0.9;
    }

    .mode-grid,
    .weather-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }

    .weather-grid {
      grid-template-columns: repeat(3, minmax(0, 1fr));
    }

    .segmented {
      appearance: none;
      border: 1px solid var(--border);
      border-radius: 14px;
      background: rgba(20, 30, 42, 0.9);
      color: var(--text);
      min-height: 56px;
      font-size: 1rem;
      font-weight: 700;
      padding: 10px 12px;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      width: 100%;
      transition: transform 0.12s ease, box-shadow 0.12s ease, border-color 0.12s ease;
      cursor: pointer;
      -webkit-tap-highlight-color: transparent;
      touch-action: manipulation;
    }

    .segmented:active {
      transform: translateY(1px) scale(0.995);
    }

    .segmented.active {
      background: linear-gradient(135deg, rgba(92, 83, 255, 0.9), rgba(163, 90, 255, 0.9));
      border-color: rgba(167, 137, 255, 0.8);
      box-shadow: 0 6px 18px rgba(117, 89, 255, 0.35);
    }

    .segmented.weather.active {
      background: linear-gradient(135deg, rgba(46, 74, 116, 0.9), rgba(60, 91, 149, 0.9));
      border-color: rgba(120, 166, 255, 0.8);
      box-shadow: 0 6px 18px rgba(80, 116, 191, 0.28);
      color: #ebf6ff;
    }

    .segmented-toggle {
      position: relative;
      display: flex;
      align-items: center;
      justify-content: center;
      min-height: 56px;
      font-weight: 700;
      font-size: 1rem;
    }

    .levels-wrap {
      display: grid;
      gap: 14px;
    }

    .level-row {
      display: grid;
      grid-template-columns: 88px 1fr 96px;
      gap: 10px;
      align-items: center;
      padding: 6px 0;
    }

    .level-row .label {
      display: flex;
      align-items: center;
      gap: 8px;
      font-weight: 700;
      font-size: 0.96rem;
      color: var(--text);
    }

    .swatch {
      width: 10px;
      height: 10px;
      border-radius: 50%;
      display: inline-block;
      box-shadow: 0 0 0 2px rgba(255,255,255,0.2);
    }

    .slider-area {
      position: relative;
      display: flex;
      align-items: center;
      min-height: 52px;
      padding: 8px 0;
    }

    .slider-rail {
      position: relative;
      width: 100%;
      height: 12px;
      border-radius: 999px;
      background: linear-gradient(90deg, rgba(255,255,255,0.08), rgba(255,255,255,0.15));
      box-shadow: inset 0 0 0 1px rgba(255,255,255,0.08);
      touch-action: pan-y;
    }

    .slider-rail::before {
      content: "";
      position: absolute;
      inset: 0;
      border-radius: inherit;
      background: linear-gradient(90deg, var(--rail-color, #fff) 0%, var(--rail-color, #fff) 100%);
      opacity: 0.18;
    }

    input[type="range"] {
      -webkit-appearance: none;
      appearance: none;
      position: absolute;
      inset: 0;
      width: 100%;
      height: 44px;
      background: transparent;
      margin: 0;
      cursor: pointer;
      z-index: 2;
    }

    input[type="range"]::-webkit-slider-runnable-track {
      height: 12px;
      background: transparent;
      border-radius: 999px;
    }

    input[type="range"]::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 30px;
      height: 30px;
      border-radius: 50%;
      background: var(--thumb);
      border: 2px solid rgba(0,0,0,0.18);
      box-shadow: 0 3px 8px rgba(0,0,0,0.35), 0 0 0 2px rgba(255,255,255,0.2);
      margin-top: -9px;
    }

    input[type="range"]::-moz-range-track {
      height: 12px;
      background: transparent;
      border-radius: 999px;
    }

    input[type="range"]::-moz-range-thumb {
      width: 30px;
      height: 30px;
      border: none;
      border-radius: 50%;
      background: var(--thumb);
      box-shadow: 0 3px 8px rgba(0,0,0,0.35), 0 0 0 2px rgba(255,255,255,0.2);
    }

    .slider-fill {
      position: absolute;
      top: 50%;
      left: 0;
      height: 12px;
      border-radius: 999px;
      transform: translateY(-50%);
      background: linear-gradient(90deg, var(--rail-color, #fff), var(--rail-color, #fff));
      box-shadow: 0 0 14px rgba(255,255,255,0.12);
      pointer-events: none;
      z-index: 1;
    }

    .slider-ticks {
      position: absolute;
      inset: 0;
      pointer-events: none;
      z-index: 0;
    }

    .slider-ticks span {
      position: absolute;
      top: 50%;
      width: 2px;
      height: 16px;
      border-radius: 2px;
      background: rgba(255,255,255,0.22);
      transform: translate(-50%, -50%);
    }

    .slider-ticks span:nth-child(1) { left: 0%; }
    .slider-ticks span:nth-child(2) { left: 25%; }
    .slider-ticks span:nth-child(3) { left: 50%; }
    .slider-ticks span:nth-child(4) { left: 75%; }
    .slider-ticks span:nth-child(5) { left: 100%; }

    .range-value {
      position: absolute;
      top: -10px;
      transform: translateX(-50%);
      min-width: 34px;
      text-align: center;
      padding: 4px 7px;
      border-radius: 8px;
      background: rgba(16, 23, 33, 0.96);
      border: 1px solid rgba(255,255,255,0.12);
      font-size: 0.72rem;
      font-weight: 800;
      color: var(--text);
      box-shadow: var(--shadow);
      pointer-events: none;
      z-index: 3;
    }

    .slider-output {
      display: flex;
      justify-content: flex-end;
      align-items: center;
      gap: 8px;
      min-width: 92px;
    }

    .value-primer {
      font-weight: 800;
      font-size: 1.1rem;
      color: var(--text);
      min-width: 2ch;
      text-align: right;
    }

    .stepper {
      display: grid;
      grid-template-columns: 40px 40px;
      gap: 8px;
      width: 88px;
      justify-self: end;
    }

    .step-btn {
      min-width: 40px;
      min-height: 40px;
      border: 1px solid var(--border);
      border-radius: 10px;
      background: rgba(20, 31, 45, 0.9);
      color: var(--text);
      font-size: 1.4rem;
      line-height: 1;
      font-weight: 800;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      -webkit-tap-highlight-color: transparent;
      touch-action: manipulation;
    }

    .step-btn:active {
      transform: translateY(1px);
    }

    .quick-presets {
      display: grid;
      grid-template-columns: repeat(5, minmax(0, 1fr));
      gap: 8px;
      margin-top: 4px;
    }

    .preset-btn {
      min-height: 60px;
      border: 1px solid var(--border);
      background: linear-gradient(180deg, rgba(17, 26, 36, 0.98), rgba(12, 20, 30, 0.98));
      color: var(--text);
      border-radius: 12px;
      padding: 8px 6px;
      font-size: 0.75rem;
      font-weight: 700;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      gap: 4px;
      cursor: pointer;
      -webkit-tap-highlight-color: transparent;
      touch-action: manipulation;
    }

    .preset-btn:active {
      transform: translateY(1px);
    }

    .preset-icon {
      font-size: 1.05rem;
      line-height: 1;
    }

    .footer-nav {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 10px;
      margin-top: 16px;
    }

    .nav-btn {
      min-height: 56px;
      border: 1px solid rgba(123, 145, 198, 0.2);
      background: linear-gradient(180deg, rgba(35, 54, 78, 0.92), rgba(17, 28, 38, 0.96));
      color: var(--text);
      border-radius: 14px;
      font-weight: 700;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      gap: 4px;
      font-size: 0.72rem;
      cursor: pointer;
    }

    .nav-btn.primary {
      background: linear-gradient(135deg, rgba(74, 112, 255, 0.26), rgba(155, 94, 255, 0.32));
      border-color: rgba(166, 128, 255, 0.52);
    }

    .nav-btn .symbol {
      font-size: 1.2rem;
      line-height: 1;
    }

    @media (max-width: 390px) {
      .status-grid {
        grid-template-columns: 1fr;
      }

      .weather-grid {
        grid-template-columns: repeat(3, minmax(0, 1fr));
      }

      .quick-presets {
        grid-template-columns: repeat(5, minmax(0, 1fr));
      }
    }
  </style>
</head>
<body>
  <div class="app">
    <div class="topbar">
      <div class="brand">
        <span class="brand-mark" aria-hidden="true"></span>
        <span id="device-label">Paludarium Light</span>
      </div>
      <div class="status-pill"><span class="status-dot"></span><span id="wifi-pill">192.168.0.31</span></div>
    </div>

    <div class="status-grid">
      <div class="card status-card">
        <div class="status-top">
          <span>◔</span>
          <span>現在時刻</span>
        </div>
        <div>
          <div id="time" class="status-main">00:03</div>
          <div id="date" class="status-sub">----/--/-- (-)</div>
        </div>
      </div>

      <div class="card status-card">
        <div class="status-top">
          <span>✦</span>
          <span>状態</span>
        </div>
        <div>
          <div id="phase" class="status-value">手動</div>
        </div>
      </div>

      <div class="card status-card">
        <div class="status-top">
          <span>◌</span>
          <span>Wi‑Fi</span>
        </div>
        <div>
          <div class="status-small">接続中</div>
          <div id="wifi" class="status-value">192.168.0.31</div>
        </div>
      </div>
    </div>

    <div class="panel">
      <div class="panel-title"><span class="icon">☀</span><span>LIGHT</span></div>

      <div class="panel" style="margin:0 0 12px; padding:12px;">
        <div class="panel-title" style="margin-bottom: 10px;"><span class="icon">✦</span><span>Mode</span></div>
        <div class="mode-grid">
          <button class="segmented" data-mode="0" id="mode-auto">AUTO</button>
          <button class="segmented active" data-mode="1" id="mode-manual">MANUAL</button>
        </div>
      </div>

      <div class="panel" style="margin:0 0 12px; padding:12px;">
        <div class="panel-title" style="margin-bottom: 10px;"><span class="icon">☼</span><span>Weather</span></div>
        <div class="weather-grid">
          <button class="segmented weather active" data-weather="0" id="weather-sunny">Sunny</button>
          <button class="segmented weather" data-weather="1" id="weather-cloudy">Cloudy</button>
          <button class="segmented weather" data-weather="2" id="weather-rain">Rain</button>
        </div>
      </div>

      <div class="panel" style="margin:0; padding:12px;">
        <div class="panel-title" style="margin-bottom: 10px;"><span class="icon">▤</span><span>Levels</span></div>

        <div class="levels-wrap">
          <div class="level-row" data-channel="r">
            <div class="label"><span class="swatch" style="background:#ef5a4d"></span>Red</div>
            <div class="slider-area" data-color="#ef5a4d">
              <div class="slider-rail"><div class="slider-fill" style="--rail-color:#ef5a4d"></div><div class="slider-ticks"><span></span><span></span><span></span><span></span><span></span></div></div>
              <input id="r" type="range" min="0" max="255" step="1" value="255" aria-label="Red">
              <div class="range-value" id="r-bubble">255</div>
            </div>
            <div class="slider-output">
              <div class="value-primer" id="r-value">255</div>
              <div class="stepper">
                <button class="step-btn" data-channel="r" data-step="-1">−</button>
                <button class="step-btn" data-channel="r" data-step="1">＋</button>
              </div>
            </div>
          </div>

          <div class="level-row" data-channel="g">
            <div class="label"><span class="swatch" style="background:#49d46d"></span>Green</div>
            <div class="slider-area" data-color="#49d46d">
              <div class="slider-rail"><div class="slider-fill" style="--rail-color:#49d46d"></div><div class="slider-ticks"><span></span><span></span><span></span><span></span><span></span></div></div>
              <input id="g" type="range" min="0" max="255" step="1" value="0" aria-label="Green">
              <div class="range-value" id="g-bubble">0</div>
            </div>
            <div class="slider-output">
              <div class="value-primer" id="g-value">0</div>
              <div class="stepper">
                <button class="step-btn" data-channel="g" data-step="-1">−</button>
                <button class="step-btn" data-channel="g" data-step="1">＋</button>
              </div>
            </div>
          </div>

          <div class="level-row" data-channel="b">
            <div class="label"><span class="swatch" style="background:#4ea3ff"></span>Blue</div>
            <div class="slider-area" data-color="#4ea3ff">
              <div class="slider-rail"><div class="slider-fill" style="--rail-color:#4ea3ff"></div><div class="slider-ticks"><span></span><span></span><span></span><span></span><span></span></div></div>
              <input id="b" type="range" min="0" max="255" step="1" value="0" aria-label="Blue">
              <div class="range-value" id="b-bubble">0</div>
            </div>
            <div class="slider-output">
              <div class="value-primer" id="b-value">0</div>
              <div class="stepper">
                <button class="step-btn" data-channel="b" data-step="-1">−</button>
                <button class="step-btn" data-channel="b" data-step="1">＋</button>
              </div>
            </div>
          </div>

          <div class="level-row" data-channel="w">
            <div class="label"><span class="swatch" style="background:#e4ecff"></span>White</div>
            <div class="slider-area" data-color="#e4ecff">
              <div class="slider-rail"><div class="slider-fill" style="--rail-color:#e4ecff"></div><div class="slider-ticks"><span></span><span></span><span></span><span></span><span></span></div></div>
              <input id="w" type="range" min="0" max="255" step="1" value="0" aria-label="White">
              <div class="range-value" id="w-bubble">0</div>
            </div>
            <div class="slider-output">
              <div class="value-primer" id="w-value">0</div>
              <div class="stepper">
                <button class="step-btn" data-channel="w" data-step="-1">−</button>
                <button class="step-btn" data-channel="w" data-step="1">＋</button>
              </div>
            </div>
          </div>

          <div class="level-row" data-channel="d">
            <div class="label"><span class="swatch" style="background:#d268ff"></span>Master</div>
            <div class="slider-area" data-color="#d268ff">
              <div class="slider-rail"><div class="slider-fill" style="--rail-color:#d268ff"></div><div class="slider-ticks"><span></span><span></span><span></span><span></span><span></span></div></div>
              <input id="d" type="range" min="0" max="255" step="1" value="255" aria-label="Master Dimmer">
              <div class="range-value" id="d-bubble">255</div>
            </div>
            <div class="slider-output">
              <div class="value-primer" id="d-value">255</div>
              <div class="stepper">
                <button class="step-btn" data-channel="d" data-step="-1">−</button>
                <button class="step-btn" data-channel="d" data-step="1">＋</button>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <div class="panel">
      <div class="panel-title"><span class="icon">❋</span><span>FAN</span></div>
      <div class="levels-wrap">
        <div class="level-row" style="grid-template-columns: 1fr auto;">
          <div class="label">Speed</div>
          <div class="value-primer" id="fan-value">0%</div>
        </div>
        <div class="slider-area" style="padding-top: 0;">
          <div class="slider-rail" style="--rail-color:#6dc5ff;"><div class="slider-fill" id="fan-fill" style="--rail-color:#6dc5ff; width:0%; background:#6dc5ff;"></div><div class="slider-ticks"><span></span><span></span><span></span><span></span><span></span></div></div>
          <input id="fan-slider" type="range" min="0" max="100" step="1" value="0" aria-label="Fan speed">
          <div class="range-value" id="fan-bubble" style="color:#6dc5ff; left:0%;">0</div>
        </div>
        <div class="quick-presets" style="margin-top: 6px;">
          <button class="preset-btn" data-fan="0"><span class="preset-icon">◌</span><span>OFF</span></button>
          <button class="preset-btn" data-fan="25"><span class="preset-icon">◐</span><span>25%</span></button>
          <button class="preset-btn" data-fan="50"><span class="preset-icon">◔</span><span>50%</span></button>
          <button class="preset-btn" data-fan="75"><span class="preset-icon">◑</span><span>75%</span></button>
          <button class="preset-btn" data-fan="100"><span class="preset-icon">◉</span><span>100%</span></button>
        </div>
      </div>
    </div>

    <div class="panel">
      <div class="panel-title"><span class="icon">💧</span><span>DOSING</span></div>
      <div class="levels-wrap">
        <div class="level-row" style="grid-template-columns: 1fr auto;">
          <div class="label">Remaining</div>
          <div class="value-primer" id="dosing-remaining">00:00</div>
        </div>
        <div class="level-row" style="grid-template-columns: 1fr 1fr; gap: 10px;">
          <button class="segmented" id="dosing-start-btn" type="button">START</button>
          <button class="segmented" id="dosing-stop-btn" type="button">STOP</button>
        </div>
        <div class="level-row" style="grid-template-columns: 1fr auto; align-items: center;">
          <div class="label">Duration</div>
          <div class="slider-output"><input id="dosing-ms" type="number" min="500" max="10000" step="500" value="3000" style="width: 90px; border-radius: 10px; border: 1px solid var(--border); background: rgba(15,23,32,0.9); color: var(--text); padding: 8px 10px; font-weight:700; text-align:center;"></div>
        </div>
      </div>
    </div>

    <div class="panel">
      <div class="panel-title"><span class="icon">⚡</span><span>Quick Presets</span></div>
      <div class="quick-presets">
        <button class="preset-btn" data-preset="sunrise"><span class="preset-icon">☀</span><span>朝焼け</span></button>
        <button class="preset-btn" data-preset="day"><span class="preset-icon">☼</span><span>昼間</span></button>
        <button class="preset-btn" data-preset="sunset"><span class="preset-icon">◐</span><span>夕焼け</span></button>
        <button class="preset-btn" data-preset="night"><span class="preset-icon">☾</span><span>夜</span></button>
        <button class="preset-btn" data-preset="off"><span class="preset-icon">◉</span><span>消灯</span></button>
      </div>
    </div>

    <div class="footer-nav">
      <button class="nav-btn primary"><span class="symbol">▣</span><span>コントロール</span></button>
      <button class="nav-btn"><span class="symbol">◌</span><span>プリセット</span></button>
      <button class="nav-btn"><span class="symbol">⏱</span><span>スケジュール</span></button>
      <button class="nav-btn" id="wifi-settings-btn" type="button"><span class="symbol">⚙</span><span>設定</span></button>
    </div>
  </div>

  <script>
    const CHANNEL_COLORS = {
      r: '#ef5a4d',
      g: '#49d46d',
      b: '#4ea3ff',
      w: '#e4ecff',
      d: '#d268ff'
    };

    let loading = false;
    let isUserDragging = false;
    let levelDebounceTimer = null;
    let holdTimer = null;
    let holdDelta = 0;
    let holdChannel = null;

    function clampChannel(value) {
      return Math.min(255, Math.max(0, value));
    }

    function clampFan(value) {
      return Math.min(100, Math.max(0, value));
    }

    function setSliderVisual(id, value) {
      const input = document.getElementById(id);
      const bubble = document.getElementById(id + '-bubble');
      const fill = input.parentElement.querySelector('.slider-fill');
      const color = CHANNEL_COLORS[id] || '#ffffff';
      const percent = (value / 255) * 100;
      if (fill) {
        fill.style.width = percent + '%';
        fill.style.background = color;
      }
      if (bubble) {
        bubble.textContent = value;
        bubble.style.left = percent + '%';
        bubble.style.color = color;
      }
      input.style.setProperty('--track-color', color);
      const label = document.getElementById(id + '-value');
      if (label) label.textContent = value;
    }

    function applyModeState() {
      const modeValue = Number(document.getElementById('mode-manual').dataset.mode || 1);
      document.getElementById('mode-auto').classList.toggle('active', Number(modeValue) === 0);
      document.getElementById('mode-manual').classList.toggle('active', Number(modeValue) === 1);
    }

    function applyWeatherState() {
      const weatherButtons = document.querySelectorAll('.segmented.weather');
      weatherButtons.forEach((btn) => {
        const isActive = Number(btn.dataset.weather) === Number(window.currentWeather || 0);
        btn.classList.toggle('active', isActive);
      });
    }

    function formatRemaining(ms) {
      const safe = Math.max(0, Number(ms) || 0);
      const totalSeconds = Math.ceil(safe / 1000);
      const minutes = Math.floor(totalSeconds / 60);
      const seconds = totalSeconds % 60;
      return `${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
    }

    function updateStatus() {
      fetch('/status')
        .then((response) => response.json())
        .then((data) => {
          loading = true;
          document.getElementById('time').textContent = data.time || '--:--';
          document.getElementById('date').textContent = data.date || '----/--/-- (-)';
          document.getElementById('phase').textContent = data.phase || '--';
          document.getElementById('wifi').textContent = data.wifi || '未接続';
          document.getElementById('wifi-pill').textContent = data.wifi || '未接続';
          document.getElementById('device-label').textContent = data.deviceId || 'Paludarium Light';
          window.currentWeather = Number(data.weather ?? 0);
          const modeValue = Number(data.mode ?? 1);
          document.getElementById('mode-auto').classList.toggle('active', modeValue === 0);
          document.getElementById('mode-manual').classList.toggle('active', modeValue === 1);
          applyWeatherState();

          const fanValue = Number(data.fan ?? 0);
          const fanSlider = document.getElementById('fan-slider');
          const fanBubble = document.getElementById('fan-bubble');
          const fanFill = document.getElementById('fan-fill');
          if (fanSlider) {
            fanSlider.value = fanValue;
          }
          if (fanBubble) {
            fanBubble.textContent = fanValue;
            fanBubble.style.left = `${fanValue}%`;
          }
          if (fanFill) {
            fanFill.style.width = `${fanValue}%`;
          }
          const fanLabel = document.getElementById('fan-value');
          if (fanLabel) fanLabel.textContent = `${fanValue}%`;

          const remaining = document.getElementById('dosing-remaining');
          if (remaining) {
            remaining.textContent = formatRemaining(Number(data.dosingRemainingMs ?? 0));
          }

          const dosingInput = document.getElementById('dosing-ms');
          const dosingDurationMs = Number(data.dosingDurationMs ?? 3000);
          if (dosingInput && document.activeElement !== dosingInput) {
            dosingInput.value = dosingDurationMs;
          }

          if (!isUserDragging) {
            const values = { r: data.r, g: data.g, b: data.b, w: data.w, d: data.d };
            Object.entries(values).forEach(([key, value]) => {
              const input = document.getElementById(key);
              if (!input) return;
              input.value = value;
              setSliderVisual(key, Number(value));
            });
          }
          loading = false;
        })
        .catch(() => {
          loading = false;
        });
    }

    function setMode(value) {
      fetch('/mode?m=' + value).then(updateStatus);
    }

    function setWeather(value) {
      fetch('/weather?w=' + value).then(updateStatus);
    }

    let fanDebounceTimer = null;

    function setFan(value) {
      const safe = clampFan(Number(value) || 0);
      if (!Number.isFinite(safe)) return;
      const fanSlider = document.getElementById('fan-slider');
      if (fanSlider) fanSlider.value = safe;
      const fanBubble = document.getElementById('fan-bubble');
      const fanFill = document.getElementById('fan-fill');
      const fanLabel = document.getElementById('fan-value');
      if (fanBubble) {
        fanBubble.textContent = safe;
        fanBubble.style.left = `${safe}%`;
      }
      if (fanFill) fanFill.style.width = `${safe}%`;
      if (fanLabel) fanLabel.textContent = `${safe}%`;

      clearTimeout(fanDebounceTimer);
      fanDebounceTimer = setTimeout(() => {
        fetch('/fan?speed=' + safe).then(updateStatus);
      }, 80);
    }

    function sendDosingStart() {
      const ms = Math.max(500, Math.min(10000, Number(document.getElementById('dosing-ms').value) || 3000));
      document.getElementById('dosing-ms').value = ms;
      fetch('/dosing/start?ms=' + ms).then(updateStatus);
    }

    function sendDosingStop() {
      fetch('/dosing/stop').then(updateStatus);
    }

    function sendLevels() {
      if (loading) return;

      const params = ['r', 'g', 'b', 'w', 'd']
        .map((id) => `${id}=${clampChannel(Number(document.getElementById(id).value))}`)
        .join('&');

      clearTimeout(levelDebounceTimer);
      levelDebounceTimer = setTimeout(() => {
        fetch('/levels?' + params);
      }, 80);
    }

    function updateChannelFromInput(id) {
      const input = document.getElementById(id);
      const value = clampChannel(Number(input.value));
      input.value = value;
      setSliderVisual(id, value);
      if (!isUserDragging) {
        sendLevels();
        return;
      }
      sendLevels();
    }

    function adjustLevel(id, delta) {
      const input = document.getElementById(id);
      const current = clampChannel(Number(input.value) + delta);
      input.value = current;
      setSliderVisual(id, current);
      sendLevels();
    }

    function startHold(channel, delta) {
      stopHold();
      holdChannel = channel;
      holdDelta = delta;
      adjustLevel(channel, delta);
      holdTimer = setInterval(() => {
        adjustLevel(channel, delta);
      }, 120);
    }

    function stopHold() {
      if (holdTimer) {
        clearInterval(holdTimer);
        holdTimer = null;
      }
      holdChannel = null;
    }

    document.querySelectorAll('#r, #g, #b, #w, #d').forEach((slider) => {
      slider.addEventListener('input', () => {
        const id = slider.id;
        const value = clampChannel(Number(slider.value));
        slider.value = value;
        setSliderVisual(id, value);
        sendLevels();
      });

      slider.addEventListener('pointerdown', () => {
        isUserDragging = true;
      });

      slider.addEventListener('pointerup', () => {
        isUserDragging = false;
      });

      slider.addEventListener('pointerleave', () => {
        isUserDragging = false;
      });

      slider.addEventListener('touchstart', () => {
        isUserDragging = true;
      }, { passive: true });

      slider.addEventListener('touchend', () => {
        isUserDragging = false;
      }, { passive: true });
    });

    document.getElementById('fan-slider')?.addEventListener('input', (event) => {
      const value = clampFan(Number(event.target.value) || 0);
      setFan(value);
    });

    document.querySelectorAll('.step-btn').forEach((button) => {
      const channel = button.dataset.channel;
      const delta = Number(button.dataset.step || 1);
      button.addEventListener('pointerdown', (event) => {
        event.preventDefault();
        startHold(channel, delta);
      });
      button.addEventListener('pointerup', stopHold);
      button.addEventListener('pointerleave', stopHold);
      button.addEventListener('pointercancel', stopHold);
    });

    document.querySelectorAll('.segmented').forEach((button) => {
      if (button.dataset.mode !== undefined) {
        button.addEventListener('click', () => setMode(Number(button.dataset.mode)));
      }
      if (button.dataset.weather !== undefined) {
        button.addEventListener('click', () => setWeather(Number(button.dataset.weather)));
      }
    });

    document.getElementById('fan-slider')?.addEventListener('input', (event) => {
      const value = Math.max(0, Math.min(100, Number(event.target.value) || 0));
      const bubble = document.getElementById('fan-bubble');
      const fill = document.getElementById('fan-fill');
      if (bubble) {
        bubble.textContent = value;
        bubble.style.left = `${value}%`;
      }
      if (fill) fill.style.width = `${value}%`;
      const label = document.getElementById('fan-value');
      if (label) label.textContent = `${value}%`;
      setFan(value);
    });

    document.querySelectorAll('[data-fan]').forEach((button) => {
      button.addEventListener('click', () => setFan(Number(button.dataset.fan || 0)));
    });

    const dosingInput = document.getElementById('dosing-ms');
    dosingInput?.addEventListener('change', () => {
      const ms = Math.max(500, Math.min(10000, Number(dosingInput.value) || 3000));
      dosingInput.value = ms;
      fetch('/dosing/config?ms=' + ms).then(updateStatus);
    });

    document.getElementById('dosing-start-btn')?.addEventListener('click', sendDosingStart);
    document.getElementById('dosing-stop-btn')?.addEventListener('click', sendDosingStop);

    document.querySelectorAll('.preset-btn').forEach((button) => {
      button.addEventListener('click', () => {
        const preset = button.dataset.preset;
        const target = {
          sunrise: { r: 255, g: 150, b: 80, w: 90, d: 220 },
          day: { r: 255, g: 255, b: 255, w: 255, d: 255 },
          sunset: { r: 255, g: 120, b: 60, w: 40, d: 220 },
          night: { r: 30, g: 20, b: 80, w: 12, d: 180 },
          off: { r: 0, g: 0, b: 0, w: 0, d: 0 }
        }[preset] || { r: 255, g: 255, b: 255, w: 255, d: 255 };

        Object.entries(target).forEach(([key, value]) => {
          const input = document.getElementById(key);
          input.value = value;
          setSliderVisual(key, Number(value));
        });

        sendLevels();
      });
    });

    Object.keys(CHANNEL_COLORS).forEach((key) => {
      setSliderVisual(key, Number(document.getElementById(key).value));
    });

    updateStatus();
    setInterval(updateStatus, 3000);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  if (apMode || !WiFi.isConnected()) {
    server.send_P(200, "text/html; charset=utf-8", WIFI_SETUP_HTML);
    return;
  }
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleWifiPage() {
  server.send_P(200, "text/html; charset=utf-8", WIFI_SETUP_HTML);
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/wifi", handleWifiPage);
  server.on("/status", handleStatus);
  server.on("/mode", handleMode);
  server.on("/weather", handleWeather);
  server.on("/levels", handleLevels);
  server.on("/fan", handleFan);
  server.on("/dosing/start", handleDosingStart);
  server.on("/dosing/stop", handleDosingStop);
  server.on("/dosing/config", handleDosingConfig);
  server.on("/wifi/scan", handleWifiScan);
  server.on("/wifi/results", handleWifiResults);
  server.on("/wifi/save", handleWifiSave);
  server.on("/wifi/saved", handleWifiSaved);
  server.on("/wifi/delete", handleWifiDelete);

  server.onNotFound([]() {
    if (apMode) {
      server.sendHeader("Location", "/wifi", true);
      server.send(302, "text/plain; charset=utf-8", "Redirecting to Wi‑Fi setup");
      return;
    }
    server.send(404, "text/plain; charset=utf-8", "Not Found");
  });

  server.begin();
}

// -----------------------------------------------------------------------------
// Wi-Fi
// -----------------------------------------------------------------------------

void startWiFi() {
  Serial.println("[W1] startWiFi entered");
  Serial.flush();

  Serial.println("[W2] loadSavedNetworks start");
  Serial.flush();
  loadSavedNetworks();
  Serial.println("[W2] loadSavedNetworks OK");
  Serial.flush();

  Serial.println("[W3] WiFi.mode(WIFI_STA) start");
  Serial.flush();
  WiFi.mode(WIFI_STA);
  Serial.println("[W3] WiFi.mode(WIFI_STA) OK");
  Serial.flush();

  Serial.println("[W4] WiFi.hostname start");
  Serial.flush();
  WiFi.hostname(hostname.c_str());
  Serial.println("[W4] WiFi.hostname OK");
  Serial.flush();

  Serial.println("[W5] WiFi.disconnect start");
  Serial.flush();
  WiFi.disconnect(false);
  Serial.println("[W5] WiFi.disconnect OK");
  Serial.flush();

  Serial.println("[W6] WiFi.scanDelete start");
  Serial.flush();
  WiFi.scanDelete();
  Serial.println("[W6] WiFi.scanDelete OK");
  Serial.flush();

  if (!apMode) {
    Serial.println("[W6A] triggerSetupMode start");
    Serial.flush();
    triggerSetupMode();
    Serial.println("[W6A] triggerSetupMode OK");
    Serial.flush();
  }

  wifiScanPending = false;
  wifiState = WIFI_STATE_IDLE;
  wifiStateChangedMs = millis();
  pendingWifiReconnectMs = 0;
  pendingWifiReconnectSsid = "";
  pendingWifiReconnectPassword = "";
  savedNetworkAttemptIndex = 0;

  Serial.print("[W7] savedNetworkCount = ");
  Serial.println(savedNetworkCount);
  Serial.flush();

  Serial.print("[W8] secrets configured = ");
  Serial.println(hasConfiguredSecretsWifi() ? "YES" : "NO");
  Serial.flush();

  if (savedNetworkCount > 0) {
    Serial.println("[W9] trySavedNetworks start");
    Serial.flush();

    const bool result = trySavedNetworks();

    Serial.print("[W9] trySavedNetworks returned: ");
    Serial.println(result ? "true" : "false");
    Serial.flush();
    return;
  }

  if (hasConfiguredSecretsWifi()) {
    Serial.println("[W10] attemptWifiConnection start");
    Serial.flush();

    attemptWifiConnection(WIFI_SSID, WIFI_PASSWORD);

    Serial.println("[W10] attemptWifiConnection returned");
    Serial.flush();
    return;
  }

  Serial.println("[W11] triggerSetupMode start");
  Serial.flush();

  triggerSetupMode();

  Serial.println("[W11] triggerSetupMode returned");
  Serial.flush();
}

void maintainWiFi() {
  if (apMode) {
    dnsServer.processNextRequest();
  }

  if (pendingWifiReconnectMs != 0 && millis() >= pendingWifiReconnectMs) {
    if (pendingWifiReconnectSsid.length() > 0) {
      attemptWifiConnection(pendingWifiReconnectSsid, pendingWifiReconnectPassword);
    }
    pendingWifiReconnectMs = 0;
    pendingWifiReconnectSsid = "";
    pendingWifiReconnectPassword = "";
  }

  const wl_status_t currentStatus = WiFi.status();

  if (currentStatus == WL_CONNECTED) {
    if (wifiState != WIFI_STATE_CONNECTED) {
      wifiState = WIFI_STATE_CONNECTED;
      wifiStateChangedMs = millis();
      if (apMode) {
        stopSetupAP();
      }
      startMDNS();
      Serial.println();
      Serial.print("Wi-Fi connected. IP: ");
      Serial.println(WiFi.localIP());
    }
    previousWifiStatus = currentStatus;
    return;
  }

  if (currentStatus != previousWifiStatus) {
    previousWifiStatus = currentStatus;
    Serial.println("Wi-Fi disconnected. DMX transmission continues.");
  }

  switch (wifiState) {
    case WIFI_STATE_IDLE:
      if (savedNetworkCount > 0) {
        if (!apMode) {
          triggerSetupMode();
        }
        trySavedNetworks();
      } else if (hasConfiguredSecretsWifi()) {
        attemptWifiConnection(WIFI_SSID, WIFI_PASSWORD);
      } else if (!apMode) {
        triggerSetupMode();
      }
      return;

    case WIFI_STATE_SCAN_START:
      if ((millis() - wifiStateChangedMs) >= WIFI_SCAN_TIMEOUT_MS) {
        WiFi.scanDelete();
        wifiScanPending = false;
        wifiState = WIFI_STATE_IDLE;
        wifiStateChangedMs = millis();
        Serial.println("[SCAN] timeout, returning to idle");
        Serial.flush();
        return;
      }
      WiFi.scanNetworks(true, true);
      wifiScanPending = true;
      wifiScanJsonCache = "[]";
      wifiState = WIFI_STATE_SCANNING;
      wifiStateChangedMs = millis();
      return;

    case WIFI_STATE_SCANNING: {
      const int found = WiFi.scanComplete();
      if (found < 0) {
        if ((millis() - wifiStateChangedMs) >= WIFI_SCAN_TIMEOUT_MS) {
          WiFi.scanDelete();
          wifiScanPending = false;
          wifiState = WIFI_STATE_IDLE;
          wifiStateChangedMs = millis();
          Serial.println("[SCAN] timeout, continuing with AP mode");
          Serial.flush();
        }
        return;
      }

      wifiScanJsonCache = buildScanJson();
      wifiScanPending = false;
      wifiState = (WiFi.status() == WL_CONNECTED) ? WIFI_STATE_CONNECTED : WIFI_STATE_FAILED;
      wifiStateChangedMs = millis();
      if (!apMode && WiFi.status() != WL_CONNECTED) {
        triggerSetupMode();
      }
      return;
    }

    case WIFI_STATE_CONNECTING:
      if ((millis() - wifiStateChangedMs) >= WIFI_SAVED_NETWORK_TIMEOUT_MS) {
        WiFi.disconnect(false);
        savedNetworkAttemptIndex += 1;
        if (savedNetworkAttemptIndex < savedNetworkCount) {
          trySavedNetworks();
          Serial.print("[W] retry saved network index: ");
          Serial.println(savedNetworkAttemptIndex);
          Serial.flush();
        } else {
          savedNetworkAttemptIndex = 0;
          wifiState = WIFI_STATE_FAILED;
          wifiStateChangedMs = millis();
          if (!apMode) {
            triggerSetupMode();
          }
          Serial.println("[W] all saved networks failed, AP remains available");
          Serial.flush();
        }
      }
      return;

    case WIFI_STATE_FAILED:
      if (apMode) {
        return;
      }
      triggerSetupMode();
      return;

    case WIFI_STATE_CONNECTED:
    default:
      return;
  }
}

// -----------------------------------------------------------------------------
// Arduino setup / loop
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("Paludarium DMX controller starting...");
  Serial.print("Reset reason code: ");
  Serial.println(static_cast<int>(esp_reset_reason()));

  // MAC由来の固有IDを、Wi-Fi・mDNS初期化より先に確定する。
  initializeDeviceIdentity();

  Serial.println("[1] loadSettings start");
  loadSettings();
  Serial.println("[1] loadSettings OK");

  // DMXを最初に起動する。Wi-Fi接続待ちでDMXを止めない。
  Serial.println("[2] setupDMX start");
  setupDMX();
  Serial.println("[2] setupDMX OK");

  Serial.println("[3] setupFan start");
  setupFan();
  Serial.println("[3] setupFan OK");

  Serial.println("[4] setupDosing start");
  setupDosing();
  Serial.println("[4] setupDosing OK");

  Serial.println("[5] loadSavedNetworks start");
  loadSavedNetworks();
  Serial.println("[5] loadSavedNetworks OK");

  Serial.println("[6] startWiFi start");
  startWiFi();
  Serial.println("[6] startWiFi OK");

  Serial.println("[7] configTime start");
  configTime(
    GMT_OFFSET_SEC,
    DAYLIGHT_OFFSET_SEC,
    NTP_SERVER
  );
  Serial.println("[7] configTime OK");

  Serial.println("[8] setupWebServer start");
  setupWebServer();
  Serial.println("[8] setupWebServer OK");

  Serial.println("SETUP COMPLETED");
}

void loop() {
  const uint32_t now = millis();

  // 約25msごとにDMXフレームを繰り返し送信する。
  if (now - lastDmxMs >= DMX_FRAME_INTERVAL_MS) {
    lastDmxMs = now;
    sendDMXFrame();
  }

  server.handleClient();
  maintainWiFi();
  updateDosing();
  saveSettings();

  if (now - lastTimeMs >= 1000) {
    lastTimeMs = now;
    updateTimeDisplay();
  }

  if (now - lastLightMs >= 250) {
    lastLightMs = now;
    updateAutomaticLight();
  }

  delay(1);
}
