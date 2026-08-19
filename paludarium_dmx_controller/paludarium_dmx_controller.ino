#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <math.h>

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
// Wi-Fi：自分の値へ書き換える。パスワードを共有するときは必ず伏せる。
// -----------------------------------------------------------------------------

const char* WIFI_SSID = "IODATA-b0c620-2G";
const char* WIFI_PASSWORD = "7218236646609";

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

String phase = "手動";
String timeStr = "--:--";

uint32_t lastDmxMs = 0;
uint32_t lastTimeMs = 0;
uint32_t lastLightMs = 0;
uint32_t lastWifiRetryMs = 0;
wl_status_t previousWifiStatus = WL_IDLE_STATUS;

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
    if (mode == 0) phase = "時刻未同期";
    return;
  }

  char buffer[6];
  snprintf(buffer, sizeof(buffer), "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
  timeStr = buffer;

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
                              preferences.isKey("manual_d");

  if (hasSavedValues) {
    mode = preferences.getUChar("mode", mode);
    weather = preferences.getUChar("weather", weather);
    red = preferences.getUChar("manual_r", red);
    green = preferences.getUChar("manual_g", green);
    blue = preferences.getUChar("manual_b", blue);
    white = preferences.getUChar("manual_w", white);
    masterDimmer = preferences.getUChar("manual_d", masterDimmer);
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
  preferences.end();

  settingsDirty = false;
}

// -----------------------------------------------------------------------------
// Web API
// -----------------------------------------------------------------------------

void handleStatus() {
  String json;
  json.reserve(256);

  json = "{";
  json += "\"r\":" + String(red) + ",";
  json += "\"g\":" + String(green) + ",";
  json += "\"b\":" + String(blue) + ",";
  json += "\"w\":" + String(white) + ",";
  json += "\"d\":" + String(masterDimmer) + ",";
  json += "\"mode\":" + String(mode) + ",";
  json += "\"weather\":" + String(weather) + ",";
  json += "\"time\":\"" + timeStr + "\",";
  json += "\"phase\":\"" + phase + "\",";

  if (WiFi.status() == WL_CONNECTED) {
    json += "\"wifi\":\"" + WiFi.localIP().toString() + "\"";
  } else {
    json += "\"wifi\":\"未接続\"";
  }

  json += "}";
  server.send(200, "application/json; charset=utf-8", json);
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
        <span>Paludarium Light</span>
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
          <div id="date" class="status-sub">2025/06/01 (日)</div>
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
      <div class="panel-title"><span class="icon">✦</span><span>Mode</span></div>
      <div class="mode-grid">
        <button class="segmented" data-mode="0" id="mode-auto">AUTO</button>
        <button class="segmented active" data-mode="1" id="mode-manual">MANUAL</button>
      </div>
    </div>

    <div class="panel">
      <div class="panel-title"><span class="icon">☼</span><span>Weather</span></div>
      <div class="weather-grid">
        <button class="segmented weather active" data-weather="0" id="weather-sunny">Sunny</button>
        <button class="segmented weather" data-weather="1" id="weather-cloudy">Cloudy</button>
        <button class="segmented weather" data-weather="2" id="weather-rain">Rain</button>
      </div>
    </div>

    <div class="panel">
      <div class="panel-title"><span class="icon">▤</span><span>Levels</span></div>

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
      <button class="nav-btn"><span class="symbol">⚙</span><span>設定</span></button>
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

    function updateStatus() {
      fetch('/status')
        .then((response) => response.json())
        .then((data) => {
          loading = true;
          document.getElementById('time').textContent = data.time || '--:--';
          document.getElementById('phase').textContent = data.phase || '--';
          document.getElementById('wifi').textContent = data.wifi || '未接続';
          document.getElementById('wifi-pill').textContent = data.wifi || '未接続';
          window.currentWeather = Number(data.weather ?? 0);
          const modeValue = Number(data.mode ?? 1);
          document.getElementById('mode-auto').classList.toggle('active', modeValue === 0);
          document.getElementById('mode-manual').classList.toggle('active', modeValue === 1);
          applyWeatherState();

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

    document.querySelectorAll('input[type="range"]').forEach((slider) => {
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

        // 既存の /levels API のみを使い、バックエンド新設は行わない。
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
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/mode", handleMode);
  server.on("/weather", handleWeather);
  server.on("/levels", handleLevels);

  server.onNotFound([]() {
    server.send(404, "text/plain; charset=utf-8", "Not Found");
  });

  server.begin();
}

// -----------------------------------------------------------------------------
// Wi-Fi
// -----------------------------------------------------------------------------

void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  previousWifiStatus = WiFi.status();
  lastWifiRetryMs = millis();
}

void maintainWiFi() {
  const wl_status_t currentStatus = WiFi.status();

  if (currentStatus != previousWifiStatus) {
    previousWifiStatus = currentStatus;

    if (currentStatus == WL_CONNECTED) {
      Serial.println();
      Serial.print("Wi-Fi connected. IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("Wi-Fi disconnected. DMX transmission continues.");
    }
  }

  // Wi-Fiに接続できなくてもDMX送信は止めない。
  if (currentStatus != WL_CONNECTED && millis() - lastWifiRetryMs >= 30000) {
    Serial.println("Retrying Wi-Fi connection...");
    WiFi.reconnect();
    lastWifiRetryMs = millis();
  }
}

// -----------------------------------------------------------------------------
// Arduino setup / loop
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("Paludarium DMX controller starting...");

  loadSettings();

  // DMXを最初に起動する。Wi-Fi接続待ちでDMXを止めない。
  setupDMX();

  startWiFi();
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  setupWebServer();

  Serial.println("DMX started: TX=GPIO18, DE/RE=GPIO4");
  Serial.println("Initial output: Red=255, Master=255, MANUAL mode");
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
