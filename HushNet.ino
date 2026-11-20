// Dual-panel noise monitor + ESP-NOW (broadcast demo)
// Optimized UI refresh: only redraw changed regions to reduce visible flicker.
// Gradient bar added (green -> yellow -> red).

#include <Arduino.h>
#include "esp32-hal-ledc.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <WiFi.h>
#include <esp_now.h>

#define TFT_CS   5
#define TFT_DC   2
#define TFT_RST  4

#define MIC_PIN 34   // ADC1 pin (works with Wi-Fi)
#define BUZZER_PIN 25

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// ---------------- LED BAR ----------------
const int LED_COUNT = 9;
const int ledPins[LED_COUNT] = {13, 14, 16, 17, 21, 22, 26, 27, 32};
const int LEDC_FREQ = 5000;
const int LEDC_RES  = 8;

static inline void setLedDuty(int idx, uint8_t duty) {
  if (idx < 0 || idx >= LED_COUNT) return;
  ledcWrite(ledPins[idx], duty);
}
void setupLedPins() {
  for (int i = 0; i < LED_COUNT; i++) {
    pinMode(ledPins[i], OUTPUT);
    ledcAttach(ledPins[i], LEDC_FREQ, LEDC_RES);
    ledcWrite(ledPins[i], 0);
  }
}
void updateLEDBar(int level) {
  if (level < 0) level = 0;
  if (level > 100) level = 100;
  float scaled = ((float)level * LED_COUNT) / 100.0f;
  int full = (int)floor(scaled);
  float frac = scaled - (float)full;
  for (int i = 0; i < LED_COUNT; ++i) {
    if (i < full) setLedDuty(i, 255);
    else if (i == full && frac > 0.0001f) setLedDuty(i, (uint8_t)(frac * 255.0f + 0.5f));
    else setLedDuty(i, 0);
  }
}

// ---------------- audio / smoothing / thresholds ----------------
const int BURST_SAMPLES = 60;
const int BURST_US = 60;

const float RISE_ALPHA  = 0.30f;
const float FALL_ALPHA  = 0.03f;
const float AMP_ALPHA = 0.18f;
float ampAvg = 0.0f;
float smoothedLevel = 0.0f;
int noiseBaseline = 0;
float envelope = 0;
const float ENVELOPE_DECAY = 0.90f;
float longTermNoise = 0.0;
const float LONG_TERM_ALPHA = 0.02;
const int ALARM_ON_LEVEL = 75;
const int ALARM_OFF_LEVEL = 55;
const unsigned long ALARM_ON_TIME  = 2000;
const unsigned long ALARM_OFF_TIME = 2000;
bool alarmActive = false;
unsigned long alarmTimer = 0;
bool buzzerState = false;
unsigned long lastBeepTime = 0;
const unsigned long BEEP_ON_TIME  = 200;
const unsigned long BEEP_OFF_TIME = 800;
int peakLevel = 0;
unsigned long peakTimer = 0;
const unsigned long PEAK_HOLD_MS = 1000;
const int PEAK_DECAY_STEP = 1;
int MAP_OFFSET = 10;
int SENSITIVITY_RANGE = 1500;
bool buzzerMuteMic = false;
const unsigned long BUZZER_MIC_DELAY = 200;  // ms ignore after beep
unsigned long buzzerMicReleaseTime = 0;

// ---------------- UI layout ----------------
const int SCREEN_W = 320;
const int SCREEN_H = 240;
const int PANEL_MARGIN = 8;
const int PANEL_W = SCREEN_W - 2 * PANEL_MARGIN;
const int PANEL_H = (SCREEN_H - 3 * PANEL_MARGIN) / 2;
const int TITLE_Y_OFFSET = 6;
const int NUM_X_OFFSET = PANEL_MARGIN + 220; // numeric area
const int BAR_X = PANEL_MARGIN + 12;
const int BAR_W = PANEL_W - 24;
const int BAR_H = 28;
const int BAR_Y_OFFSET = 36;
const int STATUS_Y_OFFSET = 76;
const int DYN_NUM_W = 100;
const int DYN_NUM_H = 28;
const int DYN_BAR_H = BAR_H;
const int DYN_STATUS_H = 18;
const unsigned long UI_UPDATE_MS = 40; // 25 FPS
unsigned long lastUIUpdate = 0;

// remote placeholders
int remoteDisplayLevel = 0;
int remotePeakLevel = 0;
unsigned long lastReceivedMs = 0;

// ESP-NOW params
uint32_t DEVICE_ID = 2;
const unsigned long SEND_INTERVAL_MS = 500UL;
unsigned long lastSendMs = 0;
uint8_t localMac[6];
float remoteSmoothed = 0.0f;
const float REMOTE_ALPHA = 0.6f;
unsigned long lastRadioActivityMs = 0;
const unsigned long RADIO_BLANK_MS = 15UL;
uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
bool espNowReady = false;
const unsigned long NO_LINK_TIMEOUT = 5000UL;

// remote alarm additions
float remoteLongTermNoise = 0.0f;
bool remoteAlarmActive = false;
unsigned long remoteAlarmTimer = 0;

// remember whether popup shown (fix for the missing symbol)
bool alarmPopupShown = false;

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

// Message struct
typedef struct __attribute__((packed)) {
  uint8_t noise;
  uint8_t peak;
  uint8_t alarm;   
  uint32_t deviceId;
} esp_msg_t;
esp_msg_t outgoingMsg;
esp_msg_t incomingMsg;

int prevAmplitude = 0;

// ---------------- previous-state (for minimal redraw) ----------------
// local previous
int prevLocalLevel = -1;
int prevLocalPeak = -1;
int prevLocalStatus = -1; // 0 quiet,1 mod,2 loud
int prevLocalBarW = -1;

// remote previous
int prevRemoteLevel = -1;
int prevRemotePeak = -1;
int prevRemoteStatus = -1;
int prevRemoteBarW = -1;
bool prevRemoteAlarmActive = false;
unsigned long prevLastReceivedMs = 0;

// prototypes
int readAmplitude();
void drawStaticUI();
void drawPanelDynamic(int topY, const char *title, int level, int peak, bool isLocal);
void calibrateNoise();
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
void drawAlarmPopup();
void drawGradientRange(int x, int y, int w, int h);

static inline int textPixelWidth(const char *s, uint8_t size) {
  int len = strlen(s);
  return len * 6 * (int)size;
}
static inline int levelToWidth(int level) {
  int w = map(level, 0, 100, 0, BAR_W);
  w = constrain(w, 0, BAR_W);
  return w;
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN, ADC_11db);
  setupLedPins();
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  tft.begin();
  tft.setRotation(1);

  drawStaticUI();
  calibrateNoise();
  drawStaticUI();

  WiFi.mode(WIFI_STA);
  WiFi.macAddress(localMac);
  Serial.print("Local MAC: ");
  for (int i=0;i<6;++i){ Serial.printf("%02X", localMac[i]); if (i<5) Serial.print(":"); }
  Serial.println();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    espNowReady = false;
  } else {
    espNowReady = true;
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_err_t addRes = esp_now_add_peer(&peerInfo);
    if (addRes != ESP_OK) {
      Serial.print("esp_now_add_peer returned ");
      Serial.println(addRes);
    } else {
      Serial.println("Broadcast peer added");
    }
  }
}

void loop() {

  // Keep microphone muted until buzzer vibration fully stops
  if (millis() < buzzerMicReleaseTime) {
    buzzerMuteMic = true;
  } else {
    buzzerMuteMic = false;
  }

  int amplitude;

  if (buzzerMuteMic) {
    amplitude = prevAmplitude;   // completely skip mic reading during beep
  }
  else {
    amplitude = readAmplitude();

    // Radio spike protection
    if (millis() - lastRadioActivityMs < RADIO_BLANK_MS) {
      amplitude = prevAmplitude;
    }
    else {
      prevAmplitude = amplitude;
    }
  }
  
  ampAvg = (1.0f - AMP_ALPHA) * ampAvg + AMP_ALPHA * (float)amplitude;

  const float rawWeight = 0.6f;
  const float avgWeight = 0.4f;
  int effectiveAmp = (int)(rawWeight * (float)amplitude + avgWeight * ampAvg + 0.5f);

  if (effectiveAmp > envelope) envelope = effectiveAmp;
  else envelope *= ENVELOPE_DECAY;

  effectiveAmp = (int)envelope;
  const int SPIKE_MAX = 3800;
  if (effectiveAmp > SPIKE_MAX) effectiveAmp = prevAmplitude;

  if (alarmActive) {
    // force envelope to decay faster during alarm
    envelope *= 0.90f;
    // gently pull the smoothed level downward
    smoothedLevel *= 0.97f;
  }

  int mapLow = noiseBaseline + MAP_OFFSET;
  int noiseLevel = map(effectiveAmp, mapLow, mapLow + SENSITIVITY_RANGE, 0, 100);
  noiseLevel = constrain(noiseLevel, 0, 100);

  // Ignore buzzer influence
  if (buzzerState || buzzerMuteMic) {
    noiseLevel = smoothedLevel; 
  }

  float alpha = (noiseLevel > smoothedLevel) ? RISE_ALPHA : FALL_ALPHA;
  smoothedLevel = (1.0f - alpha) * smoothedLevel + alpha * (float)noiseLevel;
  int displayLevel = (int)(smoothedLevel + 0.5f);

  if (!buzzerState && !buzzerMuteMic) {
    longTermNoise = (1.0 - LONG_TERM_ALPHA) * longTermNoise + LONG_TERM_ALPHA * displayLevel;
  }

  unsigned long now = millis();

  // local alarm hysteresis
  if (!alarmActive) {
    if (longTermNoise > ALARM_ON_LEVEL) {
      if (alarmTimer == 0) alarmTimer = now;
      if (now - alarmTimer > ALARM_ON_TIME) { alarmActive = true; alarmTimer = 0; }
    } else if (longTermNoise > (ALARM_ON_LEVEL - 10)) { }
    else alarmTimer = 0;
  } else {
    if (longTermNoise < ALARM_OFF_LEVEL) {
      if (alarmTimer == 0) alarmTimer = now;
      if (now - alarmTimer > ALARM_OFF_TIME) { alarmActive = false; alarmTimer = 0; }
    } else alarmTimer = 0;
  }

  // peak hold
  if (noiseLevel > peakLevel) { peakLevel = noiseLevel; peakTimer = millis(); }
  else {
    unsigned long dt = millis() - peakTimer;
    if (dt > PEAK_HOLD_MS) {
      if (peakLevel > displayLevel + PEAK_DECAY_STEP) peakLevel -= PEAK_DECAY_STEP;
      else peakLevel = displayLevel;
    }
  }

  // Update remote long-term noise (decay when no link)
  if (millis() - lastReceivedMs < NO_LINK_TIMEOUT) {
    remoteLongTermNoise = (1.0 - LONG_TERM_ALPHA) * remoteLongTermNoise + LONG_TERM_ALPHA * remoteDisplayLevel;
  } else {
    remoteLongTermNoise = (1.0 - LONG_TERM_ALPHA) * remoteLongTermNoise;
  }

  // UI updates (only redraw changed regions inside function)
  if (now - lastUIUpdate >= UI_UPDATE_MS) {
    drawPanelDynamic(PANEL_MARGIN, "LOCAL", displayLevel, peakLevel, true);

    int remoteLevel = remoteDisplayLevel;
    int remotePeak  = remotePeakLevel;
    // If remote timed out, force "no link" state
    if (millis() - lastReceivedMs > NO_LINK_TIMEOUT) {
      remoteLevel = -1;
      remotePeak  = 0;
    }
    drawPanelDynamic(PANEL_MARGIN * 2 + PANEL_H, "REMOTE", remoteLevel, remotePeak, false);
    lastUIUpdate = now;
  }

  updateLEDBar(displayLevel);

  // Alarm popup handling (local)
  if (alarmActive) {
    drawAlarmPopup();
  } else if (alarmPopupShown) {
    // restore static UI region where popup would have covered (we redraw static UI once)
    drawStaticUI();
    // force prev state clear to ensure panel redraw next cycle
    prevLocalLevel = prevLocalPeak = prevLocalStatus = -1;
    prevLocalBarW = -1;
    prevRemoteLevel = prevRemotePeak = prevRemoteStatus = -1;
    prevRemoteBarW = -1;
    prevRemoteAlarmActive = !remoteAlarmActive; // force immediate redraw of remote alarm area
    alarmPopupShown = false;
  }

  // send periodic ESP-NOW
  if (espNowReady && (now - lastSendMs >= SEND_INTERVAL_MS)) {
    outgoingMsg.noise = (uint8_t)(smoothedLevel + 0.5f);
    outgoingMsg.peak = (uint8_t)peakLevel;
    outgoingMsg.alarm = alarmActive;
    outgoingMsg.deviceId = DEVICE_ID;
    lastRadioActivityMs = millis();
    esp_err_t res = esp_now_send(broadcastAddress, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));
    (void)res;
    lastSendMs = now;
  }

  // debug
  Serial.print("amp="); Serial.print(prevAmplitude);
  Serial.print(" disp="); Serial.println((int)smoothedLevel);

  // buzzer beeping
  if (alarmActive) {

    if (!buzzerState) {
      if (millis() - lastBeepTime > BEEP_OFF_TIME) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = true;
        lastBeepTime = millis();
        buzzerMuteMic = true;   // mute microphone while beep plays
      }
    } 
    else {
      if (millis() - lastBeepTime > BEEP_ON_TIME) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
        lastBeepTime = millis();

        buzzerMicReleaseTime = millis() + BUZZER_MIC_DELAY;
      }
    }

  } else {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerState = false;
    buzzerMuteMic = false;
  }
}

// ---------------- helpers ----------------
int readAmplitude() {
  int signalMax = 0;
  int signalMin = 4095;
  for (int i = 0; i < BURST_SAMPLES; ++i) {
    int v = analogRead(MIC_PIN);
    if (v > signalMax) signalMax = v;
    if (v < signalMin) signalMin = v;
    delayMicroseconds(BURST_US);
  }
  return signalMax - signalMin;
}

void calibrateNoise() {
  tft.fillRect(0, 70, 320, 100, ILI9341_WHITE);
  tft.setCursor(35, 95);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.println("Calibrating...");
  tft.setCursor(40, 140);
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(1);
  tft.println("Please keep quiet while calibrating...");

  long totalAmp = 0;
  int samples = 0;
  unsigned long start = millis();
  while (millis() - start < 3000) {
    int amp = readAmplitude();
    totalAmp += amp;
    samples++;
    delay(8);
  }
  noiseBaseline = (samples > 0) ? (totalAmp / samples) : 0;
  Serial.print("Baseline amplitude: ");
  Serial.println(noiseBaseline);
}

void drawStaticUI() {
  tft.fillScreen(ILI9341_BLACK);

  // outer panels
  tft.drawRect(PANEL_MARGIN, PANEL_MARGIN, PANEL_W, PANEL_H, ILI9341_WHITE);
  tft.drawRect(PANEL_MARGIN, PANEL_MARGIN * 2 + PANEL_H, PANEL_W, PANEL_H, ILI9341_WHITE);

  // Titles
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setCursor(PANEL_MARGIN + 8, PANEL_MARGIN + TITLE_Y_OFFSET);
  tft.print("LOCAL");
  tft.setCursor(PANEL_MARGIN + 8, PANEL_MARGIN * 2 + PANEL_H + TITLE_Y_OFFSET);
  tft.print("REMOTE");

  // bars outlines
  int barTopY = PANEL_MARGIN + BAR_Y_OFFSET;
  tft.drawRect(BAR_X - 1, barTopY - 1, BAR_W + 2, BAR_H + 2, ILI9341_WHITE);
  int barBottomY = PANEL_MARGIN * 2 + PANEL_H + BAR_Y_OFFSET;
  tft.drawRect(BAR_X - 1, barBottomY - 1, BAR_W + 2, BAR_H + 2, ILI9341_WHITE);

  // labels
  tft.setTextSize(2);
  tft.setCursor(PANEL_MARGIN + 12, PANEL_MARGIN + 80);
  tft.print("Noise Level:");
  tft.setCursor(PANEL_MARGIN + 12, PANEL_MARGIN * 2 + PANEL_H + 80);
  tft.print("Noise Level:");

  // reset previous-state so first draw will render everything
  prevLocalLevel = prevLocalPeak = prevLocalStatus = -1;
  prevRemoteLevel = prevRemotePeak = prevRemoteStatus = -1;
  prevLocalBarW = prevRemoteBarW = -1;
  prevRemoteAlarmActive = !remoteAlarmActive;
  prevLastReceivedMs = 0;
}

// draw gradient from green -> yellow -> red for width w (w in pixels)
void drawGradientRange(int x, int y, int w, int h) {
  if (w <= 0) return;
  if (w > BAR_W) w = BAR_W;
  for (int i = 0; i < w; ++i) {
    float pos = (float)i / (float)(BAR_W - 1);
    // interpolate green->yellow->red
    uint8_t r, g, b = 0;
    if (pos < 0.5f) {
      float t = pos / 0.5f; // 0..1
      r = (uint8_t)(t * 255.0f + 0.5f);
      g = 255;
    } else {
      float t = (pos - 0.5f) / 0.5f; // 0..1
      r = 255;
      g = (uint8_t)((1.0f - t) * 255.0f + 0.5f);
    }
    uint16_t c = rgb565(r, g, b);
    tft.drawFastVLine(x + i, y, h, c);
  }
}

// drawPanelDynamic: only redraw numeric, bar, status, remote alarm area when changed
void drawPanelDynamic(int topY, const char *title, int level, int peak, bool isLocal) {
  int numX = NUM_X_OFFSET;
  int numY = topY + 8;
  int barY = topY + BAR_Y_OFFSET;
  int innerBarY = barY + 1;
  int innerBarH = max(1, BAR_H - 2);
  int statusY = topY + STATUS_Y_OFFSET;

  // compute status code
  int statusCode = (level < 25) ? 0 : (level < 60 ? 1 : 2);

  // pick previous references
  int *pLevel = isLocal ? &prevLocalLevel : &prevRemoteLevel;
  int *pPeak  = isLocal ? &prevLocalPeak  : &prevRemotePeak;
  int *pStatus = isLocal ? &prevLocalStatus : &prevRemoteStatus;
  int *pBarW = isLocal ? &prevLocalBarW : &prevRemoteBarW;

  // -------- Numeric area update (right) --------
  if (!(alarmActive && !isLocal)) {
    if (level != *pLevel) {
      tft.fillRect(numX, numY - 2, DYN_NUM_W - 6, DYN_NUM_H, ILI9341_BLACK);
      tft.setTextSize(3);
      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(numX, numY);

      if (!isLocal && level < 0) {
        tft.print("--");
      } else {
        tft.print(level);
        tft.print("%");
      }
      *pLevel = level;
    }
  }

  // -------- Bar & peak area update --------
  int newW = levelToWidth(level);
  int prevW = (*pBarW >= 0) ? *pBarW : 0;

  bool linkChanged = false;
  if (!isLocal) {
    bool prevNoLink = (millis() - prevLastReceivedMs > NO_LINK_TIMEOUT);
    bool nowNoLink  = (millis() - lastReceivedMs > NO_LINK_TIMEOUT);
    linkChanged = (prevNoLink != nowNoLink);
  }

  if (linkChanged) {
    *pBarW = -1;
    *pStatus = -1;
    *pLevel = -1;
  }

  bool needBarRedraw = (newW != prevW) || (peak != *pPeak) || linkChanged || (*pBarW == -1) || (!isLocal && level < 0);
  
  if (needBarRedraw) {
    // If we're showing "(no link)" for remote, draw that; else draw gradient up to newW
    tft.fillRect(BAR_X, innerBarY, BAR_W, innerBarH, ILI9341_BLACK);

    if (!isLocal && level < 0) {
      // no link visual
      uint16_t darkNoLink = rgb565(100, 10, 10);
      tft.fillRect(BAR_X, innerBarY, BAR_W, innerBarH, darkNoLink);
      const char *nolink = "(no link)";
      uint8_t txtSize = 2;
      int txtW = textPixelWidth(nolink, txtSize);
      int txtX = BAR_X + (BAR_W - txtW) / 2;
      int txtY = innerBarY + (innerBarH - (8 * txtSize)) / 2;
      if (txtX < BAR_X) txtX = BAR_X + 2;
      tft.setTextSize(txtSize);
      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(txtX, txtY);
      tft.print(nolink);

      // numeric fallback
      tft.fillRect(numX, numY - 2, DYN_NUM_W - 6, DYN_NUM_H, ILI9341_BLACK);
      tft.setTextSize(3);
      tft.setCursor(numX, numY);
      tft.print("--");

      // clear status area when no link
      int statusClearW = PANEL_W - 8;
      if (!isLocal) statusClearW -= 90;   // reserve space for remote alarm indicator

      tft.fillRect(PANEL_MARGIN + 4, statusY, statusClearW, DYN_STATUS_H, ILI9341_BLACK);

      *pBarW = 0;
      *pPeak = peak;
    } else {
      // draw gradient up to newW
      if (newW > 0) {
        drawGradientRange(BAR_X, innerBarY, newW, innerBarH);
      }
      // peak shaded area (between barEnd and peak)
      int peakX = BAR_X + map(peak, 0, 100, 0, BAR_W);
      peakX = constrain(peakX, BAR_X, BAR_X + BAR_W);
      int barEndX = BAR_X + newW;
      uint16_t shadeRed = rgb565(140, 0, 0);
      if (peakX != barEndX) {
        int left = min(peakX, barEndX);
        int right = max(peakX, barEndX);
        if (left < BAR_X) left = BAR_X;
        if (right > BAR_X + BAR_W) right = BAR_X + BAR_W;
        if (right > left) tft.fillRect(left, innerBarY, right - left, innerBarH, shadeRed);
      }
      // thicker peak marker (3 px)
      int px = peakX;
      if (px - 1 < BAR_X) px = BAR_X + 1;
      if (px + 1 > BAR_X + BAR_W) px = BAR_X + BAR_W - 1;
      tft.fillRect(px - 1, innerBarY, 3, innerBarH, ILI9341_RED);

      *pBarW = newW;
      *pPeak = peak;
    }
    // store lastReceivedMs snapshot for remote link checks
    if (!isLocal) prevLastReceivedMs = lastReceivedMs;
  }

  // -------- Status area update (left) --------
  if (!(alarmActive && isLocal) && !( !isLocal && level < 0 )) {
    if (statusCode != *pStatus) {
      int statusClearW = PANEL_W - 8;

      if (!isLocal) {
        statusClearW -= 90;   // reserve space for remote alarm indicator
      }

      tft.fillRect(PANEL_MARGIN + 4, statusY, statusClearW, DYN_STATUS_H, ILI9341_BLACK);      tft.setCursor(PANEL_MARGIN + 8, statusY);
      tft.setTextSize(2);
      if (statusCode == 0) {
        tft.setTextColor(ILI9341_GREEN); tft.print("Status: QUIET");
      } else if (statusCode == 1) {
        tft.setTextColor(ILI9341_YELLOW); tft.print("Status: MODERATE");
      } else {
        tft.setTextColor(ILI9341_RED); tft.print("Status: LOUD");
      }
      *pStatus = statusCode;
    }
  }

  // -------- Remote-specific alarm indicator area (lower-right) --------
  if (!isLocal) {
    int rightAreaW = 90;
    int rightAreaX = PANEL_MARGIN + PANEL_W - rightAreaW;
    // if remote alarm state changed, or we haven't drawn this yet, update it
    if (remoteAlarmActive != prevRemoteAlarmActive || level != prevRemoteLevel || prevRemoteStatus == -1) {
      // clear area
      tft.fillRect(rightAreaX, statusY, rightAreaW, DYN_STATUS_H, ILI9341_BLACK);

      int iconRadius = 7;
      int cx = rightAreaX + rightAreaW - iconRadius - 6;
      int cy = statusY + (DYN_STATUS_H / 2);

      // choose label
      const char *label = remoteAlarmActive ? "REMOTE ALARM" : "Remote";
      // compute best text size to fit
      uint8_t selectedSize = 1;
      int selectedW = 0;
      int reservedForIcon = iconRadius * 2 + 12;
      int maxTxtW = rightAreaW - reservedForIcon - 6;
      if (maxTxtW < 0) maxTxtW = 0;
      const uint8_t trySizes[] = {3,2,1};
      for (uint8_t i=0;i<sizeof(trySizes)/sizeof(trySizes[0]);++i) {
        uint8_t s = trySizes[i];
        int wpx = textPixelWidth(label, s);
        int hpx = 8 * s;
        if (wpx <= maxTxtW && hpx <= DYN_STATUS_H) { selectedSize = s; selectedW = wpx; break; }
      }
      if (selectedW == 0) { selectedSize = 1; selectedW = textPixelWidth(label,1); }

      int labelX = rightAreaX + 6;
      if (labelX + selectedW > rightAreaX + rightAreaW - reservedForIcon) {
        labelX = rightAreaX + rightAreaW - reservedForIcon - selectedW;
        if (labelX < rightAreaX + 6) labelX = rightAreaX + 6;
      }
      int labelY = statusY + ((DYN_STATUS_H - (8 * selectedSize)) / 2);

      if (remoteAlarmActive) {
        tft.fillCircle(cx, cy, iconRadius, ILI9341_RED);
        uint8_t exSize = 2;
        if (8 * exSize > (iconRadius * 2)) exSize = 1;
        tft.setTextSize(exSize);
        tft.setTextColor(ILI9341_WHITE);
        int exW = textPixelWidth("!", exSize);
        int exH = 8 * exSize;
        int exX = cx - (exW / 2);
        int exY = cy - (exH / 2) - 1;
        tft.setCursor(exX, exY); tft.print("!");
        tft.setTextSize(selectedSize);
        tft.setCursor(labelX, labelY);
        tft.setTextColor(ILI9341_RED);
        tft.print(label);
      } else {
        tft.fillCircle(cx, cy, iconRadius - 1, rgb565(80, 80, 80));
        tft.setTextSize(selectedSize);
        tft.setCursor(labelX, labelY);
        tft.setTextColor(ILI9341_WHITE);
        tft.print(label);
      }
      prevRemoteAlarmActive = remoteAlarmActive;
    }
  }
}

// small centered alarm popup for local device
void drawAlarmPopup() {
  int boxW = 220;
  int boxH = 80;
  int boxX = (SCREEN_W - boxW) / 2;
  int boxY = (SCREEN_H - boxH) / 2;
  tft.fillRect(boxX, boxY, boxW, boxH, ILI9341_RED);
  tft.drawRect(boxX, boxY, boxW, boxH, ILI9341_WHITE);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setCursor(boxX + 30, boxY + 10); tft.print("WARNING");
  tft.setTextSize(2);
  tft.setCursor(boxX + 20, boxY + 45); tft.print("Noise too loud!");
  // remember that popup was shown so we can restore static UI after it clears
  alarmPopupShown = true;
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(esp_msg_t)) return;
  if (memcmp(info->src_addr, localMac, 6) == 0) return;
  memcpy(&incomingMsg, data, sizeof(incomingMsg));
  if (incomingMsg.noise > 100) return;
  if (incomingMsg.peak > 100) incomingMsg.peak = 100;

  lastRadioActivityMs = millis();

  remoteSmoothed = (REMOTE_ALPHA * (float)incomingMsg.noise) + ((1.0f - REMOTE_ALPHA) * remoteSmoothed);
  remoteDisplayLevel = (int)(remoteSmoothed + 0.5f);
  remotePeakLevel = incomingMsg.peak;
  remoteAlarmActive = incomingMsg.alarm;
  lastReceivedMs = millis();

  Serial.print("Recv from ");
  for (int i = 0; i < 6; ++i) {
    Serial.printf("%02X", info->src_addr[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" noise="); Serial.print(remoteDisplayLevel);
  Serial.print(" peak="); Serial.print(remotePeakLevel);
  Serial.print(" id="); Serial.println(incomingMsg.deviceId);
}