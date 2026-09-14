/*******************************************************************************
 * Event-Driven BMS Telemetry & Live Blynk IoT Dashboard
 * Platform: ESP32 + Blynk Cloud + I2C 16x2 LCD + 4-Cell Monitoring
 *******************************************************************************/

#define BLYNK_TEMPLATE_ID   "TMPL3UCSufABU"
#define BLYNK_TEMPLATE_NAME "Event Driven BMS Telemetry"
#define BLYNK_AUTH_TOKEN    "DfVVBHpdhQo0l8MBbPF9GC-AL4TJp0_y"

// Direct BLR1 regional cluster (prevents redirection timeouts in Wokwi)
#define BLYNK_SERVER        "blr1.blynk.cloud"
#define BLYNK_PORT          80

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// =============================================================================
// HARDWARE PIN DEFINITIONS (Matching your Wokwi diagram)
// =============================================================================
#define NUM_CELLS 4
const uint8_t PIN_CELLS[NUM_CELLS] = { 32, 33, 34, 35 };

LiquidCrystal_I2C lcd(0x27, 16, 2);

const uint8_t PIN_RELAY       = 18; // Safety Relay
const uint8_t PIN_FAULT_BTN   = 14; // Manual Emergency Cutoff Button (Active LOW)
const uint8_t PIN_NET_SWITCH  = 23; // Network Outage Switch (Active LOW)

const uint8_t PIN_LED_WIFI    = 2;  // Green: Wi-Fi Connected
const uint8_t PIN_LED_FAULT   = 4; // Red: Fault Tripped
const uint8_t PIN_LED_DRAIN   = 19; // Blue: Queue Playback Active

// =============================================================================
// NETWORK CREDENTIALS
// =============================================================================
char ssid[] = "Wokwi-GUEST";
char pass[] = "";

// =============================================================================
// SYSTEM THRESHOLDS & EVENT PARAMETERS
// =============================================================================
const float ADC_MAX          = 4095.0f;
const float V_MIN            = 2.50f;
const float V_MAX            = 4.50f;

const float WEAK_THRESH      = 2.80f; // Undervoltage cutoff (V)
const float OV_THRESH        = 4.15f; // Overvoltage cutoff (V)
const float IMBALANCE_THRESH = 0.30f; // Imbalance delta threshold (V)

const float VOLT_DEADBAND    = 0.05f; // 50 mV delta triggers transmission
const int   RSSI_DEADBAND    = 6;     // 6 dBm shift triggers transmission
const uint32_t MIN_TX_GAP_MS = 400;   // 400ms rate limiter between live events
const uint32_t HEARTBEAT_MS  = 45000; // 45s safety heartbeat
const uint32_t DRAIN_PACE_MS = 1200;  // 1.2s spacing between queued replays (avoids flood drops)

// =============================================================================
// BMS & HEALTH STRUCTURES
// =============================================================================
enum HealthState {
  HEALTHY,
  MINOR_IMBALANCE,
  CRITICAL_IMBALANCE,
  LOW_BATTERY,
  PACK_FAILURE,
  FAULT_STATE
};

struct CellData {
  float voltage;
  float soc;
};

struct PackData {
  float packVoltage;
  float avgVoltage;
  float avgSoc;
  float deltaV;
  float imbalancePct;
  int weakestCell;
  int strongestCell;
};

struct TelemetryFrame {
  uint32_t timestamp;
  float cellV[NUM_CELLS];
  float packVoltage;
  float avgSoc;
  float deltaV;
  int weakestCell;
  int strongestCell;
  bool relayEngaged;
  char faultCode[20];
  char healthState[20];
  int rssi;
  char triggerCause[24];
  bool isQueued;
};

CellData cells[NUM_CELLS];
PackData pack;
HealthState currentHealth = HEALTHY;
String currentFault = "NORMAL";
bool relayState = true;
int currentRssi = -100;
bool manualFaultLatch = false;

// Event Tracking
float lastTxV[NUM_CELLS] = { 0, 0, 0, 0 };
HealthState lastTxHealth = HEALTHY;
String lastTxFault = "";
bool lastTxRelay = true;
int lastTxWeakest = -1;
int lastTxStrongest = -1;
int lastTxRssi = 0;
uint32_t lastTxTime = 0;

// =============================================================================
// OFFLINE EVENT RING-BUFFER QUEUE
// =============================================================================
#define MAX_QUEUE_FRAMES 40
TelemetryFrame eventQueue[MAX_QUEUE_FRAMES];
size_t queueHead = 0;
size_t queueTail = 0;
size_t queueCount = 0;
uint32_t droppedEvents = 0;

bool isQueueEmpty() { return queueCount == 0; }
bool isQueueFull()  { return queueCount >= MAX_QUEUE_FRAMES; }

void pushEventQueue(const TelemetryFrame &frame) {
  if (isQueueFull()) {
    queueTail = (queueTail + 1) % MAX_QUEUE_FRAMES;
    queueCount--;
    droppedEvents++;
    Serial.printf("[QUEUE] Buffer full! Dropped oldest frame. (Total dropped: %u)\n", droppedEvents);
  }
  eventQueue[queueHead] = frame;
  queueHead = (queueHead + 1) % MAX_QUEUE_FRAMES;
  queueCount++;

  Serial.printf("[QUEUE] Offline buffered [#%u | Cause: %s | Q-Depth: %u/%d]\n",
                queueCount, frame.triggerCause, queueCount, MAX_QUEUE_FRAMES);
}

bool popEventQueue(TelemetryFrame &frame) {
  if (isQueueEmpty()) return false;
  frame = eventQueue[queueTail];
  queueTail = (queueTail + 1) % MAX_QUEUE_FRAMES;
  queueCount--;
  return true;
}

// =============================================================================
// NON-BLOCKING WI-FI STATE MACHINE
// =============================================================================
enum WiFiState {
  WIFI_ST_IDLE,
  WIFI_ST_CONNECTING,
  WIFI_ST_CONNECTED,
  WIFI_ST_WAIT_BACKOFF
};

WiFiState currentWiFiState = WIFI_ST_IDLE;
uint32_t wifiStateTimer = 0;
uint32_t backoffDelay = 3000;
bool isPlaybackActive = false;
uint32_t lastDrainTime = 0;

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================
float voltageToSOC(float v) {
  float soc = ((v - 2.80f) / (4.20f - 2.80f)) * 100.0f;
  if (soc < 0.0f) soc = 0.0f;
  if (soc > 100.0f) soc = 100.0f;
  return soc;
}

String getRSSIQuality(int rssi) {
  if (rssi > -60) return "Excellent";
  if (rssi > -70) return "Good";
  if (rssi > -80) return "Weak";
  return "Poor";
}

String healthLabel(HealthState st) {
  switch (st) {
    case HEALTHY:            return "Healthy";
    case MINOR_IMBALANCE:   return "Minor Imbal";
    case CRITICAL_IMBALANCE: return "Crit Imbal";
    case LOW_BATTERY:        return "Low Battery";
    case PACK_FAILURE:       return "Pack Failure";
    case FAULT_STATE:        return "Fault Trip";
    default:                 return "Unknown";
  }
}

// =============================================================================
// SENSOR SAMPLING & BMS PROTECTION
// =============================================================================
void sampleBatterySensors() {
  float sum = 0;
  float minV = 999.0f;
  float maxV = -999.0f;
  int weakIdx = 0;
  int strongIdx = 0;

  for (int i = 0; i < NUM_CELLS; i++) {
    int raw = analogRead(PIN_CELLS[i]);
    float voltage = V_MIN + ((float)raw / ADC_MAX) * (V_MAX - V_MIN);
    cells[i].voltage = voltage;
    cells[i].soc = voltageToSOC(voltage);
    sum += voltage;

    if (voltage < minV) {
      minV = voltage;
      weakIdx = i;
    }
    if (voltage > maxV) {
      maxV = voltage;
      strongIdx = i;
    }
  }

  pack.packVoltage   = sum;
  pack.avgVoltage    = sum / NUM_CELLS;
  pack.weakestCell   = weakIdx;
  pack.strongestCell = strongIdx;
  pack.deltaV        = maxV - minV;
  pack.imbalancePct  = (pack.deltaV / pack.avgVoltage) * 100.0f;

  float socSum = 0;
  for (int i = 0; i < NUM_CELLS; i++) socSum += cells[i].soc;
  pack.avgSoc = socSum / NUM_CELLS;

  if (WiFi.status() == WL_CONNECTED) {
    currentRssi = WiFi.RSSI();
  } else {
    currentRssi = -100;
  }
}

void evaluateProtectionLogic() {
  if (digitalRead(PIN_FAULT_BTN) == LOW) {
    manualFaultLatch = true;
  }

  String fault = "NORMAL";
  if (manualFaultLatch) {
    fault = "MANUAL_CUTOFF";
  } else if (cells[pack.strongestCell].voltage > OV_THRESH) {
    fault = "OVER_VOLTAGE";
  } else if (cells[pack.weakestCell].voltage < WEAK_THRESH) {
    fault = "UNDER_VOLTAGE";
  } else if (pack.deltaV > IMBALANCE_THRESH) {
    fault = "CELL_IMBALANCE";
  }
  currentFault = fault;

  if (fault != "NORMAL") {
    currentHealth = FAULT_STATE;
    relayState = false;
    digitalWrite(PIN_RELAY, LOW);
    digitalWrite(PIN_LED_FAULT, HIGH);
  } else {
    currentHealth = HEALTHY;
    relayState = true;
    digitalWrite(PIN_RELAY, HIGH);
    digitalWrite(PIN_LED_FAULT, LOW);
  }
}

// =============================================================================
// EVENT DETECTION LOGIC
// =============================================================================
bool detectTelemetryEvent(String &reason) {
  uint32_t now = millis();

  if (now - lastTxTime < MIN_TX_GAP_MS) return false;

  if (lastTxTime == 0) {
    reason = "BOOT_SYNC";
    return true;
  }

  if (currentFault != lastTxFault) {
    reason = String("FAULT_") + currentFault;
    return true;
  }

  if (relayState != lastTxRelay) {
    reason = relayState ? "RELAY_ENGAGED" : "RELAY_CUTOFF";
    return true;
  }

  for (int i = 0; i < NUM_CELLS; i++) {
    if (fabs(cells[i].voltage - lastTxV[i]) >= VOLT_DEADBAND) {
      reason = String("CELL") + (i + 1) + "_DELTA";
      return true;
    }
  }

  if (abs(currentRssi - lastTxRssi) >= RSSI_DEADBAND) {
    reason = "RSSI_CHANGE";
    return true;
  }

  if (now - lastTxTime >= HEARTBEAT_MS) {
    reason = "HEARTBEAT";
    return true;
  }

  return false;
}

// =============================================================================
// TELEMETRY TRANSMISSION & RATE-PACED DRAIN
// =============================================================================
void transmitFrameToBlynk(const TelemetryFrame &frame) {
  if (!Blynk.connected()) return;

  Blynk.virtualWrite(V0, frame.cellV[0]);
  Blynk.virtualWrite(V1, frame.cellV[1]);
  Blynk.virtualWrite(V2, frame.cellV[2]);
  Blynk.virtualWrite(V3, frame.cellV[3]);

  String sStrong = "Cell " + String(frame.strongestCell + 1) + " (" + String(frame.cellV[frame.strongestCell], 2) + "V)";
  String sWeak   = "Cell " + String(frame.weakestCell + 1) + " (" + String(frame.cellV[frame.weakestCell], 2) + "V)";
  Blynk.virtualWrite(V4, sStrong);
  Blynk.virtualWrite(V5, sWeak);
  Blynk.virtualWrite(V6, frame.deltaV);

  Blynk.virtualWrite(V7, frame.relayEngaged ? 1 : 0);
  Blynk.virtualWrite(V8, frame.faultCode);
  Blynk.virtualWrite(V9, frame.rssi);
  Blynk.virtualWrite(V10, (int)queueCount);
  Blynk.virtualWrite(V11, frame.isQueued ? "QUEUED REPLAY" : "LIVE");
  Blynk.virtualWrite(V12, frame.triggerCause);

  Blynk.run();

  Serial.printf("[BLYNK TX] %-14s | Cause: %-15s | C1:%.2f C2:%.2f C3:%.2f C4:%.2f | QDepth:%u\n",
                frame.isQueued ? "[QUEUED REPLAY]" : "[LIVE]",
                frame.triggerCause,
                frame.cellV[0], frame.cellV[1], frame.cellV[2], frame.cellV[3],
                queueCount);
}

void drainOfflineQueue() {
  uint32_t now = millis();
  if (now - lastDrainTime < DRAIN_PACE_MS) return;
  lastDrainTime = now;

  TelemetryFrame queuedFrame;
  if (popEventQueue(queuedFrame)) {
    isPlaybackActive = true;
    digitalWrite(PIN_LED_DRAIN, HIGH);
    queuedFrame.isQueued = true;
    transmitFrameToBlynk(queuedFrame);
  }
}

// =============================================================================
// WI-FI & BLYNK CONNECTION HANDLER
// =============================================================================
void handleWiFiStateMachine() {
  uint32_t now = millis();

  bool forceOutage = (digitalRead(PIN_NET_SWITCH) == LOW);
  if (forceOutage) {
    if (currentWiFiState != WIFI_ST_IDLE || WiFi.status() == WL_CONNECTED) {
      Serial.println("[NETWORK] >>> NETWORK OUTAGE SIMULATOR ACTIVE (GPIO 23 to GND) <<<");
      WiFi.disconnect(true);
      currentWiFiState = WIFI_ST_IDLE;
      digitalWrite(PIN_LED_WIFI, LOW);
      backoffDelay = 2000;
    }
    return;
  }

  static uint32_t lastConnectAttempt = 0;
  static uint32_t disconnectStartTime = 0;

  switch (currentWiFiState) {
    case WIFI_ST_IDLE:
      Serial.println("[NETWORK] Connecting to Wi-Fi...");
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, pass);
      wifiStateTimer = now;
      currentWiFiState = WIFI_ST_CONNECTING;
      break;

    case WIFI_ST_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        if (now - lastConnectAttempt >= 5000) {
          lastConnectAttempt = now;
          Serial.printf("[NETWORK] Wi-Fi OK! Connecting to Blynk [%s:%d]...\n", BLYNK_SERVER, BLYNK_PORT);
          
          if (Blynk.connect(6000)) {
            Serial.println("[BLYNK] >>> CONNECTED SUCCESSFULLY TO BLYNK CLOUD! <<<");
            digitalWrite(PIN_LED_WIFI, HIGH);
            currentWiFiState = WIFI_ST_CONNECTED;
            backoffDelay = 3000;
            disconnectStartTime = 0;
          } else {
            Serial.println("[BLYNK] Connection pending. Retrying shortly...");
          }
        }
      } else if (now - wifiStateTimer >= 15000) {
        Serial.println("[NETWORK] Wi-Fi timeout. Retrying...");
        WiFi.disconnect(true);
        currentWiFiState = WIFI_ST_WAIT_BACKOFF;
        wifiStateTimer = now;
      }
      break;

    case WIFI_ST_CONNECTED:
      if (WiFi.status() != WL_CONNECTED || !Blynk.connected()) {
        if (disconnectStartTime == 0) {
          disconnectStartTime = now;
        } else if (now - disconnectStartTime >= 3000) {
          Serial.println("[NETWORK] Connection lost confirmed. Entering backoff...");
          digitalWrite(PIN_LED_WIFI, LOW);
          currentWiFiState = WIFI_ST_WAIT_BACKOFF;
          wifiStateTimer = now;
          disconnectStartTime = 0;
        }
      } else {
        disconnectStartTime = 0;
        Blynk.run();
      }
      break;

    case WIFI_ST_WAIT_BACKOFF:
      if (now - wifiStateTimer >= backoffDelay) {
        Serial.println("[NETWORK] Backoff elapsed. Re-attempting connection...");
        backoffDelay = min(backoffDelay * 2, (uint32_t)15000);
        currentWiFiState = WIFI_ST_IDLE;
      }
      break;
  }
}

// =============================================================================
// BLYNK CALLBACKS
// =============================================================================
BLYNK_WRITE(V13) {
  if (param.asInt() == 1) {
    Serial.println("[BLYNK] Remote Trip Reset received.");
    manualFaultLatch = false;
    evaluateProtectionLogic();
  }
}

BLYNK_CONNECTED() {
  Serial.println("[BLYNK] Cloud synchronized.");
  lastTxTime = 0;
}

// =============================================================================
// LCD MULTI-PAGE SYSTEM
// =============================================================================
byte lcdPage = 0;
uint32_t lastLcdSwitch = 0;

void updateLcdDisplay() {
  uint32_t now = millis();
  if (now - lastLcdSwitch >= 2500) {
    lcdPage = (lcdPage + 1) % 5;
    lastLcdSwitch = now;
    lcd.clear();
  }

  char buf0[17];
  char buf1[17];

  switch (lcdPage) {
    case 0:
      snprintf(buf0, sizeof(buf0), "Pack:%.2fV %s", pack.packVoltage, relayState ? "ON " : "CUT");
      snprintf(buf1, sizeof(buf1), "H:%-6s F:%-5s", healthLabel(currentHealth).c_str(), currentFault.c_str());
      lcd.setCursor(0, 0);
      lcd.print(buf0);
      lcd.setCursor(0, 1);
      lcd.print(buf1);
      break;

    case 1:
      snprintf(buf0, sizeof(buf0), "C1:%.2f C2:%.2f", cells[0].voltage, cells[1].voltage);
      snprintf(buf1, sizeof(buf1), "C3:%.2f C4:%.2f", cells[2].voltage, cells[3].voltage);
      lcd.setCursor(0, 0);
      lcd.print(buf0);
      lcd.setCursor(0, 1);
      lcd.print(buf1);
      break;

    case 2:
      snprintf(buf0, sizeof(buf0), "SOC:%.0f%% Imb:%.0f%%", pack.avgSoc, pack.imbalancePct);
      snprintf(buf1, sizeof(buf1), "dV:%.2fV %s", pack.deltaV, pack.deltaV > IMBALANCE_THRESH ? "IMB!" : "OK ");
      lcd.setCursor(0, 0);
      lcd.print(buf0);
      lcd.setCursor(0, 1);
      lcd.print(buf1);
      break;

    case 3:
      snprintf(buf0, sizeof(buf0), "W:C%d S:C%d", pack.weakestCell + 1, pack.strongestCell + 1);
      snprintf(buf1, sizeof(buf1), "Min:%.2f Max:%.2f", cells[pack.weakestCell].voltage, cells[pack.strongestCell].voltage);
      lcd.setCursor(0, 0);
      lcd.print(buf0);
      lcd.setCursor(0, 1);
      lcd.print(buf1);
      break;

    case 4:
      if (WiFi.status() == WL_CONNECTED) {
        snprintf(buf0, sizeof(buf0), "WiFi:%ddBm %s", currentRssi, getRSSIQuality(currentRssi).c_str());
      } else {
        snprintf(buf0, sizeof(buf0), "WiFi: OFFLINE   ");
      }
      snprintf(buf1, sizeof(buf1), "QDepth:%u %s", queueCount, isPlaybackActive ? "DRAIN" : (queueCount > 0 ? "HOLD" : "IDLE"));
      lcd.setCursor(0, 0);
      lcd.print(buf0);
      lcd.setCursor(0, 1);
      lcd.print(buf1);
      break;
  }
}

// =============================================================================
// SETUP & MAIN LOOP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
 
  Serial.println("\n=== EVENT-DRIVEN BMS TELEMETRY & BLYNK DASHBOARD ===");

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, HIGH);

  pinMode(PIN_LED_WIFI, OUTPUT);
  pinMode(PIN_LED_FAULT, OUTPUT);
  pinMode(PIN_LED_DRAIN, OUTPUT);
  digitalWrite(PIN_LED_WIFI, LOW);
  digitalWrite(PIN_LED_FAULT, LOW);
  digitalWrite(PIN_LED_DRAIN, LOW);

  pinMode(PIN_FAULT_BTN, INPUT_PULLUP);
  pinMode(PIN_NET_SWITCH, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("BMS Telemetry");
  lcd.setCursor(0, 1);
  lcd.print("Event-Driven");

  // Direct regional configuration
  Blynk.config(BLYNK_AUTH_TOKEN, BLYNK_SERVER, BLYNK_PORT);

  sampleBatterySensors();
  evaluateProtectionLogic();
}

void loop() {
  // 1. Maintain Wi-Fi and Blynk
  handleWiFiStateMachine();

  // 2. Continuous 10Hz Battery Sampling & Protection
  static uint32_t lastSampleTime = 0;
  if (millis() - lastSampleTime >= 100) {
    lastSampleTime = millis();
    sampleBatterySensors();
    evaluateProtectionLogic();
  }

  // 3. Event-Driven Telemetry Trigger
  String triggerReason = "";
  if (detectTelemetryEvent(triggerReason)) {
    TelemetryFrame frame;
    frame.timestamp = millis();
    for (int i = 0; i < NUM_CELLS; i++) frame.cellV[i] = cells[i].voltage;
    frame.packVoltage = pack.packVoltage;
    frame.avgSoc = pack.avgSoc;
    frame.deltaV = pack.deltaV;
    frame.weakestCell = pack.weakestCell;
    frame.strongestCell = pack.strongestCell;
    frame.relayEngaged = relayState;
    strncpy(frame.faultCode, currentFault.c_str(), sizeof(frame.faultCode) - 1);
    frame.faultCode[sizeof(frame.faultCode) - 1] = '\0';
    strncpy(frame.healthState, healthLabel(currentHealth).c_str(), sizeof(frame.healthState) - 1);
    frame.healthState[sizeof(frame.healthState) - 1] = '\0';
    frame.rssi = currentRssi;
    strncpy(frame.triggerCause, triggerReason.c_str(), sizeof(frame.triggerCause) - 1);
    frame.triggerCause[sizeof(frame.triggerCause) - 1] = '\0';
    frame.isQueued = false;

    if (currentWiFiState == WIFI_ST_CONNECTED && Blynk.connected() && queueCount == 0 && !isPlaybackActive) {
      transmitFrameToBlynk(frame);
    } else {
      frame.isQueued = true;
      pushEventQueue(frame);
    }

    for (int i = 0; i < NUM_CELLS; i++) lastTxV[i] = cells[i].voltage;
    lastTxHealth    = currentHealth;
    lastTxFault     = currentFault;
    lastTxRelay     = relayState;
    lastTxWeakest   = pack.weakestCell;
    lastTxStrongest = pack.strongestCell;
    lastTxRssi      = currentRssi;
    lastTxTime      = millis();
  }

  // 4. Rate-Paced Offline Queue Draining
  if (currentWiFiState == WIFI_ST_CONNECTED && Blynk.connected() && queueCount > 0) {
    drainOfflineQueue();
  } else if (queueCount == 0 && isPlaybackActive) {
    isPlaybackActive = false;
    digitalWrite(PIN_LED_DRAIN, LOW);
    Serial.println("[PLAYBACK] Offline queue drained. Reverted to LIVE streaming.");
    Blynk.virtualWrite(V11, "LIVE");
    Blynk.virtualWrite(V10, 0);
  }

  // 5. Update Diagnostic LCD Display
  updateLcdDisplay();
}
