/**********************************************************************************
 * TASK 6: ENTERPRISE BATTERY INTELLIGENCE & DECISION DASHBOARD
 * Architecture: ESP32 + Blynk IoT + I2C LCD + Battery Analytics Engine
 **********************************************************************************/


   #define BLYNK_TEMPLATE_ID "TMPL3-OeKA5v6"
   #define BLYNK_TEMPLATE_NAME "task 6 battery IOT"
   #define BLYNK_AUTH_TOKEN "3VS9oPysSPCKOC-ssJLW8lJ4RYWVyJEr"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

char ssid[] = "Wokwi-GUEST";
char pass[] = "";

LiquidCrystal_I2C lcd(0x27, 16, 2);
WidgetTerminal terminal(V9);

#define GREEN_LED   25
#define YELLOW_LED  26
#define RED_LED     27

#define NUM_CELLS   4
const int ADC_PINS[NUM_CELLS] = { 34, 35, 32, 33 };

#define ADC_RESOLUTION 4095.0f
#define V_MIN          2.50f
#define V_MAX          4.20f
#define WEAK_THRESH    2.80f
#define OV_THRESH      4.15f
#define BAL_WARN_PCT   3.00f
#define BAL_CRIT_PCT   8.00f

enum OperatingState {
  STATE_HEALTHY = 0,
  STATE_MINOR_IMBALANCE,
  STATE_CRITICAL_IMBALANCE,
  STATE_LOW_BATTERY,
  STATE_PACK_FAILURE,
  STATE_FAULT_OVERVOLTAGE,
  STATE_FAULT_UNDERVOLTAGE
};

OperatingState currentState  = STATE_HEALTHY;
OperatingState previousState = STATE_HEALTHY;

struct CellData {
  float voltage;
  float soc;
  float deviation;
};

struct PackAnalytics {
  float packVoltage;
  float avgVoltage;
  float avgSoc;
  float imbalancePct;
  float deltaMilliVolts;
  int   weakestCell;
  int   strongestCell;
  int   compositeRiskScore;
  String riskLevel;
  String recommendation;
};

CellData cells[NUM_CELLS];
PackAnalytics pack;

unsigned long systemStartTime  = 0;
unsigned int  totalFaultCount  = 0;
float         previousVoltages[NUM_CELLS];
int           prevWeakest      = -1;
int           prevStrongest    = -1;

#define POLL_INTERVAL_MS       200
#define TELEMETRY_INTERVAL_MS  1000
#define LCD_PAGE_INTERVAL_MS   3000
#define WIFI_RETRY_INTERVAL_MS 10000

unsigned long lastPollTime      = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastLCDTime       = 0;
unsigned long lastWiFiRetry     = 0;

byte lcdPage = 0;
bool lastWiFiConnected  = false;
bool lastBlynkConnected = false;

#define MAX_EVENTS 35
struct EventRecord {
  String timestamp;
  String detail;
};

EventRecord eventQueue[MAX_EVENTS];
int queueHead = 0;
int queueTail = 0;

bool isQueueEmpty() { return queueHead == queueTail; }
bool isQueueFull()  { return ((queueTail + 1) % MAX_EVENTS) == queueHead; }

String getFormattedUptime() {
  unsigned long sec = millis() / 1000;
  unsigned long d   = sec / 86400;
  unsigned long h   = (sec % 86400) / 3600;
  unsigned long m   = (sec % 3600) / 60;
  unsigned long s   = sec % 60;

  char buf[24];
  if (d > 0) {
    snprintf(buf, sizeof(buf), "%lud %02lu:%02lu:%02lu", d, h, m, s);
  } else {
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", h, m, s);
  }
  return String(buf);
}

void enqueueEvent(String detail) {
  if (isQueueFull()) return;
  eventQueue[queueTail].timestamp = getFormattedUptime();
  eventQueue[queueTail].detail    = detail;
  queueTail = (queueTail + 1) % MAX_EVENTS;
  Serial.println("[" + getFormattedUptime() + "] " + detail);
}

EventRecord dequeueEvent() {
  EventRecord record = { "", "" };
  if (isQueueEmpty()) return record;
  record = eventQueue[queueHead];
  queueHead = (queueHead + 1) % MAX_EVENTS;
  return record;
}

float voltageToSOC(float v) {
  float soc = ((v - V_MIN) / (V_MAX - V_MIN)) * 100.0f;
  return constrain(soc, 0.0f, 100.0f);
}

String getStateLabel(OperatingState state) {
  switch (state) {
    case STATE_HEALTHY:             return "NORMAL";
    case STATE_MINOR_IMBALANCE:     return "IMBALANCE WARN";
    case STATE_CRITICAL_IMBALANCE:  return "CRIT IMBALANCE";
    case STATE_LOW_BATTERY:         return "LOW SOC";
    case STATE_PACK_FAILURE:        return "PACK FAILURE";
    case STATE_FAULT_OVERVOLTAGE:   return "TRIP: OVER-VOLT";
    case STATE_FAULT_UNDERVOLTAGE:  return "TRIP: UNDER-VOLT";
    default:                        return "UNKNOWN";
  }
}

void calculateCompositeRisk() {
  int risk = 0;

  if (pack.deltaMilliVolts > 200.0f) {
    risk += 35;
  } else if (pack.deltaMilliVolts > 100.0f) {
    risk += map((int)pack.deltaMilliVolts, 100, 200, 15, 34);
  } else if (pack.deltaMilliVolts > 50.0f) {
    risk += map((int)pack.deltaMilliVolts, 50, 100, 5, 14);
  }

  if (pack.avgSoc <= 10.0f) {
    risk += 25;
  } else if (pack.avgSoc <= 20.0f) {
    risk += 15;
  } else if (pack.avgSoc >= 98.0f) {
    risk += 10;
  }

  for (int i = 0; i < NUM_CELLS; i++) {
    if (cells[i].voltage < WEAK_THRESH || cells[i].voltage > OV_THRESH) {
      risk += 30;
      break;
    }
  }

  risk += constrain(totalFaultCount * 2, 0, 10);
  pack.compositeRiskScore = constrain(risk, 0, 100);

  if (pack.compositeRiskScore < 25) {
    pack.riskLevel = "LOW";
  } else if (pack.compositeRiskScore < 55) {
    pack.riskLevel = "MODERATE";
  } else if (pack.compositeRiskScore < 80) {
    pack.riskLevel = "ELEVATED";
  } else {
    pack.riskLevel = "CRITICAL";
  }
}

void generateRecommendation() {
  switch (currentState) {
    case STATE_FAULT_OVERVOLTAGE:
      pack.recommendation = "ISOLATE CHARGER: C" + String(pack.strongestCell + 1) +
                            " exceeding " + String(OV_THRESH, 2) + "V threshold!";
      break;

    case STATE_FAULT_UNDERVOLTAGE:
      pack.recommendation = "DISCONNECT LOAD: C" + String(pack.weakestCell + 1) +
                            " below cut-off (" + String(cells[pack.weakestCell].voltage, 2) + "V)!";
      break;

    case STATE_PACK_FAILURE:
      pack.recommendation = "EMERGENCY SHUTDOWN: Complete discharge. Lockout active.";
      break;

    case STATE_CRITICAL_IMBALANCE:
      pack.recommendation = "ACTIVE BALANCING REQ: Delta=" + String(pack.deltaMilliVolts, 0) +
                            "mV. Service pack immediately.";
      break;

    case STATE_MINOR_IMBALANCE:
      pack.recommendation = "SCHEDULE TOP BALANCE: Bleed charge on C" + String(pack.strongestCell + 1);
      break;

    case STATE_LOW_BATTERY:
      pack.recommendation = "CHARGE RECOMMENDED: Plug in 0.5C constant-current charger.";
      break;

    case STATE_HEALTHY:
    default:
      if (pack.compositeRiskScore < 20) {
        pack.recommendation = "SYSTEM OPTIMAL: All cells in equilibrium. No action.";
      } else {
        pack.recommendation = "MONITORING: Nominal operation with mild drift stress.";
      }
      break;
  }
}

void readBatterySensors() {
  float sumVolts = 0.0f;
  float minV     = 99.0f;
  float maxV     = -99.0f;
  int minIdx     = 0;
  int maxIdx     = 0;

  for (int i = 0; i < NUM_CELLS; i++) {
    int rawADC = analogRead(ADC_PINS[i]);
    float v = V_MIN + ((float)rawADC / ADC_RESOLUTION) * (V_MAX - V_MIN);
    cells[i].voltage = v;
    cells[i].soc     = voltageToSOC(v);

    sumVolts += v;
    if (v < minV) { minV = v; minIdx = i; }
    if (v > maxV) { maxV = v; maxIdx = i; }
  }

  pack.packVoltage     = sumVolts;
  pack.avgVoltage      = sumVolts / NUM_CELLS;
  pack.weakestCell     = minIdx;
  pack.strongestCell   = maxIdx;
  pack.deltaMilliVolts = (maxV - minV) * 1000.0f;
  pack.imbalancePct    = ((maxV - minV) / pack.avgVoltage) * 100.0f;

  float totalSoc = 0.0f;
  for (int i = 0; i < NUM_CELLS; i++) {
    cells[i].deviation = fabs(cells[i].voltage - pack.avgVoltage);
    totalSoc += cells[i].soc;
  }
  pack.avgSoc = totalSoc / NUM_CELLS;

  calculateCompositeRisk();
}

OperatingState evaluateOperatingState() {
  for (int i = 0; i < NUM_CELLS; i++) {
    if (cells[i].voltage > OV_THRESH)   return STATE_FAULT_OVERVOLTAGE;
    if (cells[i].voltage < WEAK_THRESH) return STATE_FAULT_UNDERVOLTAGE;
  }

  if (pack.avgSoc <= 5.0f)               return STATE_PACK_FAILURE;
  if (pack.avgSoc <= 20.0f)              return STATE_LOW_BATTERY;
  if (pack.imbalancePct >= BAL_CRIT_PCT) return STATE_CRITICAL_IMBALANCE;
  if (pack.imbalancePct >= BAL_WARN_PCT) return STATE_MINOR_IMBALANCE;

  return STATE_HEALTHY;
}

void processStateTransitions() {
  currentState = evaluateOperatingState();
  generateRecommendation();

  if (currentState != previousState) {
    if (currentState >= STATE_CRITICAL_IMBALANCE) {
      totalFaultCount++;
    }

    String logEntry = "STATE: " + getStateLabel(previousState) + " -> " +
                      getStateLabel(currentState) + " (Risk: " +
                      String(pack.compositeRiskScore) + "%)";
    enqueueEvent(logEntry);
    previousState = currentState;
  }

  if (pack.weakestCell != prevWeakest && prevWeakest != -1) {
    enqueueEvent("WEAKEST HANDOFF -> Cell " + String(pack.weakestCell + 1) +
                 " (" + String(cells[pack.weakestCell].voltage, 2) + "V)");
  }
  prevWeakest = pack.weakestCell;

  if (pack.strongestCell != prevStrongest && prevStrongest != -1) {
    enqueueEvent("STRONGEST HANDOFF -> Cell " + String(pack.strongestCell + 1) +
                 " (" + String(cells[pack.strongestCell].voltage, 2) + "V)");
  }
  prevStrongest = pack.strongestCell;
}

void updateSeverityIndicators() {
  int greenVal  = 0;
  int yellowVal = 0;
  int redVal    = 0;

  switch (currentState) {
    case STATE_HEALTHY:
      greenVal = HIGH;
      break;

    case STATE_MINOR_IMBALANCE:
    case STATE_LOW_BATTERY:
      yellowVal = HIGH;
      break;

    case STATE_CRITICAL_IMBALANCE:
    case STATE_PACK_FAILURE:
    case STATE_FAULT_OVERVOLTAGE:
    case STATE_FAULT_UNDERVOLTAGE:
      redVal = HIGH;
      break;
  }

  digitalWrite(GREEN_LED,  greenVal);
  digitalWrite(YELLOW_LED, yellowVal);
  digitalWrite(RED_LED,    redVal);

  if (Blynk.connected()) {
    Blynk.virtualWrite(V14, greenVal  ? 255 : 0);
    Blynk.virtualWrite(V15, yellowVal ? 255 : 0);
    Blynk.virtualWrite(V16, redVal    ? 255 : 0);
  }
}

void sendBlynkTelemetry() {
  if (!Blynk.connected()) return;

  Blynk.virtualWrite(V0, cells[0].voltage);
  Blynk.virtualWrite(V1, cells[1].voltage);
  Blynk.virtualWrite(V2, cells[2].voltage);
  Blynk.virtualWrite(V3, cells[3].voltage);

  Blynk.virtualWrite(V4,  pack.packVoltage);
  Blynk.virtualWrite(V5,  (int)pack.avgSoc);
  Blynk.virtualWrite(V6,  pack.imbalancePct);
  Blynk.virtualWrite(V20, pack.deltaMilliVolts);

  Blynk.virtualWrite(V7,  getStateLabel(currentState));
  Blynk.virtualWrite(V8,  WiFi.RSSI());
  Blynk.virtualWrite(V10, pack.riskLevel);
  Blynk.virtualWrite(V17, pack.compositeRiskScore);
  Blynk.virtualWrite(V18, getFormattedUptime());
  Blynk.virtualWrite(V19, totalFaultCount);

  Blynk.virtualWrite(V11, pack.recommendation);
  Blynk.virtualWrite(V12, "C" + String(pack.weakestCell + 1) + " (" +
                          String(cells[pack.weakestCell].voltage, 2) + "V)");
  Blynk.virtualWrite(V13, "C" + String(pack.strongestCell + 1) + " (" +
                          String(cells[pack.strongestCell].voltage, 2) + "V)");

  int flushLimit = 0;
  while (!isQueueEmpty() && flushLimit < 2) {
    EventRecord ev = dequeueEvent();
    terminal.println("[" + ev.timestamp + "] " + ev.detail);
    flushLimit++;
  }
  terminal.flush();
}

void updateLocalDisplay() {
  unsigned long now = millis();
  if (now - lastLCDTime < LCD_PAGE_INTERVAL_MS) return;
  lastLCDTime = now;

  lcdPage = (lcdPage + 1) % 6;
  lcd.clear();

  switch (lcdPage) {
    case 0:
      lcd.setCursor(0, 0); lcd.print("PACK:"); lcd.print(pack.packVoltage, 2); lcd.print("V");
      lcd.setCursor(0, 1); lcd.print("SOC:");  lcd.print(pack.avgSoc, 0); lcd.print("% ");
      lcd.print(getStateLabel(currentState).substring(0, 7));
      break;

    case 1:
      lcd.setCursor(0, 0);
      lcd.print("1:"); lcd.print(cells[0].voltage, 2);
      lcd.print(" 2:"); lcd.print(cells[1].voltage, 2);
      lcd.setCursor(0, 1);
      lcd.print("3:"); lcd.print(cells[2].voltage, 2);
      lcd.print(" 4:"); lcd.print(cells[3].voltage, 2);
      break;

    case 2:
      lcd.setCursor(0, 0); lcd.print("RISK: "); lcd.print(pack.compositeRiskScore); lcd.print("/100");
      lcd.setCursor(0, 1); lcd.print("CLASS: "); lcd.print(pack.riskLevel);
      break;

    case 3:
      lcd.setCursor(0, 0); lcd.print("DELTA: "); lcd.print((int)pack.deltaMilliVolts); lcd.print("mV");
      lcd.setCursor(0, 1); lcd.print("W:C"); lcd.print(pack.weakestCell + 1);
      lcd.print("  S:C"); lcd.print(pack.strongestCell + 1);
      break;

    case 4:
      lcd.setCursor(0, 0); lcd.print("UPTIME: "); lcd.print(getFormattedUptime().substring(0, 8));
      lcd.setCursor(0, 1); lcd.print("FAULTS: "); lcd.print(totalFaultCount);
      lcd.print(" RSSI:"); lcd.print(WiFi.RSSI());
      break;

    case 5:
      lcd.setCursor(0, 0); lcd.print("ACTION ADVICE:");
      lcd.setCursor(0, 1);
      lcd.print(pack.recommendation.substring(0, 16));
      break;
  }
}

BLYNK_CONNECTED() {
  terminal.println("\n==========================================");
  terminal.println("  ENTERPRISE BMS GATEWAY ONLINE");
  terminal.println("  Node ID   : ESP32-BMS-01");
  terminal.println("  Auth Token: VERIFIED");
  terminal.println("==========================================");
  terminal.flush();

  enqueueEvent("Blynk IoT Cloud Sync Established");
  lastBlynkConnected = true;
}

void handleConnectivity() {
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long now = millis();
    if (now - lastWiFiRetry > WIFI_RETRY_INTERVAL_MS) {
      lastWiFiRetry = now;
      Serial.println("[WiFi] Reconnection attempt...");
      WiFi.begin(ssid, pass);
    }
    return;
  }

  if (!Blynk.connected()) {
    Blynk.connect(5000);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(GREEN_LED,  OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED,    OUTPUT);

  for (int i = 0; i < NUM_CELLS; i++) {
    pinMode(ADC_PINS[i], INPUT);
    previousVoltages[i] = V_MAX;
  }

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Enterprise BMS");
  lcd.setCursor(0, 1); lcd.print("Booting Core...");

  WiFi.begin(ssid, pass);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 25) {
    delay(400);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Online. IP: " + WiFi.localIP().toString());
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("WiFi Connected!");
    lcd.setCursor(0, 1); lcd.print(WiFi.localIP().toString());
    delay(1000);
  }

  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(8000);

  enqueueEvent("BMS Supervisor Core Started");
  systemStartTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  if (Blynk.connected()) {
    Blynk.run();
  }
  handleConnectivity();

  if (currentMillis - lastPollTime >= POLL_INTERVAL_MS) {
    lastPollTime = currentMillis;
    readBatterySensors();
    processStateTransitions();
    updateSeverityIndicators();
  }

  if (currentMillis - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = currentMillis;
    sendBlynkTelemetry();
  }

  updateLocalDisplay();
}
