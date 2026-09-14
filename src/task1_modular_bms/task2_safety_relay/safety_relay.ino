// ════════════════════════════════════════════════════════════════════════
//  Non-Blocking Protection Relay and Safety System
//  Architecture : Fully non-blocking, millis()-based, 4-tier state machine
//  Filtering    : Simple Moving Average (SMA) window to reject noise
//  Detection    : Frozen sensors, out-of-range, unrealistic jumps, dV/dt
//  Protection   : Hysteresis, debounce timing, and relay anti-chatter
// ════════════════════════════════════════════════════════════════════════

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ── Pin Configuration ──────────────────────────────────────
#define NUM_CELLS    4
const int ADC_PINS[NUM_CELLS] = {34, 35, 32, 33};
#define RELAY_PIN    27
#define BUZZER_PIN   26

// ── Voltage Limits & Hysteresis ────────────────────────────
#define ADC_MAX             4095.0f
#define V_REF               3.3f
#define V_MIN               2.5f  // Scaling minimum
#define V_MAX               4.2f  // Scaling maximum

// Hysteresis allows the voltage to dip to 2.8V to trigger a fault, 
// but it must rise above 2.95V to be considered "recovered" (anti-bounce)
#define WEAK_THRESH_FAULT   2.80f 
#define WEAK_THRESH_RECOVER 2.95f
#define OV_THRESH_FAULT     4.15f
#define OV_THRESH_RECOVER   4.05f

// ── Anomaly & Statistical Filter Tuning ────────────────────
#define WINDOW_SIZE         5      // Moving average window size
#define NOISE_JUMP_THRESH   0.80f  // Instant difference between raw and SMA
#define DVDT_THRESH         0.20f  // Genuine rapid load change (V/sec)
#define FROZEN_TOLERANCE    2      // ADC counts
#define FROZEN_WINDOW       50     // Consecutive identical reads to flag frozen

// ── Timing Constants (millis) ──────────────────────────────
#define POLL_INTERVAL       100    // Sensor read rate
#define DEBOUNCE_TIME       400    // Time fault must persist before tripping
#define RECOVERY_TIME       4000   // Time system must be stable before restoring
#define RELAY_COOLDOWN      2500   // Hardwear anti-chatter limit
#define BUZZER_TOGGLE       200
#define LCD_REFRESH         500

// ── System States ──────────────────────────────────────────
enum SystemState { STATE_NORMAL, STATE_DEBOUNCE, STATE_FAULT, STATE_RECOVERY };
SystemState currentState = STATE_NORMAL;

enum FaultType { NONE, WEAK_CELL, OVERVOLTAGE, SENSOR_FROZEN, SENSOR_JUMP, RAPID_DROP };
FaultType activeFault = NONE;
int       faultCell   = -1;

// ── Sensor Tracking Data ───────────────────────────────────
struct CellData {
  float history[WINDOW_SIZE];
  int   head;
  float smaVoltage;       // Smoothed moving average
  float prevSmaVoltage;   // For dV/dt over time
  int   stuckCount;
  int   lastRawADC;
};
CellData cells[NUM_CELLS];

// ── Timers ─────────────────────────────────────────────────
unsigned long tLastPoll        = 0;
unsigned long tStateChange     = 0;
unsigned long tLastRelay       = 0;
unsigned long tLastBuzzer      = 0;
unsigned long tLastLCD         = 0;
unsigned long tLastDvDt        = 0;

bool relayState  = true;
bool buzzerState = false;

// ── Initialization ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(100);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  for (int i = 0; i < NUM_CELLS; i++) {
    pinMode(ADC_PINS[i], INPUT);
    cells[i].head = 0;
    cells[i].stuckCount = 0;
    cells[i].lastRawADC = -1;
    
    // Pre-fill window to prevent startup artifacts
    int raw = analogRead(ADC_PINS[i]);
    float v = V_MIN + (raw / ADC_MAX) * (V_MAX - V_MIN);
    for(int w=0; w<WINDOW_SIZE; w++) cells[i].history[w] = v;
    cells[i].smaVoltage = v;
    cells[i].prevSmaVoltage = v;
  }

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);
  digitalWrite(BUZZER_PIN, LOW);

  lcd.init();
  lcd.backlight();
  lcd.print("Safety Kernel V2");
  Serial.println(F("[SYSTEM] Online. State: NORMAL"));
  tStateChange = millis();
}

// ── Core Sensor Logic ──────────────────────────────────────
bool checkFaultConditions(bool useRecoveryHysteresis, unsigned long now) {
  bool anyFaultDetected = false;
  activeFault = NONE;
  faultCell = -1;
  
  float limitWeak = useRecoveryHysteresis ? WEAK_THRESH_RECOVER : WEAK_THRESH_FAULT;
  float limitOver = useRecoveryHysteresis ? OV_THRESH_RECOVER : OV_THRESH_FAULT;

  bool evaluateDvDt = (now - tLastDvDt >= 1000); // Check genuine change once per second

  for (int i = 0; i < NUM_CELLS; i++) {
    int raw = analogRead(ADC_PINS[i]);
    float rawVolts = V_MIN + (raw / ADC_MAX) * (V_MAX - V_MIN);

    // 1. Update Statistical Window (Moving Average)
    cells[i].history[cells[i].head] = rawVolts;
    cells[i].head = (cells[i].head + 1) % WINDOW_SIZE;
    
    float sum = 0;
    for(int w=0; w<WINDOW_SIZE; w++) sum += cells[i].history[w];
    cells[i].smaVoltage = sum / WINDOW_SIZE;

    // 2. Anomaly: Unrealistic Jump (Noise rejection)
    // If raw jumps massively from the smoothed average, it's flagged as an anomaly.
    // Genuine changes will pull the SMA down gradually and trigger standard limits.
    if (abs(rawVolts - cells[i].smaVoltage) > NOISE_JUMP_THRESH) {
      anyFaultDetected = true; activeFault = SENSOR_JUMP; faultCell = i;
    }

    // 3. Anomaly: Frozen Sensor
    if (cells[i].lastRawADC != -1) {
      if (abs(raw - cells[i].lastRawADC) <= FROZEN_TOLERANCE) cells[i].stuckCount++;
      else cells[i].stuckCount = 0;
      
      if (cells[i].stuckCount >= FROZEN_WINDOW) {
        anyFaultDetected = true; activeFault = SENSOR_FROZEN; faultCell = i;
      }
    }
    cells[i].lastRawADC = raw;

    // 4. Out-of-Range (Applied on SMOOTHED data, not raw, to prevent noise trips)
    if (cells[i].smaVoltage < limitWeak) {
      anyFaultDetected = true; activeFault = WEAK_CELL; faultCell = i;
    } 
    else if (cells[i].smaVoltage > limitOver) {
      anyFaultDetected = true; activeFault = OVERVOLTAGE; faultCell = i;
    }

    // 5. Genuine Rapid Load Change (dV/dt)
    if (evaluateDvDt) {
      float dvdt = abs(cells[i].smaVoltage - cells[i].prevSmaVoltage);
      if (dvdt > DVDT_THRESH) {
         anyFaultDetected = true; activeFault = RAPID_DROP; faultCell = i;
      }
      cells[i].prevSmaVoltage = cells[i].smaVoltage;
    }
  }
  
  if (evaluateDvDt) tLastDvDt = now;
  return anyFaultDetected;
}

// ── Hardware Drivers ───────────────────────────────────────
void setRelay(bool closeRelay, unsigned long now) {
  if (closeRelay == relayState) return; 
  if (now - tLastRelay < RELAY_COOLDOWN) return; // Hard anti-chatter

  relayState = closeRelay;
  digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
  tLastRelay = now;
  
  Serial.print(F("[HARDWARE] Relay switched "));
  Serial.println(relayState ? "ON" : "OFF (TRIPPED)");
}

void driveBuzzer(unsigned long now) {
  if (currentState == STATE_NORMAL || currentState == STATE_DEBOUNCE) {
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }
  
  // Fast beep for active fault, slow beep for recovery
  unsigned long interval = (currentState == STATE_FAULT) ? BUZZER_TOGGLE : BUZZER_TOGGLE * 3;
  
  if (now - tLastBuzzer >= interval) {
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
    tLastBuzzer = now;
  }
}

const char* getFaultName() {
  switch(activeFault) {
    case WEAK_CELL: return "WEAK CELL";
    case OVERVOLTAGE: return "OVERVOLT";
    case SENSOR_FROZEN: return "FROZEN SNS";
    case SENSOR_JUMP: return "GLITCH/JUMP";
    case RAPID_DROP: return "HIGH dV/dt";
    default: return "UNKNOWN";
  }
}

// ── Main State Machine ─────────────────────────────────────
void processStateMachine(unsigned long now) {
  // Use hysteresis thresholds only if we are currently trying to clear a fault
  bool isClearing = (currentState == STATE_FAULT || currentState == STATE_RECOVERY);
  bool conditionPresent = checkFaultConditions(isClearing, now);

  switch (currentState) {
    
    case STATE_NORMAL:
      if (conditionPresent) {
        currentState = STATE_DEBOUNCE;
        tStateChange = now;
        Serial.print(F("[TRANSITION] NORMAL -> DEBOUNCE. Suspect: "));
        Serial.println(getFaultName());
      }
      break;

    case STATE_DEBOUNCE:
      if (!conditionPresent) {
        currentState = STATE_NORMAL; // Was just noise
        Serial.println(F("[TRANSITION] DEBOUNCE -> NORMAL (Noise rejected)"));
      } 
      else if (now - tStateChange >= DEBOUNCE_TIME) {
        currentState = STATE_FAULT;
        tStateChange = now;
        setRelay(false, now);
        Serial.print(F("[TRANSITION] DEBOUNCE -> FAULT CONFIRMED: "));
        Serial.println(getFaultName());
      }
      break;

    case STATE_FAULT:
      if (!conditionPresent) { // Cleared (Hysteresis bounds applied)
        currentState = STATE_RECOVERY;
        tStateChange = now;
        Serial.println(F("[TRANSITION] FAULT -> RECOVERY (Beginning timed recovery)"));
      }
      break;

    case STATE_RECOVERY:
      if (conditionPresent) {
        currentState = STATE_FAULT;
        tStateChange = now;
        Serial.println(F("[TRANSITION] RECOVERY -> FAULT (Condition returned)"));
      } 
      else if (now - tStateChange >= RECOVERY_TIME) {
        currentState = STATE_NORMAL;
        tStateChange = now;
        setRelay(true, now);
        Serial.println(F("[TRANSITION] RECOVERY -> NORMAL (System Restored)"));
      }
      break;
  }
}

// ── Display Management ─────────────────────────────────────
void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);

  if (currentState == STATE_NORMAL || currentState == STATE_DEBOUNCE) {
    lcd.print("SYSTEM NORMAL");
    lcd.setCursor(0, 1);
    lcd.print(relayState ? "Relay: CLOSED" : "Relay: OPEN");
  } 
  else if (currentState == STATE_FAULT) {
    lcd.print("FAULT! CELL "); lcd.print(faultCell + 1);
    lcd.setCursor(0, 1);
    lcd.print(getFaultName());
  } 
  else if (currentState == STATE_RECOVERY) {
    lcd.print("RECOVERING...");
    lcd.setCursor(0, 1);
    lcd.print("WAITING...");
  }
}

// ── Main Loop ──────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // 1. Evaluate State Machine & Sensors
  if (now - tLastPoll >= POLL_INTERVAL) {
    processStateMachine(now);
    tLastPoll = now;
  }

  // 2. Hardware updates
  driveBuzzer(now);

  // 3. UI update
  if (now - tLastLCD >= LCD_REFRESH) {
    updateLCD();
    tLastLCD = now;
  }
}
