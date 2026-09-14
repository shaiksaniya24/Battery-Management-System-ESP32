#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ── Pins ───────────────────────────────────────────────
#define PIN_VOLTAGE   34
#define PIN_TEMP      35
#define PIN_CURRENT   32
#define PIN_CLEAR_BTN 33

// ── Thresholds ─────────────────────────────────────────
#define MAX_VOLTAGE 15.0f // 12V system scale
#define MAX_TEMP    80.0f // Celsius scale
#define MAX_CURRENT 20.0f // Amps scale

#define FAULT_V_MAX 14.5f
#define FAULT_T_MAX 65.0f
#define FAULT_I_MAX 15.0f

// ── Timing ─────────────────────────────────────────────
#define SENSOR_POLL_MS    100
#define LCD_REFRESH_MS    200   // 5 Hz refresh rate
#define SCREEN_ROTATE_MS  3000  // 3s per page

// ── State ──────────────────────────────────────────────
enum ScreenID { SCR_BATTERY = 0, SCR_SYSTEM = 1, SCR_TELEMETRY = 2 };
const int NUM_SCREENS = 3;

ScreenID currentScreen = SCR_BATTERY;
unsigned long tLastRotate = 0;
unsigned long tLastRefresh = 0;
unsigned long tLastPoll = 0;

enum FaultState { NORMAL, FAULT_OVERVOLT, FAULT_OVERTEMP, FAULT_OVERCURRENT };
FaultState activeFault = NORMAL;
bool faultLatched = false;

// ── Sensor Data ────────────────────────────────────────
float batVoltage = 0;
float sysTemp = 0;
float loadCurrent = 0;
bool  btnPressed = false;

// ── LCD Cache for Flicker-Free Rendering ───────────────
char lcdBuffer[2][17] = {
  "                ",
  "                "
};

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_VOLTAGE, INPUT);
  pinMode(PIN_TEMP, INPUT);
  pinMode(PIN_CURRENT, INPUT);
  pinMode(PIN_CLEAR_BTN, INPUT_PULLUP);
  
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  Serial.println("System Initialized");
}

float mapFloat(int val, int in_min, int in_max, float out_min, float out_max) {
  return (float)(val - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void pollSensors() {
  batVoltage  = mapFloat(analogRead(PIN_VOLTAGE), 0, 4095, 0, MAX_VOLTAGE);
  sysTemp     = mapFloat(analogRead(PIN_TEMP), 0, 4095, 0, MAX_TEMP);
  loadCurrent = mapFloat(analogRead(PIN_CURRENT), 0, 4095, 0, MAX_CURRENT);
  
  // Button is active LOW (with pullup)
  btnPressed  = (digitalRead(PIN_CLEAR_BTN) == LOW);
}

void handleFaults() {
  if (faultLatched) {
    // Check if user explicitly clears the fault
    if (btnPressed) {
      // Only clear if the underlying physical conditions are safe
      if (batVoltage <= FAULT_V_MAX && sysTemp <= FAULT_T_MAX && loadCurrent <= FAULT_I_MAX) {
        faultLatched = false;
        activeFault = NORMAL;
        tLastRotate = millis(); // Reset rotation timer on return to normal
      }
    }
  } else {
    // Detect new faults
    if (batVoltage > FAULT_V_MAX) {
      activeFault = FAULT_OVERVOLT;
      faultLatched = true;
    } else if (sysTemp > FAULT_T_MAX) {
      activeFault = FAULT_OVERTEMP;
      faultLatched = true;
    } else if (loadCurrent > FAULT_I_MAX) {
      activeFault = FAULT_OVERCURRENT;
      faultLatched = true;
    }
  }
}

// ── Flicker-Free Diffing Print ─────────────────────────
// Updates ONLY characters that have changed, bypassing lcd.clear()
void printLine(int row, const char* str) {
  for (int col = 0; col < 16; col++) {
    char c = str[col];
    
    // If string is shorter than 16 chars, pad remainder with spaces
    if (c == '\0') {
      for (int k = col; k < 16; k++) {
        if (lcdBuffer[row][k] != ' ') {
          lcd.setCursor(k, row);
          lcd.print(' ');
          lcdBuffer[row][k] = ' ';
        }
      }
      break;
    }
    
    // Send I2C command ONLY if the character has physically changed
    if (lcdBuffer[row][col] != c) {
      lcd.setCursor(col, row);
      lcd.print(c);
      lcdBuffer[row][col] = c;
    }
  }
}

void renderScreen() {
  char line0[17];
  char line1[17];
  
  if (faultLatched) {
    snprintf(line0, 17, "!! CRITICAL !!  ");
    switch (activeFault) {
      case FAULT_OVERVOLT:    snprintf(line1, 17, "OVERVOLT: %.1fV ", batVoltage); break;
      case FAULT_OVERTEMP:    snprintf(line1, 17, "OVERTEMP: %.1fC ", sysTemp); break;
      case FAULT_OVERCURRENT: snprintf(line1, 17, "OVERCURR: %.1fA ", loadCurrent); break;
      default:                snprintf(line1, 17, "UNKNOWN FAULT   "); break;
    }
  } else {
    switch (currentScreen) {
      case SCR_BATTERY:
        snprintf(line0, 17, "Bat Status      ");
        snprintf(line1, 17, "V: %.2fV        ", batVoltage);
        break;
      case SCR_SYSTEM:
        snprintf(line0, 17, "System State    ");
        snprintf(line1, 17, "Temp: %.1fC     ", sysTemp);
        break;
      case SCR_TELEMETRY:
        snprintf(line0, 17, "Telemetry Info  ");
        snprintf(line1, 17, "Load: %.2fA     ", loadCurrent);
        break;
    }
  }
  
  printLine(0, line0);
  printLine(1, line1);
}

void loop() {
  unsigned long now = millis();
  
  // 1. High-speed, non-blocking sensor polling and fault detection
  if (now - tLastPoll >= SENSOR_POLL_MS) {
    tLastPoll = now;
    pollSensors();
    handleFaults();
  }
  
  // 2. Rotate screens on timer (Only if there are no faults)
  if (!faultLatched) {
    if (now - tLastRotate >= SCREEN_ROTATE_MS) {
      tLastRotate = now;
      currentScreen = (ScreenID)((currentScreen + 1) % NUM_SCREENS);
    }
  }
  
  // 3. Independent display render cycle
  if (now - tLastRefresh >= LCD_REFRESH_MS) {
    tLastRefresh = now;
    renderScreen();
  }
}
