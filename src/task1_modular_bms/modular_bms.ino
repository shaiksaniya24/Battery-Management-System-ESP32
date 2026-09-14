// ═══════════════════════════════════════════════════════════════════════════════
//  MODULAR BATTERY MANAGEMENT ENGINE
//  Platform  : ESP32 DevKit V1 (Wokwi Simulation)
//  Display   : 16x2 I2C LCD  →  SDA: GPIO 21 | SCL: GPIO 22
//  Cells     : 4x Potentiometers  →  GPIO 34, 35, 32, 33
//  Button    : GPIO 18  (10 kΩ pull-down, cycles LCD pages)
//  LEDs      : GPIO 4 (Green-HEALTHY) | 2 (Yellow-MINOR) |
//              GPIO 15 (Orange-CRITICAL) | 5 (Red-FAULT)
//  Buzzer    : GPIO 19  (active-low buzzer on PACK FAILURE)
//  Serial    : 115200 baud  (real-time telemetry + scaling report)
//
//  SCALABILITY — change NUM_CELLS (4 → 16) here only:
// ═══════════════════════════════════════════════════════════════════════════════

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ─── Compile-Time Pack Sizing ────────────────────────────────────────────────
#define NUM_CELLS          4        // ← SINGLE constant to scale (4 / 8 / 12 / 16)

// ─── Hardware Pin Map ────────────────────────────────────────────────────────
#define BTN_PIN            18
#define LED_GREEN          4
#define LED_YELLOW         2
#define LED_ORANGE         15
#define LED_RED            5
#define BUZZER_PIN         19

// ADC input channels (add more entries when NUM_CELLS > 4)
const uint8_t ADC_PINS[NUM_CELLS] = {34, 35, 32, 33};

// ─── ADC / Voltage Parameters ────────────────────────────────────────────────
#define ADC_BITS           12
#define ADC_MAX_F          4095.0f
#define V_CELL_MIN         2.50f   // Absolute floor  (V)
#define V_CELL_MAX         4.20f   // Absolute ceiling(V)
#define V_UV_FAULT         2.80f   // Undervoltage fault trip (V)
#define V_OV_FAULT         4.22f   // Overvoltage  fault trip (V)
#define ADC_OVERSAMPLE     16      // Oversampling count per channel
#define SAMPLE_PERIOD_MS   1000    // Engine execution period (ms)
#define DEBOUNCE_MS        200     // Button debounce (ms)

// ─── Adaptive Threshold Constants ────────────────────────────────────────────
#define TH_BASE_PCT        2.50f   // Flat-plateau base threshold (%)
#define TH_MAX_PCT         9.00f   // Maximum relaxed threshold cap (%)
#define SOC_KNEE_LOW       20.0f   // Low-SoC knee boundary (%)
#define SOC_KNEE_HIGH      80.0f   // High-SoC knee boundary (%)
#define SOC_KNEE_EXPANSION 2.20f   // Quadratic expansion factor in knees

// ─── Trend Derivative Parameters ─────────────────────────────────────────────
#define EMA_ALPHA          0.20f   // EMA smoothing coefficient
#define DIVERGE_RATE_MV_S  2.00f   // |dΔV/dt| threshold for DIVERGING (mV/s)

// ─── LCD ─────────────────────────────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Custom CGRAM glyphs  (index 0-4)
const uint8_t GLYPH_BAT[8]    = {0x0E,0x1F,0x11,0x1F,0x1F,0x1F,0x1F,0x00};
const uint8_t GLYPH_WARN[8]   = {0x04,0x0E,0x0E,0x1F,0x1F,0x0E,0x04,0x00};
const uint8_t GLYPH_UP[8]     = {0x04,0x0E,0x15,0x04,0x04,0x04,0x04,0x00};
const uint8_t GLYPH_DOWN[8]   = {0x04,0x04,0x04,0x04,0x15,0x0E,0x04,0x00};
const uint8_t GLYPH_STABLE[8] = {0x00,0x1F,0x00,0x00,0x00,0x1F,0x00,0x00};

// ─── Data Structures ─────────────────────────────────────────────────────────
enum HealthState : uint8_t {
    HEALTHY = 0, MINOR_IMBALANCE, CRITICAL_IMBALANCE, PACK_FAILURE
};

enum ImbalanceTrend : uint8_t {
    STABLE = 0, DIVERGING, CONVERGING
};

struct CellMetrics {
    int   adcRaw;
    float voltage;
    float socPct;
    float devPct;          // deviation from pack average (%)
    bool  isWeakest;
    bool  isStrongest;
    bool  uvFault;
    bool  ovFault;
};

struct AdaptiveThreshold {
    float basePct;
    float effectivePct;    // = base × SoC_multiplier
    float socMultiplier;
    bool  inKnee;
};

struct TrendInfo {
    float          rateMvPerSec;   // EMA-filtered d(ΔV)/dt
    ImbalanceTrend direction;
    uint32_t       divergeMs;
};

struct PackAnalysis {                 // ← CLEAN PUBLIC INTERFACE
    CellMetrics       cells[NUM_CELLS];
    float             packVoltage;
    float             avgVoltage;
    float             deltaVmV;
    float             imbalancePct;
    float             avgSocPct;
    int               weakIdx;
    int               strongIdx;
    HealthState       health;
    AdaptiveThreshold threshold;
    TrendInfo         trend;
    uint32_t          sampleIndex;
};

// ─── Engine Internal State ────────────────────────────────────────────────────
static PackAnalysis g_pack;
static float        g_prevDeltaVmV   = 0.0f;
static float        g_filteredRate   = 0.0f;
static uint32_t     g_lastEngineMs   = 0;
static uint32_t     g_divergeStartMs = 0;

// ─── UI State ─────────────────────────────────────────────────────────────────
static int      g_page         = 0;
static bool     g_lastBtn      = LOW;
static uint32_t g_debounceMs   = 0;
static uint32_t g_lastSampleMs = 0;

// Total pages: 1 (Overview) + NUM_CELLS + 1 (Extremes/Trend) + 1 (Scaling)
static const int TOTAL_PAGES = NUM_CELLS + 3;

// ═══════════════════════════════════════════════════════════════════════════════
//  MODULE 1 — Hardware Abstraction (Oversampled ADC)
// ═══════════════════════════════════════════════════════════════════════════════
int oversampleADC(uint8_t pin) {
    long acc = 0;
    for (int k = 0; k < ADC_OVERSAMPLE; k++) {
        acc += analogRead(pin);
        delayMicroseconds(80);
    }
    return (int)(acc / ADC_OVERSAMPLE);
}

float adcToVoltage(int raw) {
    return V_CELL_MIN + (raw / ADC_MAX_F) * (V_CELL_MAX - V_CELL_MIN);
}

float voltageToSoC(float v) {
    if (v <= V_CELL_MIN) return 0.0f;
    if (v >= V_CELL_MAX) return 100.0f;
    return ((v - V_CELL_MIN) / (V_CELL_MAX - V_CELL_MIN)) * 100.0f;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MODULE 2 — Adaptive Threshold Computation  f(SoC)
// ═══════════════════════════════════════════════════════════════════════════════
void computeAdaptiveThreshold(float avgSoc, AdaptiveThreshold &th) {
    th.basePct       = TH_BASE_PCT;
    th.socMultiplier = 1.0f;
    th.inKnee        = false;

    if (avgSoc < SOC_KNEE_LOW) {
        th.inKnee = true;
        float depth = (SOC_KNEE_LOW - avgSoc) / SOC_KNEE_LOW;   // 0..1
        th.socMultiplier = 1.0f + (SOC_KNEE_EXPANSION - 1.0f) * depth * depth;
    } else if (avgSoc > SOC_KNEE_HIGH) {
        th.inKnee = true;
        float depth = (avgSoc - SOC_KNEE_HIGH) / (100.0f - SOC_KNEE_HIGH);
        th.socMultiplier = 1.0f + (SOC_KNEE_EXPANSION - 1.0f) * depth * depth;
    }

    float eff = th.basePct * th.socMultiplier;
    th.effectivePct = (eff < TH_BASE_PCT) ? TH_BASE_PCT : (eff > TH_MAX_PCT ? TH_MAX_PCT : eff);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MODULE 3 — Imbalance Trend Tracker  (EMA-filtered d(ΔV)/dt)
// ═══════════════════════════════════════════════════════════════════════════════
void computeTrend(float currentDeltaVmV, uint32_t nowMs, TrendInfo &tr) {
    float dtSec = (g_lastEngineMs > 0 && nowMs > g_lastEngineMs)
                  ? (float)(nowMs - g_lastEngineMs) / 1000.0f
                  : SAMPLE_PERIOD_MS / 1000.0f;

    float rawRate = (dtSec > 0.001f && g_lastEngineMs > 0)
                    ? (currentDeltaVmV - g_prevDeltaVmV) / dtSec
                    : 0.0f;

    g_filteredRate = EMA_ALPHA * rawRate + (1.0f - EMA_ALPHA) * g_filteredRate;
    tr.rateMvPerSec = g_filteredRate;

    if (g_filteredRate > DIVERGE_RATE_MV_S) {
        tr.direction = DIVERGING;
        if (g_divergeStartMs == 0) g_divergeStartMs = nowMs;
        tr.divergeMs = nowMs - g_divergeStartMs;
    } else if (g_filteredRate < -DIVERGE_RATE_MV_S) {
        tr.direction = CONVERGING;
        g_divergeStartMs = 0;
        tr.divergeMs = 0;
    } else {
        tr.direction = STABLE;
        g_divergeStartMs = 0;
        tr.divergeMs = 0;
    }

    g_prevDeltaVmV = currentDeltaVmV;
    g_lastEngineMs = nowMs;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MODULE 4 — Core Battery Intelligence Engine  (O(N) single-pass)
// ═══════════════════════════════════════════════════════════════════════════════
void runEngine() {
    uint32_t nowMs = millis();
    g_pack.sampleIndex++;

    float vMin = 9999.0f, vMax = -9999.0f, vSum = 0.0f, socSum = 0.0f;
    int   wIdx = 0, sIdx = 0;
    bool  anyUV = false, anyOV = false;

    // ── Single-pass acquisition + extremes search ─────────────────────────
    for (int i = 0; i < NUM_CELLS; i++) {
        g_pack.cells[i].adcRaw  = oversampleADC(ADC_PINS[i]);
        g_pack.cells[i].voltage = adcToVoltage(g_pack.cells[i].adcRaw);
        g_pack.cells[i].socPct  = voltageToSoC(g_pack.cells[i].voltage);
        g_pack.cells[i].uvFault = (g_pack.cells[i].voltage < V_UV_FAULT);
        g_pack.cells[i].ovFault = (g_pack.cells[i].voltage > V_OV_FAULT);

        if (g_pack.cells[i].uvFault) anyUV = true;
        if (g_pack.cells[i].ovFault) anyOV = true;

        vSum   += g_pack.cells[i].voltage;
        socSum += g_pack.cells[i].socPct;
        if (g_pack.cells[i].voltage < vMin) { vMin = g_pack.cells[i].voltage; wIdx = i; }
        if (g_pack.cells[i].voltage > vMax) { vMax = g_pack.cells[i].voltage; sIdx = i; }
    }

    g_pack.packVoltage  = vSum;
    g_pack.avgVoltage   = vSum / NUM_CELLS;
    g_pack.avgSocPct    = socSum / NUM_CELLS;
    g_pack.deltaVmV     = (vMax - vMin) * 1000.0f;
    g_pack.imbalancePct = (g_pack.avgVoltage > 0.01f)
                          ? ((vMax - vMin) / g_pack.avgVoltage) * 100.0f : 0.0f;
    g_pack.weakIdx      = wIdx;
    g_pack.strongIdx    = sIdx;

    // ── Second pass: per-cell deviation & flags ───────────────────────────
    for (int i = 0; i < NUM_CELLS; i++) {
        g_pack.cells[i].isWeakest   = (i == wIdx);
        g_pack.cells[i].isStrongest = (i == sIdx);
        g_pack.cells[i].devPct      = (g_pack.avgVoltage > 0.01f)
                                      ? fabsf(g_pack.cells[i].voltage - g_pack.avgVoltage) / g_pack.avgVoltage * 100.0f
                                      : 0.0f;
    }

    // ── Adaptive threshold & trend ────────────────────────────────────────
    computeAdaptiveThreshold(g_pack.avgSocPct, g_pack.threshold);
    computeTrend(g_pack.deltaVmV, nowMs, g_pack.trend);

    // ── Health classifier (adaptive boundary) ─────────────────────────────
    float eff = g_pack.threshold.effectivePct;
    if (anyUV || anyOV)
        g_pack.health = PACK_FAILURE;
    else if (g_pack.imbalancePct >= eff * 1.80f)
        g_pack.health = CRITICAL_IMBALANCE;
    else if (g_pack.imbalancePct >= eff)
        g_pack.health = MINOR_IMBALANCE;
    else
        g_pack.health = HEALTHY;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MODULE 5 — LED & Buzzer Actuator
// ═══════════════════════════════════════════════════════════════════════════════
void updateActuators() {
    digitalWrite(LED_GREEN,  LOW);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_ORANGE, LOW);
    digitalWrite(LED_RED,    LOW);
    digitalWrite(BUZZER_PIN, HIGH); // active-low buzzer OFF

    switch (g_pack.health) {
        case HEALTHY:            digitalWrite(LED_GREEN,  HIGH); break;
        case MINOR_IMBALANCE:    digitalWrite(LED_YELLOW, HIGH); break;
        case CRITICAL_IMBALANCE: digitalWrite(LED_ORANGE, HIGH); break;
        case PACK_FAILURE:
            digitalWrite(LED_RED,    HIGH);
            digitalWrite(BUZZER_PIN, LOW);  // active-low = sound ON
            break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MODULE 6 — LCD Multi-Page Renderer (auto-scales with NUM_CELLS)
// ═══════════════════════════════════════════════════════════════════════════════
void lcdOverview() {
    // Row 0: Battery icon | Pack voltage | Avg SoC | Trend icon
    lcd.setCursor(0, 0);
    lcd.write(byte(0));           // battery glyph
    lcd.print(" ");
    lcd.print(g_pack.packVoltage, 2);
    lcd.print("V ");
    lcd.print((int)g_pack.avgSocPct);
    lcd.print("% ");
    if      (g_pack.trend.direction == DIVERGING)  lcd.write(byte(2));
    else if (g_pack.trend.direction == CONVERGING) lcd.write(byte(3));
    else                                           lcd.write(byte(4));

    // Row 1: Health label
    lcd.setCursor(0, 1);
    switch (g_pack.health) {
        case HEALTHY:            lcd.print("  STATUS:HEALTHY"); break;
        case MINOR_IMBALANCE:    lcd.write(byte(1)); lcd.print("MINOR IMBAL   "); break;
        case CRITICAL_IMBALANCE: lcd.write(byte(1)); lcd.print("CRIT IMBALANCE"); break;
        case PACK_FAILURE:       lcd.write(byte(1)); lcd.print("PACK FAILURE! "); break;
    }
}

void lcdCell(int idx) {
    const CellMetrics &c = g_pack.cells[idx];
    // Row 0: C#: voltage SoC%
    lcd.setCursor(0, 0);
    lcd.print("C"); lcd.print(idx + 1); lcd.print(":");
    lcd.print(c.voltage, 3); lcd.print("V ");
    lcd.print((int)c.socPct); lcd.print("%  ");

    // Row 1: status + deviation
    lcd.setCursor(0, 1);
    if      (c.uvFault)     { lcd.write(byte(1)); lcd.print("UV! Dev:");  }
    else if (c.ovFault)     { lcd.write(byte(1)); lcd.print("OV! Dev:");  }
    else if (c.isStrongest) { lcd.write(byte(2)); lcd.print("BEST Dev:"); }
    else if (c.isWeakest)   { lcd.write(byte(3)); lcd.print("WEAK Dev:"); }
    else                    {                     lcd.print("OK   Dev:"); }
    lcd.print(c.devPct, 1); lcd.print("%  ");
}

void lcdExtremesTrend() {
    // Row 0: Weakest | Strongest
    lcd.setCursor(0, 0);
    lcd.print("W:C"); lcd.print(g_pack.weakIdx   + 1);
    lcd.print(" S:C"); lcd.print(g_pack.strongIdx + 1);
    lcd.print(" dV:");
    lcd.print((int)g_pack.deltaVmV); lcd.print("m");

    // Row 1: Adaptive threshold | Trend
    lcd.setCursor(0, 1);
    lcd.print("Th:"); lcd.print(g_pack.threshold.effectivePct, 1); lcd.print("% ");
    switch (g_pack.trend.direction) {
        case DIVERGING:  lcd.print("DRIFT^"); break;
        case CONVERGING: lcd.print("CNVRG "); break;
        default:         lcd.print("STABLE"); break;
    }
}

void lcdScalingReport() {
    // Page showing real-time scaling metrics
    lcd.setCursor(0, 0);
    lcd.print("Cells:"); lcd.print(NUM_CELLS);
    lcd.print(" S#"); lcd.print(g_pack.sampleIndex % 1000);

    lcd.setCursor(0, 1);
    lcd.print("Knee:");
    lcd.print(g_pack.threshold.inKnee ? "Y" : "N");
    lcd.print(" x"); lcd.print(g_pack.threshold.socMultiplier, 2);
}

void renderPage() {
    lcd.clear();
    if (g_page == 0) {
        lcdOverview();
    } else if (g_page >= 1 && g_page <= NUM_CELLS) {
        lcdCell(g_page - 1);
    } else if (g_page == NUM_CELLS + 1) {
        lcdExtremesTrend();
    } else {
        lcdScalingReport();
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MODULE 7 — Serial Telemetry Report
// ═══════════════════════════════════════════════════════════════════════════════
void serialReport() {
    Serial.println(F("══════════════════════════════════════════════════════════════"));
    Serial.printf("  BMS ENGINE  |  Sample #%lu  |  Pack: %dS Series\r\n",
                  (unsigned long)g_pack.sampleIndex, NUM_CELLS);
    Serial.println(F("  ────┬─────────┬─────────┬───────┬────────────────────────"));
    Serial.println(F("  #   │ Voltage │  Dev%   │  SoC  │ Status"));
    Serial.println(F("  ────┼─────────┼─────────┼───────┼────────────────────────"));

    for (int i = 0; i < NUM_CELLS; i++) {
        const char *tag = "Normal  ";
        if      (g_pack.cells[i].uvFault)     tag = "UV FAULT";
        else if (g_pack.cells[i].ovFault)     tag = "OV FAULT";
        else if (g_pack.cells[i].isStrongest) tag = "STRONGEST";
        else if (g_pack.cells[i].isWeakest)   tag = "WEAKEST ";
        Serial.printf("   C%d │ %6.3fV │ %5.2f%% │ %4.1f%% │ %s\r\n",
                      i+1,
                      g_pack.cells[i].voltage,
                      g_pack.cells[i].devPct,
                      g_pack.cells[i].socPct,
                      tag);
    }

    Serial.println(F("  ────┴─────────┴─────────┴───────┴────────────────────────"));
    Serial.printf("  Pack      : %.3f V   Avg : %.3f V\r\n", g_pack.packVoltage, g_pack.avgVoltage);
    Serial.printf("  Delta-V   : %.1f mV   Imbalance: %.2f%%\r\n", g_pack.deltaVmV, g_pack.imbalancePct);
    Serial.printf("  Avg SoC   : %.1f%%\r\n", g_pack.avgSocPct);
    Serial.printf("  Weakest   : C%d (%.3fV)   Strongest: C%d (%.3fV)\r\n",
                  g_pack.weakIdx+1, g_pack.cells[g_pack.weakIdx].voltage,
                  g_pack.strongIdx+1, g_pack.cells[g_pack.strongIdx].voltage);
    Serial.printf("  Adap.Th   : %.2f%%  (Base: %.1f%% × %.2fx)  Knee:%s\r\n",
                  g_pack.threshold.effectivePct, g_pack.threshold.basePct,
                  g_pack.threshold.socMultiplier,
                  g_pack.threshold.inKnee ? "YES" : "NO ");
    Serial.printf("  Trend     : dV/dt = %+.2f mV/s  →  %s\r\n",
                  g_pack.trend.rateMvPerSec,
                  g_pack.trend.direction == DIVERGING  ? "DIVERGING" :
                  g_pack.trend.direction == CONVERGING ? "CONVERGING" : "STABLE");

    const char *hl[] = { "HEALTHY", "MINOR IMBALANCE", "CRITICAL IMBALANCE", "PACK FAILURE" };
    Serial.printf("  Health    : %s\r\n", hl[g_pack.health]);

    // ── Inline Scaling Analysis ───────────────────────────────────────────
    Serial.println(F("\r\n  ┌─ SCALING ANALYSIS ─────────────────────────────────────┐"));
    Serial.printf("  │  NUM_CELLS = %2d  │  sizeof(CellMetrics)   = %2d B       │\r\n",
                  NUM_CELLS, (int)sizeof(CellMetrics));
    Serial.printf("  │  cells[] RAM = %3d B  │  PackAnalysis RAM = %3d B   │\r\n",
                  (int)(NUM_CELLS * sizeof(CellMetrics)), (int)sizeof(PackAnalysis));
    Serial.printf("  │  ADC sample time ≈ %4lu ms  (16× oversample/ch)      │\r\n",
                  (unsigned long)(NUM_CELLS * ADC_OVERSAMPLE * 80UL / 1000UL));
    Serial.println(F("  │  Algorithm: O(N) single-pass  │  Heap alloc: 0 bytes  │"));
    Serial.println(F("  └───────────────────────────────────────────────────────┘"));
    Serial.println(F("══════════════════════════════════════════════════════════════\r\n"));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MODULE 8 — Debounced Button Handler
// ═══════════════════════════════════════════════════════════════════════════════
void checkButton() {
    bool st = digitalRead(BTN_PIN);
    if (st == HIGH && g_lastBtn == LOW) {
        uint32_t now = millis();
        if (now - g_debounceMs > DEBOUNCE_MS) {
            g_page = (g_page + 1) % TOTAL_PAGES;
            renderPage();
            g_debounceMs = now;
        }
    }
    g_lastBtn = st;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(200);

    analogReadResolution(ADC_BITS);
    analogSetAttenuation(ADC_11db);
    for (int i = 0; i < NUM_CELLS; i++) pinMode(ADC_PINS[i], INPUT);

    pinMode(BTN_PIN, INPUT);
    pinMode(LED_GREEN,  OUTPUT); digitalWrite(LED_GREEN,  LOW);
    pinMode(LED_YELLOW, OUTPUT); digitalWrite(LED_YELLOW, LOW);
    pinMode(LED_ORANGE, OUTPUT); digitalWrite(LED_ORANGE, LOW);
    pinMode(LED_RED,    OUTPUT); digitalWrite(LED_RED,    LOW);
    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, HIGH); // active-low OFF

    Wire.begin(21, 22);
    lcd.init();
    lcd.backlight();
    lcd.createChar(0, (uint8_t*)GLYPH_BAT);
    lcd.createChar(1, (uint8_t*)GLYPH_WARN);
    lcd.createChar(2, (uint8_t*)GLYPH_UP);
    lcd.createChar(3, (uint8_t*)GLYPH_DOWN);
    lcd.createChar(4, (uint8_t*)GLYPH_STABLE);

    lcd.setCursor(0, 0); lcd.print(" BMS Engine v2  ");
    lcd.setCursor(0, 1); lcd.printf("  %d-Cell Modular  ", NUM_CELLS);
    delay(1800);

    runEngine();
    updateActuators();
    renderPage();
    serialReport();

    g_lastSampleMs = millis();
    Serial.println(F("[BOOT] Modular Battery Intelligence Engine Running."));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
    uint32_t now = millis();

    if (now - g_lastSampleMs >= SAMPLE_PERIOD_MS) {
        runEngine();
        updateActuators();
        renderPage();
        serialReport();
        g_lastSampleMs = now;
    }

    checkButton();
}
