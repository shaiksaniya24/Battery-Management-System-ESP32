#include <Arduino.h>

// 1. Four operating states
enum class SystemState {
    NORMAL,
    DEGRADED,
    FAILSAFE,
    SHUTDOWN
};

// 2. Fault sources
enum class FaultSource {
    NONE,
    BATTERY_CELL,
    RELAY,
    COMMUNICATION,
    ADC_FROZEN
};

// 3. State variables
SystemState currentState = SystemState::NORMAL;
FaultSource activeFault = FaultSource::NONE;

// 4. Pin Definitions (ESP32)
const int PIN_ADC          = 34; // Potentiometer for ADC
const int PIN_RELAY_CMD    = 25; // Output to Relay (LED)
const int PIN_RELAY_FB     = 26; // Input from Relay (Slide Switch)
const int PIN_COMM_FAULT   = 27; // Button to simulate Comm Fault
const int PIN_VERIFY       = 14; // Button for Verification/Recovery

// 5. Thresholds & Timers
const unsigned long ADC_FROZEN_TIMEOUT = 10000; // 10 seconds without movement = frozen
const int ADC_NOISE_THRESHOLD = 5;
const unsigned long RELAY_TIMEOUT = 1000;       // 1 second

// 6. Monitoring variables
int lastAdcValue = -1;
unsigned long lastAdcChangeTime = 0;
bool relayCommanded = false;
unsigned long relayCmdTime = 0;
unsigned long failsafeStartTime = 0;
int recoveryVerificationSteps = 0;
unsigned long lastVerificationPress = 0;

// Helper to get string representations
const char* getStateName(SystemState s) {
    switch(s) {
        case SystemState::NORMAL:   return "NORMAL";
        case SystemState::DEGRADED: return "DEGRADED";
        case SystemState::FAILSAFE: return "FAILSAFE";
        case SystemState::SHUTDOWN: return "SHUTDOWN";
        default: return "UNKNOWN";
    }
}

const char* getFaultName(FaultSource f) {
    switch(f) {
        case FaultSource::NONE:          return "NONE";
        case FaultSource::BATTERY_CELL:  return "BATTERY_CELL";
        case FaultSource::RELAY:         return "RELAY_MISMATCH";
        case FaultSource::COMMUNICATION: return "COMM_FAULT";
        case FaultSource::ADC_FROZEN:    return "ADC_FROZEN";
        default: return "UNKNOWN";
    }
}

// Structured Logging
void logTransition(SystemState prev, SystemState next, FaultSource fault) {
    // Format: [TIMESTAMP] TRANSITION: PREV -> NEXT | FAULT: FAULT_ID
    Serial.print("[");
    Serial.print(millis());
    Serial.print(" ms] TRANSITION: ");
    Serial.print(getStateName(prev));
    Serial.print(" -> ");
    Serial.print(getStateName(next));
    Serial.print(" | FAULT: ");
    Serial.println(getFaultName(fault));
}

void setup() {
    Serial.begin(115200);
    
    pinMode(PIN_ADC, INPUT);
    pinMode(PIN_RELAY_CMD, OUTPUT);
    pinMode(PIN_RELAY_FB, INPUT_PULLUP);
    pinMode(PIN_COMM_FAULT, INPUT_PULLUP);
    pinMode(PIN_VERIFY, INPUT_PULLUP);
    
    digitalWrite(PIN_RELAY_CMD, LOW);
    
    Serial.println("System Initialized");
    Serial.println("Current State: NORMAL");
    Serial.println("Waiting for potential faults...");
    
    lastAdcValue = analogRead(PIN_ADC);
    lastAdcChangeTime = millis();
}

void loop() {
    unsigned long now = millis();
    
    // --- 1. Read Inputs ---
    int adcValue = analogRead(PIN_ADC);
    bool relayFbClosed = (digitalRead(PIN_RELAY_FB) == LOW); // LOW because it's pullup and switch shorts to GND
    bool commFaultActive = (digitalRead(PIN_COMM_FAULT) == LOW);
    bool verifyBtnPressed = (digitalRead(PIN_VERIFY) == LOW);
    
    // --- 2. Fault Detection ---
    FaultSource detectedFault = FaultSource::NONE;
    
    // Check ADC Frozen (Needs to fluctuate by more than threshold)
    // NOTE for Wokwi users: You must wiggle the potentiometer every 10 seconds, 
    // otherwise it considers the ADC frozen (since Wokwi pots are perfectly stable).
    if (abs(adcValue - lastAdcValue) > ADC_NOISE_THRESHOLD) {
        lastAdcValue = adcValue;
        lastAdcChangeTime = now;
    } else if (now - lastAdcChangeTime > ADC_FROZEN_TIMEOUT) {
        detectedFault = FaultSource::ADC_FROZEN;
    }
    
    // Check Relay Mismatch
    if (now - relayCmdTime > RELAY_TIMEOUT) {
        if (relayCommanded != relayFbClosed) {
            detectedFault = FaultSource::RELAY;
        }
    }
    
    // Check Communication Fault (Simulated via button)
    if (commFaultActive) {
        detectedFault = FaultSource::COMMUNICATION;
    }
    
    // --- 3. State Machine Logic ---
    SystemState nextState = currentState;
    
    switch (currentState) {
        case SystemState::NORMAL:
            // Nominal operation: Toggle relay every 4 seconds to demonstrate operation
            if (now % 8000 < 4000) {
                if (!relayCommanded) { relayCommanded = true; relayCmdTime = now; digitalWrite(PIN_RELAY_CMD, HIGH); }
            } else {
                if (relayCommanded) { relayCommanded = false; relayCmdTime = now; digitalWrite(PIN_RELAY_CMD, LOW); }
            }
            
            if (detectedFault != FaultSource::NONE) {
                activeFault = detectedFault;
                if (detectedFault == FaultSource::COMMUNICATION) {
                    // Minor fault -> DEGRADED
                    nextState = SystemState::DEGRADED;
                } else {
                    // Critical fault -> FAILSAFE
                    nextState = SystemState::FAILSAFE;
                    failsafeStartTime = now;
                    recoveryVerificationSteps = 0;
                }
            }
            break;
            
        case SystemState::DEGRADED:
            // In DEGRADED, system still operates but with limitations (e.g., relay stays off)
            digitalWrite(PIN_RELAY_CMD, LOW); 
            relayCommanded = false;
            
            if (detectedFault != FaultSource::COMMUNICATION && detectedFault != FaultSource::NONE) {
                // Escalate to FAILSAFE if a more severe fault occurs
                activeFault = detectedFault;
                nextState = SystemState::FAILSAFE;
                failsafeStartTime = now;
                recoveryVerificationSteps = 0;
            } else if (detectedFault == FaultSource::NONE && !commFaultActive) {
                // Recover to NORMAL if comm fault clears
                activeFault = FaultSource::NONE;
                nextState = SystemState::NORMAL;
                
                // Reset ADC timer so we don't immediately fault upon return
                lastAdcValue = analogRead(PIN_ADC);
                lastAdcChangeTime = now; 
            }
            break;
            
        case SystemState::FAILSAFE:
            // Ensure system is in a safe state
            digitalWrite(PIN_RELAY_CMD, LOW);
            relayCommanded = false;
            relayCmdTime = now; // Suppress relay mismatch while in failsafe
            
            // Structured Recovery Verification Process
            // User must press the VERIFY button 3 times (1 sec apart) when NO faults are present
            if (verifyBtnPressed && (now - lastVerificationPress > 1000)) {
                lastVerificationPress = now;
                
                if (detectedFault == FaultSource::NONE) {
                    recoveryVerificationSteps++;
                    Serial.print("[");
                    Serial.print(now);
                    Serial.print(" ms] Recovery verification step ");
                    Serial.print(recoveryVerificationSteps);
                    Serial.println("/3 successful.");
                    
                    if (recoveryVerificationSteps >= 3) {
                        Serial.println("Recovery verified. Transitioning to DEGRADED state.");
                        activeFault = FaultSource::NONE;
                        nextState = SystemState::DEGRADED; // Step down to DEGRADED first
                    }
                } else {
                    Serial.print("[");
                    Serial.print(now);
                    Serial.print(" ms] Verification failed! Fault ");
                    Serial.print(getFaultName(detectedFault));
                    Serial.println(" still present. Restarting verification.");
                    recoveryVerificationSteps = 0;
                    failsafeStartTime = now; // Reset failsafe timeout
                }
            }
            
            // If in FAILSAFE for > 60 seconds without successful recovery, transition to SHUTDOWN
            if (now - failsafeStartTime > 60000) {
                nextState = SystemState::SHUTDOWN;
            }
            break;
            
        case SystemState::SHUTDOWN:
            // Terminal state. System halted. Requires physical reset.
            digitalWrite(PIN_RELAY_CMD, LOW);
            break;
    }
    
    // --- 4. Handle State Transitions ---
    if (nextState != currentState) {
        logTransition(currentState, nextState, activeFault);
        currentState = nextState;
    }
    
    delay(50); // Loop delay for debouncing and stability
}
