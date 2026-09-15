#define BLYNK_TEMPLATE_ID "TMPL3MCNLNze-"
#define BLYNK_TEMPLATE_NAME "BMS"
#define BLYNK_AUTH_TOKEN "gLmMp8Y6JghVlOyVPSwyjQcrHkoVnC79"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

char ssid[] = "Wokwi-GUEST";
char pass[] = "";

LiquidCrystal_I2C lcd(0x27, 16, 2);

// =====================================================
// HARDWARE CONFIGURATION
// =====================================================

constexpr uint8_t CELL_COUNT = 4;

const uint8_t CELL_PINS[CELL_COUNT] = {
    34,  // Cell 1
    35,  // Cell 2
    32,  // Cell 3
    33   // Cell 4
};

constexpr uint8_t RED_LED = 2;
constexpr uint8_t GREEN_LED = 4;
constexpr uint8_t YELLOW_LED = 5;
constexpr uint8_t BUZZER = 18;
constexpr uint8_t RELAY = 19;
constexpr uint8_t RELAY_FEEDBACK_PIN = 23;

// Keep false in Wokwi unless GPIO 23 is wired to relay feedback.
constexpr bool USE_PHYSICAL_RELAY_FEEDBACK = false;

// Original circuit behaviour:
// LOW  = battery connected
// HIGH = battery disconnected
constexpr uint8_t RELAY_CONNECTED_LEVEL = LOW;
constexpr uint8_t RELAY_DISCONNECTED_LEVEL = HIGH;

// =====================================================
// TIMING CONFIGURATION
// =====================================================

constexpr unsigned long SAMPLE_INTERVAL_MS = 5000;
constexpr unsigned long TRIP_DEBOUNCE_MS = 10000;
constexpr unsigned long RECOVERY_DELAY_MS = 15000;
constexpr unsigned long LCD_REFRESH_INTERVAL_MS = 250;
constexpr unsigned long LCD_PAGE_INTERVAL_MS = 4000;
constexpr unsigned long SYSTEM_RECOVERY_VERIFY_MS = 15000;
constexpr unsigned long RELAY_FEEDBACK_SETTLE_MS = 1000;
constexpr unsigned long SHUTDOWN_ESCALATION_MS = 30000;
constexpr unsigned long ADC_FROZEN_TIMEOUT_MS = 30000;
constexpr unsigned long WIFI_CONNECT_TIMEOUT_MS = 10000;
constexpr unsigned long RECONNECT_BACKOFF_MS = 5000;
constexpr unsigned long BLYNK_RETRY_INTERVAL_MS = 1000;
constexpr unsigned long TELEMETRY_HEARTBEAT_MS = 30000;
constexpr unsigned long TELEMETRY_FIELD_INTERVAL_MS = 300;
constexpr unsigned long RSSI_EVENT_COOLDOWN_MS = 30000;
constexpr unsigned long QUEUE_FULL_LOG_INTERVAL_MS = 30000;
constexpr unsigned long BLYNK_CONNECT_TIMEOUT_MS = 20000;

// =====================================================
// SERIAL LOGGING CONFIGURATION
// =====================================================

constexpr bool PRINT_PERIODIC_REPORT = true;
constexpr bool PRINT_TELEMETRY_EVENTS = false;
constexpr bool PRINT_QUEUE_WARNINGS = true;

// =====================================================
// ANALYSIS CONFIGURATION
// =====================================================

constexpr uint8_t FILTER_WINDOW_SIZE = 5;

constexpr float ADC_MIN_VOLTAGE = 0.0;
constexpr float ADC_MAX_VOLTAGE = 3.3;
constexpr float ADC_MOVEMENT_TOLERANCE = 0.020;
constexpr float TELEMETRY_CELL_CHANGE = 0.030;
constexpr float TELEMETRY_IMBALANCE_CHANGE = 0.020;
constexpr int TELEMETRY_RSSI_CHANGE = 10;

// Changes larger than this within one sample are suspicious.
constexpr float MAX_SINGLE_SAMPLE_JUMP = 0.50;

// Used when determining whether several cells changed together.
constexpr float LOAD_CHANGE_SIMILARITY = 0.20;

// Module 1 adaptive-imbalance settings.
constexpr float BASE_IMBALANCE_THRESHOLD = 0.100;
constexpr float DISCHARGE_COMPENSATION = 0.500;
constexpr float IMBALANCE_HYSTERESIS = 0.020;
constexpr float TREND_TOLERANCE = 0.005;

// =====================================================
// MODULE 1 TYPES
// =====================================================

enum ImbalanceTrend {
    IMBALANCE_STABLE,
    IMBALANCE_INCREASING,
    IMBALANCE_DECREASING
};

struct BatteryInfo {
    float cellVoltage[CELL_COUNT];

    float totalVoltage;
    float averageVoltage;

    float weakestVoltage;
    float strongestVoltage;

    uint8_t weakestCellIndex;
    uint8_t strongestCellIndex;

    float lastImbalance;
    float imbalance;

    ImbalanceTrend imbalanceTrend;

    float dischargeRate;
    float adaptiveThreshold;
    float recoveryThreshold;

    bool imbalanceExceeded;
};

// =====================================================
// MODULE 2 TYPES
// =====================================================

enum FaultId {
    FAULT_NONE,
    FAULT_IMBALANCE,
    FAULT_SENSOR_JUMP,
    FAULT_SENSOR_OUT_OF_RANGE,
    FAULT_ADC_FROZEN,
    FAULT_RELAY_MISMATCH,
    FAULT_COMMUNICATION_LOST
};

enum RelayState {
    RELAY_CONNECTED,
    RELAY_TRIP_DEBOUNCE,
    RELAY_DISCONNECTED,
    RELAY_RECOVERY_WAIT
};

struct SensorHealth {
    bool unrealisticJump[CELL_COUNT];
    bool outOfRange[CELL_COUNT];

    uint8_t faultCellIndex;
    FaultId activeFault;
};

struct ProtectionInfo {
    RelayState relayState;
    FaultId activeFault;

    unsigned long stateEnteredAt;
    unsigned long lastTransitionAt;

    uint32_t transitionCount;
};

// =====================================================
// MODULE 4 TYPES
// =====================================================

enum SystemState {
    NORMAL,
    DEGRADED,
    FAILSAFE,
    SHUTDOWN
};

enum FaultSource {
    SOURCE_NONE,
    SOURCE_BATTERY,
    SOURCE_RELAY,
    SOURCE_COMMUNICATION,
    SOURCE_ADC
};

struct SystemStatus {
    SystemState state;

    FaultId activeFault;
    FaultSource activeSource;
    uint8_t activeCellIndex;
    unsigned long activeFaultDetectedAt;

    // Retained information about the most recently detected fault.
    FaultId lastFault;
    FaultSource lastSource;
    uint8_t lastFaultCellIndex;
    unsigned long lastFaultDetectedAt;

    unsigned long stateEnteredAt;
    uint32_t transitionCount;

    bool recoveryVerificationActive;
    unsigned long recoveryVerificationStartedAt;
};

struct AdcHealth {
    bool frozen[CELL_COUNT];
    uint8_t frozenCellIndex;
};

// =====================================================
// MODULE 5 TYPES
// =====================================================

enum ConnectionState {
    CONNECTION_OFFLINE,
    WIFI_CONNECTING,
    BLYNK_CONNECTING,
    CONNECTION_ONLINE,
    RECONNECT_BACKOFF
};

enum TelemetryReason {
    EVENT_STARTUP,
    EVENT_CELL_CHANGE,
    EVENT_IMBALANCE_CHANGE,
    EVENT_RELAY_CHANGE,
    EVENT_FAULT_CHANGE,
    EVENT_STATE_CHANGE,
    EVENT_CONNECTION_CHANGE,
    EVENT_RSSI_CHANGE,
    EVENT_HEARTBEAT
};

// =====================================================
// MODULE 6 TYPES
// =====================================================

enum RiskLevel {
    RISK_LOW,
    RISK_MODERATE,
    RISK_HIGH,
    RISK_CRITICAL
};

struct AnalyticsInfo {
    float riskScore;
    RiskLevel riskLevel;
    uint32_t totalFaultCount;
    float stateOfCharge;
    bool socAvailable;
};

struct FaultHistoryEntry {
    unsigned long timestamp;
    FaultId fault;
    FaultSource source;
    uint8_t cellIndex;
    SystemState state;
};

struct TelemetrySnapshot {
    unsigned long timestamp;
    float cellVoltage[CELL_COUNT];
    float averageVoltage;
    float imbalance;
    float adaptiveThreshold;
    float dischargeRate;
    uint8_t weakestCell;
    uint8_t strongestCell;
    RelayState relayState;
    SystemState systemState;
    FaultId fault;
    FaultSource faultSource;
    ImbalanceTrend trend;
    int rssi;
    uint8_t queueDepth;
    bool storedOffline;
    TelemetryReason reason;
    float riskScore;
    RiskLevel riskLevel;
    uint32_t totalFaultCount;
    FaultId lastFault;
    FaultSource lastFaultSource;
    uint8_t lastFaultCell;
    unsigned long lastFaultTimestamp;
    float stateOfCharge;
    bool socAvailable;
    uint32_t droppedEvents;
};

// =====================================================
// GLOBAL INFORMATION OBJECTS
// =====================================================

BatteryInfo batteryInfo;
SensorHealth sensorHealth;
ProtectionInfo protectionInfo;
SystemStatus systemStatus;
AdcHealth adcHealth;
AnalyticsInfo analyticsInfo;

constexpr uint8_t TELEMETRY_QUEUE_SIZE = 24;
TelemetrySnapshot telemetryQueue[TELEMETRY_QUEUE_SIZE];
uint8_t telemetryQueueHead = 0;
uint8_t telemetryQueueTail = 0;
uint8_t telemetryQueueCount = 0;
uint32_t droppedTelemetryEvents = 0;

constexpr uint8_t FAULT_HISTORY_SIZE = 16;
FaultHistoryEntry faultHistory[FAULT_HISTORY_SIZE];
uint8_t faultHistoryHead = 0;
uint8_t faultHistoryCount = 0;

ConnectionState connectionState = CONNECTION_OFFLINE;
unsigned long connectionStateEnteredAt = 0;
unsigned long previousBlynkAttemptAt = 0;
bool networkOutageInjection = false;

bool telemetryTransmissionActive = false;
uint8_t telemetryFieldIndex = 0;
unsigned long previousTelemetryFieldAt = 0;

bool telemetryBaselineReady = false;
float lastEventCellVoltage[CELL_COUNT];
float lastEventImbalance = 0.0;
uint8_t lastEventWeakestCell = 0;
uint8_t lastEventStrongestCell = 0;
RelayState lastEventRelayState = RELAY_CONNECTED;
SystemState lastEventSystemState = NORMAL;
FaultId lastEventFault = FAULT_NONE;
ConnectionState lastEventConnectionState = CONNECTION_OFFLINE;
int lastEventRssi = -127;
unsigned long lastTelemetryEventAt = 0;
unsigned long lastRssiEventAt = 0;
unsigned long lastQueueFullLogAt = 0;

int getCurrentRssi();

// =====================================================
// MEASUREMENT HISTORY
// =====================================================

float rawVoltage[CELL_COUNT];
float previousRawVoltage[CELL_COUNT];

float voltageHistory[CELL_COUNT][FILTER_WINDOW_SIZE];

uint8_t historyIndex = 0;
uint8_t historyCount = 0;

float previousAverageVoltage = 0.0;
unsigned long previousSampleTime = 0;
unsigned long previousLoopSampleTime = 0;
unsigned long lastAdcMovementTime[CELL_COUNT];
unsigned long frozenCandidateStartedAt[CELL_COUNT];
bool relayCommandDisconnected = false;
bool lastRelayCommandDisconnected = false;
unsigned long relayCommandChangedAt = 0;
bool relayMismatchInjection = false;
bool shutdownResetRequested = false;

bool firstSample = true;

// =====================================================
// MODULE 3: LCD DISPLAY ENGINE
// =====================================================

enum LcdPage {
    LCD_PAGE_BATTERY,
    LCD_PAGE_SYSTEM,
    LCD_PAGE_TELEMETRY
};

LcdPage currentLcdPage = LCD_PAGE_BATTERY;

unsigned long previousLcdRefreshTime = 0;
unsigned long previousLcdPageTime = 0;

char lcdCache[2][17];
bool faultDisplayLatched = false;
FaultId latchedDisplayFault = FAULT_NONE;
uint8_t latchedDisplayFaultCell = 0;

// =====================================================
// OPTIONAL FAULT INJECTION
// =====================================================

enum FaultInjectionMode {
    INJECT_NONE,
    INJECT_FROZEN,
    INJECT_JUMP,
    INJECT_OUT_OF_RANGE
};

FaultInjectionMode injectionMode = INJECT_NONE;

bool jumpToggle = false;
float frozenInjectedValue = 0.0;

// =====================================================
// TEXT FUNCTIONS
// =====================================================

const char* getTrendText(ImbalanceTrend trend) {
    switch (trend) {
        case IMBALANCE_INCREASING:
            return "INCREASING";

        case IMBALANCE_DECREASING:
            return "DECREASING";

        default:
            return "STABLE";
    }
}

const char* getFaultText(FaultId fault) {
    switch (fault) {
        case FAULT_IMBALANCE:
            return "IMBALANCE";

        case FAULT_SENSOR_JUMP:
            return "SENSOR_JUMP";

        case FAULT_SENSOR_OUT_OF_RANGE:
            return "SENSOR_OUT_OF_RANGE";

        case FAULT_ADC_FROZEN:
            return "ADC_FROZEN";

        case FAULT_RELAY_MISMATCH:
            return "RELAY_MISMATCH";

        case FAULT_COMMUNICATION_LOST:
            return "COMMUNICATION_LOST";

        default:
            return "NONE";
    }
}

const char* getRelayStateText(RelayState state) {
    switch (state) {
        case RELAY_CONNECTED:
            return "CONNECTED";

        case RELAY_TRIP_DEBOUNCE:
            return "TRIP_DEBOUNCE";

        case RELAY_DISCONNECTED:
            return "DISCONNECTED";

        case RELAY_RECOVERY_WAIT:
            return "RECOVERY_WAIT";

        default:
            return "UNKNOWN";
    }
}

// =====================================================
// ADC READING
// =====================================================

float readVoltage(uint8_t pin) {
    int rawADC = analogRead(pin);

    return (rawADC / 4095.0) * 3.30;
}

// =====================================================
// SERIAL FAULT-INJECTION COMMANDS
// =====================================================

void handleSerialCommands() {
    if (!Serial.available()) {
        return;
    }

    char command = Serial.read();

    switch (command) {
        case 'f':
        case 'F':
            injectionMode = INJECT_FROZEN;
            frozenInjectedValue = rawVoltage[0];
            Serial.println("FAULT INJECTION: ADC Cell 1 frozen");
            break;

        case 'j':
        case 'J':
            injectionMode = INJECT_JUMP;

            Serial.println(
                "FAULT INJECTION: Cell 1 unrealistic jumps"
            );
            break;

        case 'r':
        case 'R':
            injectionMode = INJECT_OUT_OF_RANGE;

            Serial.println(
                "FAULT INJECTION: Cell 1 out of range"
            );
            break;

        case 'c':
        case 'C':
            injectionMode = INJECT_NONE;
            relayMismatchInjection = false;
            networkOutageInjection = false;

            Serial.println(
                "FAULT INJECTION: All injections cleared"
            );
            break;

        case 'n':
        case 'N':
            networkOutageInjection = true;
            Blynk.disconnect();
            WiFi.disconnect();
            Serial.println("FAULT INJECTION: Network outage");
            break;

        case 'm':
        case 'M':
            relayMismatchInjection = true;

            Serial.println(
                "FAULT INJECTION: Relay feedback mismatch"
            );
            break;

        case 'x':
        case 'X':
            shutdownResetRequested = true;

            Serial.println(
                "SYSTEM: Manual shutdown reset requested"
            );
            break;

        case 'a':
        case 'A':
            if (systemStatus.activeFault == FAULT_NONE &&
                systemStatus.state != SHUTDOWN) {
                faultDisplayLatched = false;
                latchedDisplayFault = FAULT_NONE;
                currentLcdPage = LCD_PAGE_BATTERY;
                previousLcdPageTime = millis();
                Serial.println("LCD FAULT SCREEN: Acknowledged");
            }
            else {
                Serial.println("LCD FAULT SCREEN: Fault still active");
            }
            break;

    }
}

const char* getSystemStateText(SystemState state) {
    switch (state) {
        case NORMAL:
            return "NORMAL";
        case DEGRADED:
            return "DEGRADED";
        case FAILSAFE:
            return "FAILSAFE";
        case SHUTDOWN:
            return "SHUTDOWN";
        default:
            return "UNKNOWN";
    }
}

const char* getFaultSourceText(FaultSource source) {
    switch (source) {
        case SOURCE_BATTERY:
            return "BATTERY";
        case SOURCE_RELAY:
            return "RELAY";
        case SOURCE_COMMUNICATION:
            return "COMMUNICATION";
        case SOURCE_ADC:
            return "ADC";
        default:
            return "NONE";
    }
}

bool isCriticalFault(FaultId fault) {
    return fault != FAULT_NONE &&
           fault != FAULT_COMMUNICATION_LOST;
}

const char* getConnectionStateText(ConnectionState state) {
    switch (state) {
        case WIFI_CONNECTING:
            return "WIFI_CONNECTING";
        case BLYNK_CONNECTING:
            return "BLYNK_CONNECTING";
        case CONNECTION_ONLINE:
            return "ONLINE";
        case RECONNECT_BACKOFF:
            return "BACKOFF";
        default:
            return "OFFLINE";
    }
}

const char* getWifiHealthText(int rssi) {
    if (rssi <= -120) {
        return "OFFLINE";
    }
    if (rssi >= -55) {
        return "EXCELLENT";
    }
    if (rssi >= -67) {
        return "GOOD";
    }
    if (rssi >= -75) {
        return "WEAK";
    }
    return "POOR";
}

const char* getTelemetryReasonText(TelemetryReason reason) {
    switch (reason) {
        case EVENT_STARTUP:
            return "STARTUP";
        case EVENT_CELL_CHANGE:
            return "CELL_CHANGE";
        case EVENT_IMBALANCE_CHANGE:
            return "IMBALANCE_CHANGE";
        case EVENT_RELAY_CHANGE:
            return "RELAY_CHANGE";
        case EVENT_FAULT_CHANGE:
            return "FAULT_CHANGE";
        case EVENT_STATE_CHANGE:
            return "STATE_CHANGE";
        case EVENT_CONNECTION_CHANGE:
            return "CONNECTION_CHANGE";
        case EVENT_RSSI_CHANGE:
            return "RSSI_CHANGE";
        default:
            return "HEARTBEAT";
    }
}

const char* getRiskLevelText(RiskLevel level) {
    switch (level) {
        case RISK_MODERATE:
            return "MODERATE";
        case RISK_HIGH:
            return "HIGH";
        case RISK_CRITICAL:
            return "CRITICAL";
        default:
            return "LOW";
    }
}

const char* getRecommendationText() {
    if (systemStatus.state == SHUTDOWN) {
        return "Manual inspection required";
    }
    if (systemStatus.activeFault == FAULT_RELAY_MISMATCH) {
        return "Inspect relay and feedback";
    }
    if (systemStatus.activeFault == FAULT_ADC_FROZEN ||
        systemStatus.activeFault == FAULT_SENSOR_JUMP ||
        systemStatus.activeFault == FAULT_SENSOR_OUT_OF_RANGE) {
        return "Inspect ADC and cell wiring";
    }
    if (systemStatus.activeFault == FAULT_IMBALANCE) {
        return "Inspect weakest cell and balance pack";
    }
    if (systemStatus.activeFault == FAULT_COMMUNICATION_LOST) {
        return "Restore WiFi and Blynk connection";
    }
    if (analyticsInfo.socAvailable &&
        analyticsInfo.stateOfCharge < 20.0) {
        return "Recharge battery soon";
    }
    if (batteryInfo.imbalanceTrend == IMBALANCE_INCREASING) {
        return "Monitor increasing imbalance";
    }
    return "Battery operating normally";
}

// =====================================================
// READ RAW CELL VALUES
// =====================================================

void readRawCells() {
    for (uint8_t i = 0; i < CELL_COUNT; i++) {
        rawVoltage[i] = readVoltage(CELL_PINS[i]);
    }

    // Software fault injection is applied only to Cell 1.
    if (injectionMode == INJECT_FROZEN) {
        rawVoltage[0] = frozenInjectedValue;
    }
    else if (injectionMode == INJECT_JUMP) {
        jumpToggle = !jumpToggle;

        if (jumpToggle) {
            rawVoltage[0] += 0.80;
        }
        else {
            rawVoltage[0] -= 0.80;
        }
    }
    else if (injectionMode == INJECT_OUT_OF_RANGE) {
        rawVoltage[0] = 4.50;
    }
}

// =====================================================
// GENUINE LOAD-CHANGE DETECTION
// =====================================================

bool isGenuineLoadChange(uint8_t targetCell) {
    float targetChange =
        rawVoltage[targetCell] -
        previousRawVoltage[targetCell];

    if (fabsf(targetChange) <= MAX_SINGLE_SAMPLE_JUMP) {
        return false;
    }

    uint8_t matchingCells = 0;

    for (uint8_t i = 0; i < CELL_COUNT; i++) {
        float change =
            rawVoltage[i] - previousRawVoltage[i];

        bool sameDirection =
            (targetChange > 0.0 && change > 0.0) ||
            (targetChange < 0.0 && change < 0.0);

        bool similarMagnitude =
            fabsf(fabsf(change) - fabsf(targetChange))
            <= LOAD_CHANGE_SIMILARITY;

        if (sameDirection && similarMagnitude) {
            matchingCells++;
        }
    }

    /*
       If at least half of the cells move together in the
       same direction by a similar amount, treat it as a
       genuine rapid pack/load change.
    */
    return matchingCells >= ((CELL_COUNT + 1) / 2);
}

// =====================================================
// SENSOR ANOMALY DETECTION
// =====================================================

void detectSensorAnomalies() {
    sensorHealth.activeFault = FAULT_NONE;
    sensorHealth.faultCellIndex = 0;

    for (uint8_t i = 0; i < CELL_COUNT; i++) {
        sensorHealth.unrealisticJump[i] = false;

        sensorHealth.outOfRange[i] =
            rawVoltage[i] < ADC_MIN_VOLTAGE ||
            rawVoltage[i] > ADC_MAX_VOLTAGE;

        if (!firstSample) {
            float movement =
                fabsf(
                    rawVoltage[i] -
                    previousRawVoltage[i]
                );

            if (movement > MAX_SINGLE_SAMPLE_JUMP &&
                !isGenuineLoadChange(i)) {

                sensorHealth.unrealisticJump[i] = true;
            }
        }
    }

    /*
       Fault priority:
       1. Out-of-range reading
       2. Unrealistic jump
    */
    for (uint8_t i = 0; i < CELL_COUNT; i++) {
        if (sensorHealth.outOfRange[i]) {
            sensorHealth.activeFault =
                FAULT_SENSOR_OUT_OF_RANGE;

            sensorHealth.faultCellIndex = i;
            return;
        }
    }

    for (uint8_t i = 0; i < CELL_COUNT; i++) {
        if (sensorHealth.unrealisticJump[i]) {
            sensorHealth.activeFault =
                FAULT_SENSOR_JUMP;

            sensorHealth.faultCellIndex = i;
            return;
        }
    }

}

// =====================================================
// MODULE 4: FROZEN ADC DETECTION
// =====================================================

void detectFrozenAdc(unsigned long currentTime) {
    adcHealth.frozenCellIndex = 0;

    bool cellMoved[CELL_COUNT];

    for (uint8_t i = 0; i < CELL_COUNT; i++) {
        adcHealth.frozen[i] = false;

        if (firstSample) {
            lastAdcMovementTime[i] = currentTime;
            frozenCandidateStartedAt[i] = 0;
            cellMoved[i] = false;
            continue;
        }

        float movement = fabsf(
            rawVoltage[i] - previousRawVoltage[i]
        );

        cellMoved[i] =
            movement > ADC_MOVEMENT_TOLERANCE;

        if (cellMoved[i]) {
            lastAdcMovementTime[i] = currentTime;
        }
    }

    for (uint8_t i = 0; i < CELL_COUNT; i++) {
        if (cellMoved[i]) {
            frozenCandidateStartedAt[i] = 0;
            continue;
        }

        uint8_t activeOtherCells = 0;

        for (uint8_t j = 0; j < CELL_COUNT; j++) {
            if (j == i) {
                continue;
            }

            if (cellMoved[j]) {
                activeOtherCells++;
            }
        }

        // Require two other cells to remain active continuously. If they
        // settle, the suspected-frozen timer is cancelled immediately.
        if (activeOtherCells < 2) {
            frozenCandidateStartedAt[i] = 0;
            continue;
        }

        if (frozenCandidateStartedAt[i] == 0) {
            frozenCandidateStartedAt[i] = currentTime;
            continue;
        }

        if (currentTime - frozenCandidateStartedAt[i] >=
            ADC_FROZEN_TIMEOUT_MS) {
            adcHealth.frozen[i] = true;
            adcHealth.frozenCellIndex = i;
            return;
        }
    }
}

// =====================================================
// WINDOW-BASED VOLTAGE FILTER
// =====================================================

void updateVoltageFilter() {
    for (uint8_t i = 0; i < CELL_COUNT; i++) {
        bool validSample =
            !sensorHealth.outOfRange[i] &&
            !sensorHealth.unrealisticJump[i];

        if (validSample) {
            voltageHistory[i][historyIndex] =
                rawVoltage[i];
        }
        else if (historyCount > 0) {
            /*
               Reject the abnormal sample and retain the
               previous value in the window.
            */
            uint8_t previousIndex =
                (historyIndex + FILTER_WINDOW_SIZE - 1) %
                FILTER_WINDOW_SIZE;

            voltageHistory[i][historyIndex] =
                voltageHistory[i][previousIndex];
        }
        else {
            voltageHistory[i][historyIndex] = 0.0;
        }
    }

    historyIndex =
        (historyIndex + 1) % FILTER_WINDOW_SIZE;

    if (historyCount < FILTER_WINDOW_SIZE) {
        historyCount++;
    }

    for (uint8_t i = 0; i < CELL_COUNT; i++) {
        float total = 0.0;

        for (uint8_t sample = 0;
             sample < historyCount;
             sample++) {

            total += voltageHistory[i][sample];
        }

        batteryInfo.cellVoltage[i] =
            total / historyCount;
    }
}

// =====================================================
// MODULE 1 CALCULATIONS
// =====================================================

float calculateDischargeRate(
    unsigned long currentTime
) {
    if (firstSample) {
        return 0.0;
    }

    float elapsedSeconds =
        (currentTime - previousSampleTime) /
        1000.0;

    if (elapsedSeconds <= 0.0) {
        return 0.0;
    }

    float rate =
        (previousAverageVoltage -
         batteryInfo.averageVoltage) /
        elapsedSeconds;

    if (rate < 0.0) {
        rate = 0.0;
    }

    return rate;
}

float calculateAdaptiveThreshold(
    float dischargeRate
) {
    return BASE_IMBALANCE_THRESHOLD +
           (DISCHARGE_COMPENSATION * dischargeRate);
}

ImbalanceTrend calculateImbalanceTrend(
    float currentImbalance,
    float lastImbalance
) {
    float change =
        currentImbalance - lastImbalance;

    if (change > TREND_TOLERANCE) {
        return IMBALANCE_INCREASING;
    }

    if (change < -TREND_TOLERANCE) {
        return IMBALANCE_DECREASING;
    }

    return IMBALANCE_STABLE;
}

void analyzeBattery(unsigned long currentTime) {
    batteryInfo.totalVoltage = 0.0;

    batteryInfo.weakestVoltage =
        batteryInfo.cellVoltage[0];

    batteryInfo.strongestVoltage =
        batteryInfo.cellVoltage[0];

    batteryInfo.weakestCellIndex = 0;
    batteryInfo.strongestCellIndex = 0;

    for (uint8_t i = 0; i < CELL_COUNT; i++) {
        float voltage = batteryInfo.cellVoltage[i];

        batteryInfo.totalVoltage += voltage;

        if (voltage < batteryInfo.weakestVoltage) {
            batteryInfo.weakestVoltage = voltage;
            batteryInfo.weakestCellIndex = i;
        }

        if (voltage > batteryInfo.strongestVoltage) {
            batteryInfo.strongestVoltage = voltage;
            batteryInfo.strongestCellIndex = i;
        }
    }

    batteryInfo.averageVoltage =
        batteryInfo.totalVoltage / CELL_COUNT;

    batteryInfo.imbalance =
        batteryInfo.strongestVoltage -
        batteryInfo.weakestVoltage;

    batteryInfo.dischargeRate =
        calculateDischargeRate(currentTime);

    batteryInfo.adaptiveThreshold =
        calculateAdaptiveThreshold(
            batteryInfo.dischargeRate
        );

    batteryInfo.recoveryThreshold =
        batteryInfo.adaptiveThreshold -
        IMBALANCE_HYSTERESIS;

    if (batteryInfo.recoveryThreshold < 0.0) {
        batteryInfo.recoveryThreshold = 0.0;
    }

    if (firstSample) {
        batteryInfo.imbalanceTrend =
            IMBALANCE_STABLE;
    }
    else {
        batteryInfo.imbalanceTrend =
            calculateImbalanceTrend(
                batteryInfo.imbalance,
                batteryInfo.lastImbalance
            );
    }

    batteryInfo.imbalanceExceeded =
        batteryInfo.imbalance >
        batteryInfo.adaptiveThreshold;

    previousAverageVoltage =
        batteryInfo.averageVoltage;

    batteryInfo.lastImbalance =
        batteryInfo.imbalance;

    previousSampleTime = currentTime;
}

// =====================================================
// CLEAN INTERFACES
// =====================================================

const BatteryInfo& getBatteryInfo() {
    return batteryInfo;
}

const SensorHealth& getSensorHealth() {
    return sensorHealth;
}

const ProtectionInfo& getProtectionInfo() {
    return protectionInfo;
}

// =====================================================
// DETERMINE CURRENT PROTECTION FAULT
// =====================================================

FaultId evaluateProtectionFault() {
    if (sensorHealth.activeFault != FAULT_NONE) {
        return sensorHealth.activeFault;
    }

    /*
       Hysteresis:

       When connected, use the higher trip threshold.

       When disconnected or recovering, require imbalance
       to fall below the lower recovery threshold.
    */
    if (protectionInfo.relayState ==
            RELAY_CONNECTED ||
        protectionInfo.relayState ==
            RELAY_TRIP_DEBOUNCE) {

        if (batteryInfo.imbalance >
            batteryInfo.adaptiveThreshold) {

            return FAULT_IMBALANCE;
        }
    }
    else {
        if (batteryInfo.imbalance >
            batteryInfo.recoveryThreshold) {

            return FAULT_IMBALANCE;
        }
    }

    return FAULT_NONE;
}

// =====================================================
// STRUCTURED TRANSITION LOGGING
// =====================================================

void logRelayTransition(
    RelayState previousState,
    RelayState newState,
    FaultId fault,
    unsigned long timestamp
) {
    Serial.print("time=");
    Serial.print(timestamp);

    Serial.print(",event=RELAY_TRANSITION");

    Serial.print(",previous=");
    Serial.print(getRelayStateText(previousState));

    Serial.print(",new=");
    Serial.print(getRelayStateText(newState));

    Serial.print(",fault=");
    Serial.print(getFaultText(fault));

    if (fault == FAULT_SENSOR_JUMP ||
        fault == FAULT_SENSOR_OUT_OF_RANGE) {

        Serial.print(",cell=");
        Serial.print(sensorHealth.faultCellIndex + 1);
    }

    Serial.println();
}

void changeRelayState(
    RelayState newState,
    FaultId fault,
    unsigned long currentTime
) {
    if (newState == protectionInfo.relayState) {
        return;
    }

    RelayState previousState =
        protectionInfo.relayState;

    protectionInfo.relayState = newState;
    protectionInfo.activeFault = fault;
    protectionInfo.stateEnteredAt = currentTime;
    protectionInfo.lastTransitionAt = currentTime;
    protectionInfo.transitionCount++;

    logRelayTransition(
        previousState,
        newState,
        fault,
        currentTime
    );
}

// =====================================================
// NON-BLOCKING RELAY STATE MACHINE
// =====================================================

void updateProtectionSystem(
    unsigned long currentTime
) {
    FaultId currentFault =
        evaluateProtectionFault();

    protectionInfo.activeFault = currentFault;

    switch (protectionInfo.relayState) {
        case RELAY_CONNECTED:
            if (currentFault != FAULT_NONE) {
                changeRelayState(
                    RELAY_TRIP_DEBOUNCE,
                    currentFault,
                    currentTime
                );
            }
            break;

        case RELAY_TRIP_DEBOUNCE:
            if (currentFault == FAULT_NONE) {
                changeRelayState(
                    RELAY_CONNECTED,
                    FAULT_NONE,
                    currentTime
                );
            }
            else if (
                currentTime -
                protectionInfo.stateEnteredAt >=
                TRIP_DEBOUNCE_MS
            ) {
                changeRelayState(
                    RELAY_DISCONNECTED,
                    currentFault,
                    currentTime
                );
            }
            break;

        case RELAY_DISCONNECTED:
            if (currentFault == FAULT_NONE) {
                changeRelayState(
                    RELAY_RECOVERY_WAIT,
                    FAULT_NONE,
                    currentTime
                );
            }
            break;

        case RELAY_RECOVERY_WAIT:
            if (currentFault != FAULT_NONE) {
                changeRelayState(
                    RELAY_DISCONNECTED,
                    currentFault,
                    currentTime
                );
            }
            else if (
                currentTime -
                protectionInfo.stateEnteredAt >=
                RECOVERY_DELAY_MS
            ) {
                changeRelayState(
                    RELAY_CONNECTED,
                    FAULT_NONE,
                    currentTime
                );
            }
            break;
    }

}

// =====================================================
// MODULE 4: FAULT ISOLATION
// =====================================================

FaultId evaluateSystemFault(
    FaultSource& source,
    uint8_t& cellIndex,
    unsigned long currentTime
) {
    bool relayFeedbackDisconnected;

    if (USE_PHYSICAL_RELAY_FEEDBACK) {
        relayFeedbackDisconnected =
            digitalRead(RELAY_FEEDBACK_PIN) == HIGH;
    }
    else {
        relayFeedbackDisconnected = relayCommandDisconnected;
    }

    if (relayMismatchInjection) {
        relayFeedbackDisconnected = !relayCommandDisconnected;
    }

    bool feedbackSettled =
        currentTime - relayCommandChangedAt >=
        RELAY_FEEDBACK_SETTLE_MS;

    if (feedbackSettled &&
        relayFeedbackDisconnected != relayCommandDisconnected) {
        source = SOURCE_RELAY;
        cellIndex = 0;
        return FAULT_RELAY_MISMATCH;
    }

    if (sensorHealth.activeFault ==
        FAULT_SENSOR_OUT_OF_RANGE) {
        source = SOURCE_ADC;
        cellIndex = sensorHealth.faultCellIndex;
        return FAULT_SENSOR_OUT_OF_RANGE;
    }

    if (adcHealth.frozen[adcHealth.frozenCellIndex]) {
        source = SOURCE_ADC;
        cellIndex = adcHealth.frozenCellIndex;
        return FAULT_ADC_FROZEN;
    }

    if (sensorHealth.activeFault == FAULT_SENSOR_JUMP) {
        source = SOURCE_ADC;
        cellIndex = sensorHealth.faultCellIndex;
        return FAULT_SENSOR_JUMP;
    }

    if (protectionInfo.activeFault == FAULT_IMBALANCE) {
        source = SOURCE_BATTERY;
        cellIndex = batteryInfo.weakestCellIndex;
        return FAULT_IMBALANCE;
    }

    if (!Blynk.connected()) {
        source = SOURCE_COMMUNICATION;
        cellIndex = 0;
        return FAULT_COMMUNICATION_LOST;
    }

    source = SOURCE_NONE;
    cellIndex = 0;
    return FAULT_NONE;
}

void logSystemTransition(
    SystemState previousState,
    SystemState newState,
    FaultId fault,
    FaultSource source,
    uint8_t cellIndex,
    unsigned long currentTime
) {
    Serial.print("time=");
    Serial.print(currentTime);
    Serial.print(",event=SYSTEM_STATE_TRANSITION");
    Serial.print(",previous=");
    Serial.print(getSystemStateText(previousState));
    Serial.print(",new=");
    Serial.print(getSystemStateText(newState));
    Serial.print(",fault=");
    Serial.print(getFaultText(fault));
    Serial.print(",source=");
    Serial.print(getFaultSourceText(source));

    if (source == SOURCE_ADC || source == SOURCE_BATTERY) {
        Serial.print(",cell=");
        Serial.print(cellIndex + 1);
    }

    Serial.println();
}

void changeSystemState(
    SystemState newState,
    unsigned long currentTime
) {
    if (newState == systemStatus.state) {
        return;
    }

    SystemState previousState = systemStatus.state;
    systemStatus.state = newState;
    systemStatus.stateEnteredAt = currentTime;
    systemStatus.transitionCount++;

    logSystemTransition(
        previousState,
        newState,
        systemStatus.activeFault,
        systemStatus.activeSource,
        systemStatus.activeCellIndex,
        currentTime
    );
}

void recordFaultHistory(
    FaultId fault,
    FaultSource source,
    uint8_t cellIndex,
    unsigned long currentTime
) {
    FaultHistoryEntry& entry =
        faultHistory[faultHistoryHead];

    entry.timestamp = currentTime;
    entry.fault = fault;
    entry.source = source;
    entry.cellIndex = cellIndex;
    entry.state = systemStatus.state;

    faultHistoryHead =
        (faultHistoryHead + 1) % FAULT_HISTORY_SIZE;

    if (faultHistoryCount < FAULT_HISTORY_SIZE) {
        faultHistoryCount++;
    }

    analyticsInfo.totalFaultCount++;
}

void updateSystemFaultRecord(
    FaultId fault,
    FaultSource source,
    uint8_t cellIndex,
    unsigned long currentTime
) {
    bool newFaultEvent =
        fault != FAULT_NONE &&
        (systemStatus.activeFault == FAULT_NONE ||
         fault != systemStatus.activeFault ||
         source != systemStatus.activeSource);

    if (fault != systemStatus.activeFault ||
        source != systemStatus.activeSource ||
        cellIndex != systemStatus.activeCellIndex) {

        systemStatus.activeFaultDetectedAt = currentTime;
    }

    systemStatus.activeFault = fault;
    systemStatus.activeSource = source;
    systemStatus.activeCellIndex = cellIndex;

    if (fault != FAULT_NONE) {
        systemStatus.lastFault = fault;
        systemStatus.lastSource = source;
        systemStatus.lastFaultCellIndex = cellIndex;
        systemStatus.lastFaultDetectedAt =
            systemStatus.activeFaultDetectedAt;

        if (newFaultEvent) {
            recordFaultHistory(
                fault,
                source,
                cellIndex,
                currentTime
            );
        }
    }
}

void updateAnalytics() {
    float risk = 0.0;

    if (batteryInfo.adaptiveThreshold > 0.0) {
        float imbalanceRatio =
            batteryInfo.imbalance /
            batteryInfo.adaptiveThreshold;

        if (imbalanceRatio > 1.0) {
            imbalanceRatio = 1.0;
        }

        risk += imbalanceRatio * 35.0;
    }

    if (batteryInfo.imbalanceTrend ==
        IMBALANCE_INCREASING) {
        risk += 10.0;
    }

    float frequencyRisk =
        analyticsInfo.totalFaultCount * 3.0;

    if (frequencyRisk > 15.0) {
        frequencyRisk = 15.0;
    }

    risk += frequencyRisk;

    if (analyticsInfo.socAvailable) {
        if (analyticsInfo.stateOfCharge < 20.0) {
            risk += 20.0;
        }
        else if (analyticsInfo.stateOfCharge < 40.0) {
            risk += 10.0;
        }
    }

    switch (systemStatus.state) {
        case DEGRADED:
            risk += 5.0;
            break;
        case FAILSAFE:
            risk += 15.0;
            break;
        case SHUTDOWN:
            risk += 20.0;
            break;
        default:
            break;
    }

    if (risk > 100.0) {
        risk = 100.0;
    }

    analyticsInfo.riskScore = risk;

    if (risk >= 75.0) {
        analyticsInfo.riskLevel = RISK_CRITICAL;
    }
    else if (risk >= 50.0) {
        analyticsInfo.riskLevel = RISK_HIGH;
    }
    else if (risk >= 25.0) {
        analyticsInfo.riskLevel = RISK_MODERATE;
    }
    else {
        analyticsInfo.riskLevel = RISK_LOW;
    }
}

void updateSystemState(unsigned long currentTime) {
    /*
       Deterministic transition table:

       NORMAL   + warning        -> DEGRADED
       NORMAL   + critical fault -> FAILSAFE
       DEGRADED + no fault       -> NORMAL
       DEGRADED + critical fault -> FAILSAFE
       FAILSAFE + fault active   -> FAILSAFE
       FAILSAFE + verified clear -> NORMAL or DEGRADED
       FAILSAFE + relay failure  -> SHUTDOWN after escalation
       SHUTDOWN + safe manual X  -> NORMAL or DEGRADED
    */
    FaultSource source = SOURCE_NONE;
    uint8_t cellIndex = 0;

    FaultId fault = evaluateSystemFault(
        source,
        cellIndex,
        currentTime
    );

    updateSystemFaultRecord(
        fault,
        source,
        cellIndex,
        currentTime
    );

    bool protectionFaultAwaitingDebounce =
        (fault == FAULT_IMBALANCE ||
         fault == FAULT_SENSOR_JUMP ||
         fault == FAULT_SENSOR_OUT_OF_RANGE) &&
        protectionInfo.relayState == RELAY_TRIP_DEBOUNCE;

    bool criticalFault =
        isCriticalFault(fault) &&
        !protectionFaultAwaitingDebounce;

    bool warningFault =
        fault == FAULT_COMMUNICATION_LOST ||
        protectionFaultAwaitingDebounce;

    switch (systemStatus.state) {
        case NORMAL:
            systemStatus.recoveryVerificationActive = false;

            if (criticalFault) {
                changeSystemState(FAILSAFE, currentTime);
            }
            else if (warningFault) {
                changeSystemState(DEGRADED, currentTime);
            }
            break;

        case DEGRADED:
            systemStatus.recoveryVerificationActive = false;

            if (criticalFault) {
                changeSystemState(FAILSAFE, currentTime);
            }
            else if (!warningFault) {
                changeSystemState(NORMAL, currentTime);
            }
            break;

        case FAILSAFE:
            if (criticalFault) {
                systemStatus.recoveryVerificationActive = false;

                bool relayFailurePersistent =
                    fault == FAULT_RELAY_MISMATCH &&
                    currentTime -
                        systemStatus.activeFaultDetectedAt >=
                        SHUTDOWN_ESCALATION_MS;

                if (relayFailurePersistent) {
                    changeSystemState(SHUTDOWN, currentTime);
                }
            }
            else {
                if (!systemStatus.recoveryVerificationActive) {
                    systemStatus.recoveryVerificationActive = true;
                    systemStatus.recoveryVerificationStartedAt =
                        currentTime;

                    Serial.print("time=");
                    Serial.print(currentTime);
                    Serial.println(",event=RECOVERY_VERIFICATION_STARTED");
                }
                else if (
                    currentTime -
                    systemStatus.recoveryVerificationStartedAt >=
                    SYSTEM_RECOVERY_VERIFY_MS
                ) {
                    systemStatus.recoveryVerificationActive = false;

                    changeSystemState(
                        warningFault ? DEGRADED : NORMAL,
                        currentTime
                    );
                }
            }
            break;

        case SHUTDOWN:
            systemStatus.recoveryVerificationActive = false;

            if (shutdownResetRequested && !criticalFault) {
                shutdownResetRequested = false;

                changeSystemState(
                    warningFault ? DEGRADED : NORMAL,
                    currentTime
                );
            }
            break;
    }
}

void applyRelayCommand(unsigned long currentTime) {
    bool protectionRequestsDisconnect =
        protectionInfo.relayState == RELAY_DISCONNECTED ||
        protectionInfo.relayState == RELAY_RECOVERY_WAIT;

    bool systemRequestsDisconnect =
        systemStatus.state == FAILSAFE ||
        systemStatus.state == SHUTDOWN;

    relayCommandDisconnected =
        protectionRequestsDisconnect || systemRequestsDisconnect;

    if (relayCommandDisconnected !=
        lastRelayCommandDisconnected) {
        relayCommandChangedAt = currentTime;
        lastRelayCommandDisconnected =
            relayCommandDisconnected;
    }

    digitalWrite(
        RELAY,
        relayCommandDisconnected
            ? RELAY_DISCONNECTED_LEVEL
            : RELAY_CONNECTED_LEVEL
    );
}

// =====================================================
// INDICATORS
// =====================================================

void updateIndicators() {
    digitalWrite(RED_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BUZZER, LOW);

    if (systemStatus.state == FAILSAFE ||
        systemStatus.state == SHUTDOWN) {

        digitalWrite(RED_LED, HIGH);
        digitalWrite(BUZZER, HIGH);
        return;
    }

    if (systemStatus.state == DEGRADED) {
        digitalWrite(YELLOW_LED, HIGH);
        return;
    }

    switch (protectionInfo.relayState) {
        case RELAY_CONNECTED:
            digitalWrite(GREEN_LED, HIGH);
            break;

        case RELAY_TRIP_DEBOUNCE:
            digitalWrite(YELLOW_LED, HIGH);
            break;

        case RELAY_DISCONNECTED:
            digitalWrite(RED_LED, HIGH);
            digitalWrite(BUZZER, HIGH);
            break;

        case RELAY_RECOVERY_WAIT:
            digitalWrite(YELLOW_LED, HIGH);
            break;
    }
}

// =====================================================
// SERIAL REPORT
// =====================================================

void printSystemReport() {
    if (!PRINT_PERIODIC_REPORT) {
        return;
    }

    Serial.println();
    Serial.print("[BMS t=");
    Serial.print(millis() / 1000UL);
    Serial.println("s]");

    for (uint8_t i = 0; i < CELL_COUNT; i++) {
        Serial.print("C");
        Serial.print(i + 1);
        Serial.print("=");
        Serial.print(batteryInfo.cellVoltage[i], 3);
        Serial.print("V ");
    }

    Serial.print("AVG=");
    Serial.print(batteryInfo.averageVoltage, 3);
    Serial.println(" V");

    Serial.print("WEAK=C");
    Serial.print(batteryInfo.weakestCellIndex + 1);
    Serial.print(" STRONG=C");
    Serial.print(batteryInfo.strongestCellIndex + 1);
    Serial.print(" IMB=");
    Serial.print(batteryInfo.imbalance, 3);
    Serial.print("V TREND=");
    Serial.print(getTrendText(batteryInfo.imbalanceTrend));
    Serial.print(" RATE=");
    Serial.print(batteryInfo.dischargeRate, 4);
    Serial.print("V/s TRIP=");
    Serial.print(batteryInfo.adaptiveThreshold, 3);
    Serial.print("V REC=");
    Serial.print(batteryInfo.recoveryThreshold, 3);
    Serial.println("V");

    Serial.print("FAULT=");
    Serial.print(getFaultText(protectionInfo.activeFault));
    Serial.print(" SOURCE=");
    Serial.print(getFaultSourceText(systemStatus.activeSource));
    Serial.print(" RELAY=");
    Serial.print(getRelayStateText(protectionInfo.relayState));
    Serial.print(" STATE=");
    Serial.print(getSystemStateText(systemStatus.state));
    Serial.print(" VERIFY=");
    Serial.println(systemStatus.recoveryVerificationActive
        ? "ACTIVE" : "INACTIVE");

    Serial.print("WIFI=");
    Serial.print(getConnectionStateText(connectionState));
    Serial.print(" RSSI=");
    Serial.print(getCurrentRssi());
    Serial.print("dBm QUEUE=");
    Serial.print(telemetryQueueCount);
    Serial.print(" RISK=");
    Serial.print(analyticsInfo.riskScore, 1);
    Serial.print("/100 ");
    Serial.print(getRiskLevelText(analyticsInfo.riskLevel));
    Serial.print(" RECOMMENDATION=");
    Serial.println(getRecommendationText());
}

// =====================================================
// MODULE 3: FLICKER-FREE LCD ENGINE
// =====================================================

const char* getRelayShortText(RelayState state) {
    switch (state) {
        case RELAY_CONNECTED:
            return "CONNECTED";

        case RELAY_TRIP_DEBOUNCE:
            return "DEBOUNCE";

        case RELAY_DISCONNECTED:
            return "TRIPPED";

        case RELAY_RECOVERY_WAIT:
            return "RECOVERY";

        default:
            return "UNKNOWN";
    }
}

const char* getFaultShortText(FaultId fault) {
    switch (fault) {
        case FAULT_IMBALANCE:
            return "IMBALANCE";

        case FAULT_SENSOR_JUMP:
            return "SENSOR JUMP";

        case FAULT_SENSOR_OUT_OF_RANGE:
            return "OUT OF RANGE";

        case FAULT_ADC_FROZEN:
            return "ADC FROZEN";

        case FAULT_RELAY_MISMATCH:
            return "RELAY MISMATCH";

        case FAULT_COMMUNICATION_LOST:
            return "COMM LOST";

        default:
            return "NONE";
    }
}

void padLcdLine(char* line) {
    size_t length = strlen(line);

    if (length > 16) {
        length = 16;
    }

    for (size_t i = length; i < 16; i++) {
        line[i] = ' ';
    }

    line[16] = '\0';
}

void writeChangedLcdLine(uint8_t row, const char* newLine) {
    uint8_t column = 0;

    while (column < 16) {
        if (lcdCache[row][column] == newLine[column]) {
            column++;
            continue;
        }

        lcd.setCursor(column, row);

        while (column < 16 &&
               lcdCache[row][column] != newLine[column]) {

            lcd.print(newLine[column]);
            lcdCache[row][column] = newLine[column];
            column++;
        }
    }
}

void renderBatteryPage(char* line1, char* line2) {
    snprintf(
        line1,
        17,
        "AVG:%4.2fV",
        batteryInfo.averageVoltage
    );

    snprintf(
        line2,
        17,
        "IMB:%5.3fV",
        batteryInfo.imbalance
    );
}

void renderSystemPage(char* line1, char* line2) {
    snprintf(
        line1,
        17,
        "STATE:%s",
        getSystemStateText(systemStatus.state)
    );

    snprintf(
        line2,
        17,
        "FAULT:%s",
        getFaultShortText(systemStatus.activeFault)
    );
}

void renderTelemetryPage(char* line1, char* line2) {
    snprintf(
        line1,
        17,
        "RATE:%.3fV/s",
        batteryInfo.dischargeRate
    );

    snprintf(
        line2,
        17,
        "TREND:%s",
        getTrendText(batteryInfo.imbalanceTrend)
    );
}

void renderFaultPage(char* line1, char* line2) {
    snprintf(line1, 17, "!! BMS FAULT !!");

    if (latchedDisplayFault == FAULT_SENSOR_JUMP ||
        latchedDisplayFault == FAULT_SENSOR_OUT_OF_RANGE ||
        latchedDisplayFault == FAULT_ADC_FROZEN) {

        snprintf(
            line2,
            17,
            "%s C%u",
            getFaultShortText(latchedDisplayFault),
            latchedDisplayFaultCell + 1
        );
    }
    else {
        snprintf(
            line2,
            17,
            "%s",
            getFaultShortText(latchedDisplayFault)
        );
    }
}

void initializeLcdCache() {
    for (uint8_t row = 0; row < 2; row++) {
        for (uint8_t column = 0; column < 16; column++) {
            lcdCache[row][column] = '\0';
        }

        lcdCache[row][16] = '\0';
    }
}

void updateLCD(unsigned long currentTime) {
    if (isCriticalFault(systemStatus.activeFault)) {
        faultDisplayLatched = true;
        latchedDisplayFault = systemStatus.activeFault;
        latchedDisplayFaultCell = systemStatus.activeCellIndex;
    }

    bool criticalDisplayOverride =
        faultDisplayLatched || systemStatus.state == SHUTDOWN;

    if (!criticalDisplayOverride &&
        currentTime - previousLcdPageTime >=
            LCD_PAGE_INTERVAL_MS) {

        previousLcdPageTime = currentTime;

        currentLcdPage = static_cast<LcdPage>(
            (static_cast<uint8_t>(currentLcdPage) + 1) % 3
        );
    }

    if (currentTime - previousLcdRefreshTime <
        LCD_REFRESH_INTERVAL_MS) {
        return;
    }

    previousLcdRefreshTime = currentTime;

    char line1[17] = "";
    char line2[17] = "";

    if (criticalDisplayOverride) {
        renderFaultPage(line1, line2);
    }
    else {
        switch (currentLcdPage) {
            case LCD_PAGE_BATTERY:
                renderBatteryPage(line1, line2);
                break;

            case LCD_PAGE_SYSTEM:
                renderSystemPage(line1, line2);
                break;

            case LCD_PAGE_TELEMETRY:
                renderTelemetryPage(line1, line2);
                break;
        }
    }

    padLcdLine(line1);
    padLcdLine(line2);

    writeChangedLcdLine(0, line1);
    writeChangedLcdLine(1, line2);
}

// =====================================================
// MODULE 5: NON-BLOCKING CONNECTIVITY
// =====================================================

void changeConnectionState(
    ConnectionState newState,
    unsigned long currentTime
) {
    if (newState == connectionState) {
        return;
    }

    Serial.print("time=");
    Serial.print(currentTime);
    Serial.print(",event=CONNECTION_TRANSITION,previous=");
    Serial.print(getConnectionStateText(connectionState));
    Serial.print(",new=");
    Serial.println(getConnectionStateText(newState));

    connectionState = newState;
    connectionStateEnteredAt = currentTime;
}

void updateConnectionState(unsigned long currentTime) {
    if (networkOutageInjection) {
        if (connectionState != RECONNECT_BACKOFF) {
            Blynk.disconnect();
            WiFi.disconnect();
            changeConnectionState(RECONNECT_BACKOFF, currentTime);
        }
        return;
    }

    switch (connectionState) {
        case CONNECTION_OFFLINE:
            WiFi.mode(WIFI_STA);
            WiFi.begin(ssid, pass, 6);
            changeConnectionState(WIFI_CONNECTING, currentTime);
            break;

        case WIFI_CONNECTING:
            if (WiFi.status() == WL_CONNECTED) {
                previousBlynkAttemptAt = 0;
                changeConnectionState(BLYNK_CONNECTING, currentTime);
            }
            else if (currentTime - connectionStateEnteredAt >=
                WIFI_CONNECT_TIMEOUT_MS) {
                WiFi.disconnect();
                changeConnectionState(RECONNECT_BACKOFF, currentTime);
            }
            break;

        case BLYNK_CONNECTING:
            if (WiFi.status() != WL_CONNECTED) {
                Blynk.disconnect();
                changeConnectionState(RECONNECT_BACKOFF, currentTime);
            }
            else if (currentTime - previousBlynkAttemptAt >=
                BLYNK_RETRY_INTERVAL_MS) {

                previousBlynkAttemptAt = currentTime;

                // Bounded attempt: never blocks for the default 30 seconds.
                if (Blynk.connect(500)) {
                    changeConnectionState(CONNECTION_ONLINE, currentTime);
                }
                else if (currentTime - connectionStateEnteredAt >=
                    BLYNK_CONNECT_TIMEOUT_MS) {
                    changeConnectionState(RECONNECT_BACKOFF, currentTime);
                }
            }
            break;

        case CONNECTION_ONLINE:
            if (WiFi.status() != WL_CONNECTED ||
                !Blynk.connected()) {
                Blynk.disconnect();
                changeConnectionState(RECONNECT_BACKOFF, currentTime);
            }
            break;

        case RECONNECT_BACKOFF:
            if (currentTime - connectionStateEnteredAt >=
                RECONNECT_BACKOFF_MS) {
                changeConnectionState(CONNECTION_OFFLINE, currentTime);
            }
            break;
    }
}

// =====================================================
// MODULE 5: OFFLINE TELEMETRY QUEUE
// =====================================================

int getCurrentRssi() {
    if (WiFi.status() != WL_CONNECTED) {
        return -127;
    }
    return WiFi.RSSI();
}

TelemetrySnapshot makeTelemetrySnapshot(
    TelemetryReason reason,
    unsigned long currentTime
) {
    TelemetrySnapshot snapshot;
    snapshot.timestamp = currentTime;

    for (uint8_t i = 0; i < CELL_COUNT; i++) {
        snapshot.cellVoltage[i] = batteryInfo.cellVoltage[i];
    }

    snapshot.averageVoltage = batteryInfo.averageVoltage;
    snapshot.imbalance = batteryInfo.imbalance;
    snapshot.adaptiveThreshold = batteryInfo.adaptiveThreshold;
    snapshot.dischargeRate = batteryInfo.dischargeRate;
    snapshot.weakestCell = batteryInfo.weakestCellIndex + 1;
    snapshot.strongestCell = batteryInfo.strongestCellIndex + 1;
    snapshot.relayState = protectionInfo.relayState;
    snapshot.systemState = systemStatus.state;
    snapshot.fault = systemStatus.activeFault;
    snapshot.faultSource = systemStatus.activeSource;
    snapshot.trend = batteryInfo.imbalanceTrend;
    snapshot.rssi = getCurrentRssi();
    snapshot.queueDepth = telemetryQueueCount + 1;
    snapshot.storedOffline =
        connectionState != CONNECTION_ONLINE;
    snapshot.reason = reason;
    snapshot.riskScore = analyticsInfo.riskScore;
    snapshot.riskLevel = analyticsInfo.riskLevel;
    snapshot.totalFaultCount = analyticsInfo.totalFaultCount;
    snapshot.lastFault = systemStatus.lastFault;
    snapshot.lastFaultSource = systemStatus.lastSource;
    snapshot.lastFaultCell = systemStatus.lastFaultCellIndex;
    snapshot.lastFaultTimestamp = systemStatus.lastFaultDetectedAt;
    snapshot.stateOfCharge = analyticsInfo.stateOfCharge;
    snapshot.socAvailable = analyticsInfo.socAvailable;
    snapshot.droppedEvents = droppedTelemetryEvents;

    return snapshot;
}

bool enqueueTelemetryEvent(
    TelemetryReason reason,
    unsigned long currentTime
) {
    if (telemetryQueueCount >= TELEMETRY_QUEUE_SIZE) {
        droppedTelemetryEvents++;

        if (PRINT_QUEUE_WARNINGS &&
            (lastQueueFullLogAt == 0 ||
            currentTime - lastQueueFullLogAt >=
                QUEUE_FULL_LOG_INTERVAL_MS)) {
            lastQueueFullLogAt = currentTime;

            Serial.print("time=");
            Serial.print(currentTime);
            Serial.print(",event=TELEMETRY_QUEUE_FULL,dropped_total=");
            Serial.println(droppedTelemetryEvents);
        }
        return false;
    }

    telemetryQueue[telemetryQueueTail] =
        makeTelemetrySnapshot(reason, currentTime);

    telemetryQueueTail =
        (telemetryQueueTail + 1) % TELEMETRY_QUEUE_SIZE;
    telemetryQueueCount++;

    if (PRINT_TELEMETRY_EVENTS) {
        Serial.print("time=");
        Serial.print(currentTime);
        Serial.print(",event=TELEMETRY_QUEUED,reason=");
        Serial.print(getTelemetryReasonText(reason));
        Serial.print(",offline=");
        Serial.print(
            connectionState == CONNECTION_ONLINE ? 0 : 1
        );
        Serial.print(",depth=");
        Serial.println(telemetryQueueCount);
    }

    return true;
}

void updateTelemetryBaseline(unsigned long currentTime) {
    for (uint8_t i = 0; i < CELL_COUNT; i++) {
        lastEventCellVoltage[i] = batteryInfo.cellVoltage[i];
    }

    lastEventImbalance = batteryInfo.imbalance;
    lastEventWeakestCell = batteryInfo.weakestCellIndex;
    lastEventStrongestCell = batteryInfo.strongestCellIndex;
    lastEventRelayState = protectionInfo.relayState;
    lastEventSystemState = systemStatus.state;
    lastEventFault = systemStatus.activeFault;
    lastEventConnectionState = connectionState;
    lastEventRssi = getCurrentRssi();
    lastTelemetryEventAt = currentTime;
    telemetryBaselineReady = true;
}

void evaluateTelemetryEvents(unsigned long currentTime) {
    if (firstSample) {
        return;
    }

    TelemetryReason reason = EVENT_HEARTBEAT;
    bool meaningfulEvent = false;

    if (!telemetryBaselineReady) {
        reason = EVENT_STARTUP;
        meaningfulEvent = true;
    }
    else if (systemStatus.state != lastEventSystemState) {
        reason = EVENT_STATE_CHANGE;
        meaningfulEvent = true;
    }
    else if (systemStatus.activeFault != lastEventFault) {
        reason = EVENT_FAULT_CHANGE;
        meaningfulEvent = true;
    }
    else if (protectionInfo.relayState != lastEventRelayState) {
        reason = EVENT_RELAY_CHANGE;
        meaningfulEvent = true;
    }
    else if (connectionState != lastEventConnectionState) {
        reason = EVENT_CONNECTION_CHANGE;
        meaningfulEvent = true;
    }
    else if (batteryInfo.weakestCellIndex != lastEventWeakestCell ||
        batteryInfo.strongestCellIndex != lastEventStrongestCell) {
        reason = EVENT_CELL_CHANGE;
        meaningfulEvent = true;
    }
    else {
        for (uint8_t i = 0; i < CELL_COUNT; i++) {
            if (fabsf(
                batteryInfo.cellVoltage[i] -
                lastEventCellVoltage[i]
            ) >= TELEMETRY_CELL_CHANGE) {
                reason = EVENT_CELL_CHANGE;
                meaningfulEvent = true;
                break;
            }
        }
    }

    if (!meaningfulEvent &&
        fabsf(batteryInfo.imbalance - lastEventImbalance) >=
            TELEMETRY_IMBALANCE_CHANGE) {
        reason = EVENT_IMBALANCE_CHANGE;
        meaningfulEvent = true;
    }

    if (!meaningfulEvent &&
        abs(getCurrentRssi() - lastEventRssi) >=
            TELEMETRY_RSSI_CHANGE &&
        (lastRssiEventAt == 0 ||
         currentTime - lastRssiEventAt >=
            RSSI_EVENT_COOLDOWN_MS)) {
        reason = EVENT_RSSI_CHANGE;
        meaningfulEvent = true;
    }

    if (!meaningfulEvent &&
        currentTime - lastTelemetryEventAt >=
            TELEMETRY_HEARTBEAT_MS) {
        reason = EVENT_HEARTBEAT;
        meaningfulEvent = true;
    }

    if (meaningfulEvent) {
        enqueueTelemetryEvent(reason, currentTime);

        if (reason == EVENT_RSSI_CHANGE) {
            lastRssiEventAt = currentTime;
        }

        updateTelemetryBaseline(currentTime);
    }
}

const char* getSnapshotRecommendation(
    const TelemetrySnapshot& snapshot
) {
    if (snapshot.systemState == SHUTDOWN) {
        return "Manual inspection required";
    }
    if (snapshot.fault == FAULT_RELAY_MISMATCH) {
        return "Inspect relay and feedback";
    }
    if (snapshot.fault == FAULT_ADC_FROZEN ||
        snapshot.fault == FAULT_SENSOR_JUMP ||
        snapshot.fault == FAULT_SENSOR_OUT_OF_RANGE) {
        return "Inspect ADC and cell wiring";
    }
    if (snapshot.fault == FAULT_IMBALANCE) {
        return "Inspect weakest cell and balance pack";
    }
    if (snapshot.fault == FAULT_COMMUNICATION_LOST) {
        return "Restore WiFi and Blynk connection";
    }
    if (snapshot.socAvailable && snapshot.stateOfCharge < 20.0) {
        return "Recharge battery soon";
    }
    if (snapshot.trend == IMBALANCE_INCREASING) {
        return "Monitor increasing imbalance";
    }
    return "Battery operating normally";
}

constexpr uint8_t TELEMETRY_FIELD_COUNT = 15;

void sendTelemetryField(
    const TelemetrySnapshot& snapshot,
    uint8_t field
) {
    switch (field) {
        case 0:  Blynk.virtualWrite(V0, snapshot.averageVoltage); break;
        case 1:  Blynk.virtualWrite(V1, snapshot.cellVoltage[0]); break;
        case 2:  Blynk.virtualWrite(V2, snapshot.cellVoltage[1]); break;
        case 3:  Blynk.virtualWrite(V3, snapshot.cellVoltage[2]); break;
        case 4: {
            char statusText[120];
            snprintf(
                statusText,
                sizeof(statusText),
                "relay=%s,wifi=%s,rssi=%d,queue=%u,data=%s",
                getRelayStateText(snapshot.relayState),
                getWifiHealthText(snapshot.rssi),
                snapshot.rssi,
                snapshot.queueDepth,
                snapshot.storedOffline ? "QUEUED" : "LIVE"
            );
            Blynk.virtualWrite(V4, statusText);
            break;
        }
        case 5:  Blynk.virtualWrite(V5, snapshot.imbalance); break;
        case 6:  Blynk.virtualWrite(V6, snapshot.cellVoltage[3]); break;
        case 7:  Blynk.virtualWrite(V7, snapshot.weakestCell); break;
        case 8:  Blynk.virtualWrite(V8, snapshot.strongestCell); break;
        case 9:  Blynk.virtualWrite(V9, snapshot.adaptiveThreshold); break;
        case 10: Blynk.virtualWrite(V10, getTrendText(snapshot.trend)); break;
        case 11: Blynk.virtualWrite(V11, snapshot.dischargeRate); break;
        case 12: Blynk.virtualWrite(V12, getFaultText(snapshot.fault)); break;
        case 13: {
            char executiveText[120];
            snprintf(
                executiveText,
                sizeof(executiveText),
                "state=%s,risk=%.1f,level=%s,uptime=%lus,faults=%lu",
                getSystemStateText(snapshot.systemState),
                snapshot.riskScore,
                getRiskLevelText(snapshot.riskLevel),
                snapshot.timestamp / 1000UL,
                static_cast<unsigned long>(snapshot.totalFaultCount)
            );
            Blynk.virtualWrite(V13, executiveText);
            break;
        }
        case 14: {
            char faultSummary[180];
            snprintf(
                faultSummary,
                sizeof(faultSummary),
                "source=%s,last=%s,t_s=%lu,cell=%u,recommendation=%s",
                getFaultSourceText(snapshot.faultSource),
                getFaultText(snapshot.lastFault),
                snapshot.lastFaultTimestamp/1000UL,
                snapshot.lastFaultCell + 1,
                getSnapshotRecommendation(snapshot)
            );
            Blynk.virtualWrite(V14, faultSummary);
            break;
        }
    }
}

void processTelemetryTransmission(unsigned long currentTime) {
    if (connectionState != CONNECTION_ONLINE ||
        !Blynk.connected()) {
        telemetryTransmissionActive = false;
        telemetryFieldIndex = 0;
        return;
    }

    if (telemetryQueueCount == 0) {
        telemetryTransmissionActive = false;
        return;
    }

    if (!telemetryTransmissionActive) {
        telemetryTransmissionActive = true;
        telemetryFieldIndex = 0;
        previousTelemetryFieldAt = currentTime;
        return;
    }

    if (currentTime - previousTelemetryFieldAt <
        TELEMETRY_FIELD_INTERVAL_MS) {
        return;
    }

    previousTelemetryFieldAt = currentTime;

    sendTelemetryField(
        telemetryQueue[telemetryQueueHead],
        telemetryFieldIndex
    );

    telemetryFieldIndex++;

    if (telemetryFieldIndex >= TELEMETRY_FIELD_COUNT) {
        telemetryQueueHead =
            (telemetryQueueHead + 1) % TELEMETRY_QUEUE_SIZE;
        telemetryQueueCount--;

        if (PRINT_TELEMETRY_EVENTS) {
            Serial.print("time=");
            Serial.print(currentTime);
            Serial.print(",event=TELEMETRY_SENT,remaining=");
            Serial.println(telemetryQueueCount);
        }

        telemetryTransmissionActive = false;
        telemetryFieldIndex = 0;
    }
}

// =====================================================
// PER-SAMPLE PROCESSING
// =====================================================

void processBatterySample(
    unsigned long currentTime
) {
    readRawCells();
    detectSensorAnomalies();
    detectFrozenAdc(currentTime);
    updateVoltageFilter();
    analyzeBattery(currentTime);
    updateProtectionSystem(currentTime);
    updateSystemState(currentTime);
    updateAnalytics();
    applyRelayCommand(currentTime);
    updateIndicators();

    printSystemReport();

    for (uint8_t i = 0; i < CELL_COUNT; i++) {
        previousRawVoltage[i] = rawVoltage[i];
    }

    firstSample = false;
}

// =====================================================
// SETUP
// =====================================================

void setup() {
    Serial.begin(115200);

    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(YELLOW_LED, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(RELAY, OUTPUT);

    if (USE_PHYSICAL_RELAY_FEEDBACK) {
        pinMode(RELAY_FEEDBACK_PIN, INPUT_PULLUP);
    }

    digitalWrite(
        RELAY,
        RELAY_CONNECTED_LEVEL
    );

    analogReadResolution(12);

    lcd.init();
    lcd.backlight();

    lcd.setCursor(0, 0);
    lcd.print("Starting BMS");

    protectionInfo.relayState =
        RELAY_CONNECTED;

    protectionInfo.activeFault =
        FAULT_NONE;

    protectionInfo.stateEnteredAt = millis();
    protectionInfo.lastTransitionAt = millis();
    protectionInfo.transitionCount = 0;

    systemStatus.state = NORMAL;
    systemStatus.activeFault = FAULT_NONE;
    systemStatus.activeSource = SOURCE_NONE;
    systemStatus.activeCellIndex = 0;
    systemStatus.activeFaultDetectedAt = millis();
    systemStatus.lastFault = FAULT_NONE;
    systemStatus.lastSource = SOURCE_NONE;
    systemStatus.lastFaultCellIndex = 0;
    systemStatus.lastFaultDetectedAt = 0;
    systemStatus.stateEnteredAt = millis();
    systemStatus.transitionCount = 0;
    systemStatus.recoveryVerificationActive = false;
    systemStatus.recoveryVerificationStartedAt = 0;

    relayCommandDisconnected = false;
    lastRelayCommandDisconnected = false;
    relayCommandChangedAt = millis();

    analyticsInfo.riskScore = 0.0;
    analyticsInfo.riskLevel = RISK_LOW;
    analyticsInfo.totalFaultCount = 0;
    analyticsInfo.stateOfCharge = 0.0;
    analyticsInfo.socAvailable = false;

    for (uint8_t i = 0; i < CELL_COUNT; i++) {
        lastAdcMovementTime[i] = millis();
        frozenCandidateStartedAt[i] = 0;
        adcHealth.frozen[i] = false;
    }

    connectionState = CONNECTION_OFFLINE;
    connectionStateEnteredAt = millis();

    WiFi.mode(WIFI_STA);
    Blynk.config(BLYNK_AUTH_TOKEN);

    initializeLcdCache();
    previousLcdPageTime = millis();

    Serial.println(
        "All 6 BMS modules started"
    );

    Serial.println(
        "Fault commands:"
    );

    Serial.println(
        "F = frozen ADC Cell 1"
    );

    Serial.println(
        "J = jumping Cell 1"
    );

    Serial.println(
        "R = out-of-range Cell 1"
    );

    Serial.println(
        "C = clear injection"
    );

    Serial.println(
        "M = relay feedback mismatch"
    );

    Serial.println(
        "N = network outage"
    );

    Serial.println(
        "X = request SHUTDOWN reset"
    );

    Serial.println(
        "A = acknowledge cleared LCD fault"
    );

}

// =====================================================
// LOOP
// =====================================================

void loop() {
    handleSerialCommands();

    unsigned long currentTime = millis();

    updateConnectionState(currentTime);

    if (connectionState == CONNECTION_ONLINE &&
        Blynk.connected()) {
        Blynk.run();
    }

    updateLCD(currentTime);

    if (
        currentTime - previousLoopSampleTime >=
        SAMPLE_INTERVAL_MS
    ) {
        previousLoopSampleTime = currentTime;

        processBatterySample(currentTime);
        evaluateTelemetryEvents(currentTime);
    }

    processTelemetryTransmission(currentTime);
}
