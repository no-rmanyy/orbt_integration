#include <Arduino.h>
#include <NimBLEDevice.h>
#include <string.h>
#include <math.h>
#include "utils/initiator.h"

// Defined in BLE_Client_Joystick.cpp. Used to detect controller connection
// state so the scan can switch between discovery and ranging parameters.
extern NimBLEClient *debugBLEClient;

// ---------------------------------------------------------------------------
// TUNABLES
// ---------------------------------------------------------------------------

// Beacon names. Index here == anchor_id - 1.
static const char *TARGET_NAMES[RANGING_N_TARGETS] = {
    "orbt_ble1", "orbt_ble2", "orbt_ble3"
};

// BRING-UP ONLY: run ranging without a controller connected.
// Set to false once the gamepad is in the loop.
static const bool FORCE_RANGING = true;

// Ranging scan duty cycle. NimBLE 2.x takes MILLISECONDS (1.x used 0.625ms).
// The controller only needs ~10% of the radio (2ms every 20ms), so 80/100 is
// reasonable. If BLE_ConnInterval drifts above 16, back the window off.
static const uint16_t RANGE_INTERVAL_MS = 100;
static const uint16_t RANGE_WINDOW_MS   = 80;

// Discovery params -- must match BLE_Client_Joystick::begin() so gamepad
// discovery behaves exactly as before when the controller is disconnected.
static const uint16_t DISC_INTERVAL_MS = 45;
static const uint16_t DISC_WINDOW_MS   = 15;

static const uint32_t WINDOW_MS = 200;    // reporting window -> 5 Hz

// RSSI -> distance log fit: rssi = A*ln(d_cm) + B   (NimBLE-calibrated values)
// Shared across beacons. Per-anchor FIT_A/FIT_B is more accurate if hardware
// varies, but shared values work functionally.
static const float FIT_A       = -15.75f;
static const float FIT_B       = 3.1501f;
static const float MIN_DIST_CM = 2.0f;
static const float MAX_DIST_CM = 70.0f;   // 60cm field

// 1D Kalman on RSSI. Rate term deliberately absent (biggest lag reducer).
static const float KF_Q = 0.5f;
static const float KF_R = 4.0f;
static const bool  WEIGHT_R_BY_N = true;  // R = KF_R / n : distrust sparse windows

static const bool     SERIAL_PRINT = true;
static const uint32_t PRINT_MS     = 200;

// ---------------------------------------------------------------------------
// STATE
// ---------------------------------------------------------------------------

struct BeaconState {
    // accumulator (written from the NimBLE task)
    int32_t  rssiSum;
    uint16_t rssiN;

    // latched address: after the first name match, compare addresses instead
    // of strings -- cheaper in the callback, and immune to the name being
    // absent from later adverts.
    NimBLEAddress addr;
    bool          haveAddr;

    // Kalman + output
    float    kfX, kfP;
    bool     kfInit;
    float    rssiFilt;
    float    distM;
    uint16_t lastN;
};

static BeaconState g_b[RANGING_N_TARGETS];

static bool g_begun   = false;
static bool g_ranging = false;

// The accumulator is touched from the NimBLE task and from loop().
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

static bool     g_allValid    = false;
static uint32_t g_windowStart = 0;
static uint32_t g_lastPrint   = 0;
static uint32_t g_rateStart   = 0;
static uint32_t g_rateCount   = 0;
static float    g_pktRate     = 0.0f;

static void resetBeacons() {
    for (int i = 0; i < RANGING_N_TARGETS; i++) {
        g_b[i].rssiSum  = 0;
        g_b[i].rssiN    = 0;
        g_b[i].haveAddr = false;
        g_b[i].kfX      = 0.0f;
        g_b[i].kfP      = 1.0f;
        g_b[i].kfInit   = false;
        g_b[i].rssiFilt = 0.0f;
        g_b[i].distM    = -1.0f;
        g_b[i].lastN    = 0;
    }
}

// ---------------------------------------------------------------------------
// FILTER
// ---------------------------------------------------------------------------

static float kalmanStep(BeaconState &b, float z, float R) {
    if (!b.kfInit) {                 // seed to first measurement, not 0
        b.kfX    = z;
        b.kfP    = R;
        b.kfInit = true;
        return b.kfX;
    }
    b.kfP += KF_Q;                   // predict (no rate term)
    const float S = b.kfP + R;
    const float K = b.kfP / S;
    b.kfX += K * (z - b.kfX);        // update
    b.kfP  = (1.0f - K) * b.kfP;
    return b.kfX;
}

static float rssiToDistM(float rssi) {
    float cm = expf((rssi - FIT_B) / FIT_A);
    if (cm < MIN_DIST_CM) cm = MIN_DIST_CM;
    if (cm > MAX_DIST_CM) cm = MAX_DIST_CM;
    return cm * 0.01f;
}

// ---------------------------------------------------------------------------
// SCAN PARAMETER SWITCHING
// ---------------------------------------------------------------------------
//
// The joystick stops the scan the instant it connects and does not restart it
// until onDisconnect. So the two roles never need the scan simultaneously:
//   disconnected -> joystick owns it (discovery params, active, dup filter on)
//   connected    -> ranging owns it (ranging params, passive, dup filter off)

static void applyRangingParams() {
    NimBLEScan *s = NimBLEDevice::getScan();
    if (!s) return;

    if (s->isScanning()) s->stop();

    s->setInterval(RANGE_INTERVAL_MS);
    s->setWindow(RANGE_WINDOW_MS);
    s->setActiveScan(false);        // passive: no SCAN_REQ = less airtime
    s->setDuplicateFilter(false);   // ESSENTIAL: else one hit then silence

    // NOTE: deliberately NOT calling setMaxResults(0). BLE_Client_Joystick
    // stores a raw `const NimBLEAdvertisedDevice* advDevice` and dereferences
    // it in connectToServer(); that pointer is only kept alive by the results
    // vector. Disabling results would dangle it and corrupt the heap on
    // connect. Instead the vector is cleared each window in initiatorUpdate().

    s->start(0, false, true);       // duration 0 = forever
    Serial.printf("[initiator] ranging scan %u/%u ms passive, %d targets\n",
                  RANGE_INTERVAL_MS, RANGE_WINDOW_MS, RANGING_N_TARGETS);
}

static void applyDiscoveryParams() {
    NimBLEScan *s = NimBLEDevice::getScan();
    if (!s) return;

    if (s->isScanning()) s->stop();

    s->setInterval(DISC_INTERVAL_MS);
    s->setWindow(DISC_WINDOW_MS);
    s->setActiveScan(true);         // joystick needs scan responses
    s->setDuplicateFilter(true);
    s->clearResults();

    s->start(0, false, true);
    Serial.println("[initiator] discovery scan restored");
}

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

bool initiatorBegin() {
    if (!NimBLEDevice::isInitialized()) {
        Serial.println("[initiator] NimBLE not initialised - call AFTER bleJoystick.begin()");
        return false;                       // NEVER init here
    }

    resetBeacons();

    g_windowStart = millis();
    g_rateStart   = g_windowStart;
    g_begun       = true;

    Serial.print("[initiator] armed, targets:");
    for (int i = 0; i < RANGING_N_TARGETS; i++) {
        Serial.printf(" %s", TARGET_NAMES[i]);
    }
    Serial.println();
    return true;
}

// Called from BLE_Client_Joystick's onResult(), on the NimBLE task.
// Keep this short -- it runs in a callback.
bool initiatorOnAdvert(const NimBLEAdvertisedDevice *dev) {
    if (!g_begun || !g_ranging || dev == nullptr) return false;

    int idx = -1;

    // Fast path: address already latched for this beacon.
    for (int i = 0; i < RANGING_N_TARGETS; i++) {
        if (g_b[i].haveAddr && dev->getAddress() == g_b[i].addr) { idx = i; break; }
    }

    // Slow path: match by advertised name, then latch the address.
    if (idx < 0) {
        if (!dev->haveName()) return false;
        const char *nm = dev->getName().c_str();
        for (int i = 0; i < RANGING_N_TARGETS; i++) {
            if (!g_b[i].haveAddr && strcmp(nm, TARGET_NAMES[i]) == 0) {
                g_b[i].addr     = dev->getAddress();
                g_b[i].haveAddr = true;
                idx = i;
                break;
            }
        }
        if (idx < 0) return false;
    }

    const int rssi = dev->getRSSI();
    portENTER_CRITICAL(&g_mux);
    g_b[idx].rssiSum = g_b[idx].rssiSum + rssi;
    g_b[idx].rssiN   = g_b[idx].rssiN + 1;
    portEXIT_CRITICAL(&g_mux);
    return true;
}

void initiatorUpdate() {
    if (!g_begun) return;

    const uint32_t now = millis();

    // --- controller connection state edge detection ---
    const bool connected = FORCE_RANGING ||
                           (debugBLEClient != nullptr && debugBLEClient->isConnected());
    if (connected != g_ranging) {
        g_ranging = connected;
        if (connected) {
            applyRangingParams();
            g_windowStart = now;
            g_rateStart   = now;
            g_rateCount   = 0;
        } else {
            applyDiscoveryParams();
            resetBeacons();             // re-latch and re-seed on next session
            g_pktRate  = 0.0f;
            g_allValid = false;
        }
    }

    if (!g_ranging) return;             // joystick owns the scan while disconnected

    // Watchdog: only restart while ranging, so we never stomp on the
    // joystick's stop()-then-connect sequence.
    NimBLEScan *s = NimBLEDevice::getScan();
    if (s && !s->isScanning()) {
        s->start(0, false, true);
    }

    if (now - g_windowStart < WINDOW_MS) return;
    g_windowStart = now;

    // Safe here: ranging only runs once the controller is up, so the
    // joystick is not holding advDevice into the results vector.
    if (s) s->clearResults();

    // Snapshot and clear all accumulators in one critical section.
    int32_t  sum[RANGING_N_TARGETS];
    uint16_t n[RANGING_N_TARGETS];
    portENTER_CRITICAL(&g_mux);
    for (int i = 0; i < RANGING_N_TARGETS; i++) {
        sum[i] = g_b[i].rssiSum;
        n[i]   = g_b[i].rssiN;
        g_b[i].rssiSum = 0;
        g_b[i].rssiN   = 0;
    }
    portEXIT_CRITICAL(&g_mux);

    g_allValid = true;
    for (int i = 0; i < RANGING_N_TARGETS; i++) {
        g_b[i].lastN = n[i];
        g_rateCount  = g_rateCount + n[i];

        if (n[i] > 0) {
            const float avg = (float)sum[i] / (float)n[i];
            const float R   = WEIGHT_R_BY_N ? (KF_R / (float)n[i]) : KF_R;
            g_b[i].rssiFilt = kalmanStep(g_b[i], avg, R);  // filter BEFORE log conversion
            g_b[i].distM    = rssiToDistM(g_b[i].rssiFilt);
        } else {
            g_allValid = false;     // hold last distance, do not blank it
        }
    }

    if (now - g_rateStart >= 1000) {
        g_pktRate   = (float)g_rateCount * 1000.0f / (float)(now - g_rateStart);
        g_rateStart = now;
        g_rateCount = 0;
    }

    if (SERIAL_PRINT && (now - g_lastPrint >= PRINT_MS)) {
        g_lastPrint = now;
        // One line: per-beacon n and distance, then total packet rate.
        for (int i = 0; i < RANGING_N_TARGETS; i++) {
            Serial.printf("b%d n=%-2u ", i + 1, g_b[i].lastN);
            if (g_b[i].distM >= 0.0f) Serial.printf("d=%4.2fm  ", g_b[i].distM);
            else                      Serial.print("d= --    ");
        }
        Serial.printf("| pps=%4.1f\n", g_pktRate);
    }
}

float initiatorDistanceM(uint8_t idx) {
    return (idx < RANGING_N_TARGETS) ? g_b[idx].distM : -1.0f;
}
float initiatorRssi(uint8_t idx) {
    return (idx < RANGING_N_TARGETS) ? g_b[idx].rssiFilt : 0.0f;
}
uint16_t initiatorPacketCount(uint8_t idx) {
    return (idx < RANGING_N_TARGETS) ? g_b[idx].lastN : 0;
}
bool  initiatorAllValid()   { return g_allValid; }
float initiatorPacketRate() { return g_pktRate; }
bool  initiatorRanging()    { return g_ranging; }