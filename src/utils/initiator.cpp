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

// Ranging scan duty cycle. NimBLE 2.x takes MILLISECONDS (1.x used 0.625ms).
// The controller only needs ~10% of the radio (2ms every 20ms), so 80/100 is
// reasonable. If BLE_ConnInterval starts drifting above 16, back the window off.
static const uint16_t RANGE_INTERVAL_MS = 100;
static const uint16_t RANGE_WINDOW_MS   = 80;

// Discovery params -- must match BLE_Client_Joystick::begin() so gamepad
// discovery behaves exactly as before when the controller is disconnected.
static const uint16_t DISC_INTERVAL_MS = 45;
static const uint16_t DISC_WINDOW_MS   = 15;

static const uint32_t WINDOW_MS = 200;    // reporting window -> 5 Hz

// RSSI -> distance log fit: rssi = A*ln(d_cm) + B   (NimBLE-calibrated values)
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

static const bool FORCE_RANGING = true;

// ---------------------------------------------------------------------------
// STATE
// ---------------------------------------------------------------------------

static char g_targetName[32] = {0};
static bool g_begun    = false;
static bool g_ranging  = false;   // true once controller is connected

// Written from the NimBLE task, read from loop() -> guard with a spinlock.
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static int32_t  g_rssiSum = 0;
static uint16_t g_rssiN   = 0;

// Latched target address: after the first name match, compare addresses
// (cheaper, and immune to the name being absent from later adverts).
static bool          g_haveAddr = false;
static NimBLEAddress g_targetAddr;

static float    g_rssiFilt = 0.0f;
static float    g_distM    = -1.0f;
static uint16_t g_lastN    = 0;

static float g_kfX = 0.0f, g_kfP = 1.0f;
static bool  g_kfInit = false;

static uint32_t g_windowStart = 0;
static uint32_t g_lastPrint   = 0;
static uint32_t g_rateStart   = 0;
static uint32_t g_rateCount   = 0;
static float    g_pktRate     = 0.0f;

// ---------------------------------------------------------------------------
// FILTER
// ---------------------------------------------------------------------------

static float kalmanStep(float z, float R) {
    if (!g_kfInit) {                 // seed to first measurement, not 0
        g_kfX    = z;
        g_kfP    = R;
        g_kfInit = true;
        return g_kfX;
    }
    g_kfP += KF_Q;                   // predict (no rate term)
    const float S = g_kfP + R;
    const float K = g_kfP / S;
    g_kfX += K * (z - g_kfX);        // update
    g_kfP  = (1.0f - K) * g_kfP;
    return g_kfX;
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
    // connect. Instead the vector is cleared each window in initiatorUpdate(),
    // which is safe because that only runs while already connected.

    s->start(0, false, true);       // duration 0 = forever
    Serial.printf("[initiator] ranging scan %u/%u ms passive\n",
                  RANGE_INTERVAL_MS, RANGE_WINDOW_MS);
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

bool initiatorBegin(const char *targetName) {
    if (!NimBLEDevice::isInitialized()) {
        Serial.println("[initiator] NimBLE not initialised - call AFTER bleJoystick.begin()");
        return false;                       // NEVER init here
    }

    strncpy(g_targetName, targetName, sizeof(g_targetName) - 1);

    g_windowStart = millis();
    g_rateStart   = g_windowStart;
    g_begun       = true;

    Serial.printf("[initiator] armed, target=\"%s\" (ranging starts when controller connects)\n",
                  g_targetName);
    return true;
}

// Called from BLE_Client_Joystick's onResult(), on the NimBLE task.
// Keep this short -- it runs in a callback.
bool initiatorOnAdvert(const NimBLEAdvertisedDevice *dev) {
    if (!g_begun || !g_ranging || dev == nullptr) return false;

    if (g_haveAddr) {
        if (dev->getAddress() != g_targetAddr) return false;
    } else {
        if (!dev->haveName()) return false;
        if (strcmp(dev->getName().c_str(), g_targetName) != 0) return false;
        g_targetAddr = dev->getAddress();
        g_haveAddr   = true;
    }

    const int rssi = dev->getRSSI();
    portENTER_CRITICAL(&g_mux);
    g_rssiSum = g_rssiSum + rssi;
    g_rssiN   = g_rssiN + 1;
    portEXIT_CRITICAL(&g_mux);
    return true;
}

void initiatorUpdate() {
    if (!g_begun) return;

    const uint32_t now = millis();

    // --- controller connection state edge detection ---
    const bool connected = FORCE_RANGING || (debugBLEClient != nullptr && debugBLEClient->isConnected());
    if (connected != g_ranging) {
        g_ranging = connected;
        if (connected) {
            applyRangingParams();
            g_windowStart = now;
            g_rateStart   = now;
            g_rateCount   = 0;
        } else {
            applyDiscoveryParams();
            g_haveAddr = false;         // re-latch target on next session
            g_kfInit   = false;         // re-seed the filter
            g_lastN    = 0;
            g_pktRate  = 0.0f;
        }
    }

    if (!g_ranging) return;             // joystick owns the scan while disconnected

    // Watchdog: only restart while connected, so we never stomp on the
    // joystick's stop()-then-connect sequence.
    NimBLEScan *s = NimBLEDevice::getScan();
    if (s && !s->isScanning()) {
        s->start(0, false, true);
    }

    if (now - g_windowStart < WINDOW_MS) return;
    g_windowStart = now;

    // Safe here: we are connected, so advDevice is not in use by the joystick.
    if (s) s->clearResults();

    portENTER_CRITICAL(&g_mux);
    const int32_t  sum = g_rssiSum;
    const uint16_t n   = g_rssiN;
    g_rssiSum = 0;
    g_rssiN   = 0;
    portEXIT_CRITICAL(&g_mux);

    g_lastN     = n;
    g_rateCount = g_rateCount + n;

    if (n > 0) {
        const float avg = (float)sum / (float)n;
        const float R   = WEIGHT_R_BY_N ? (KF_R / (float)n) : KF_R;
        g_rssiFilt = kalmanStep(avg, R);     // filter BEFORE the log conversion
        g_distM    = rssiToDistM(g_rssiFilt);
    }
    // n == 0: hold the last value, do not blank it.

    if (now - g_rateStart >= 1000) {
        g_pktRate   = (float)g_rateCount * 1000.0f / (float)(now - g_rateStart);
        g_rateStart = now;
        g_rateCount = 0;
    }

    if (SERIAL_PRINT && (now - g_lastPrint >= PRINT_MS)) {
        g_lastPrint = now;
        Serial.printf("n=%-2u  rssi=%6.1f dBm  dist=%5.2f m  pps=%4.1f\n",
                      g_lastN, g_rssiFilt, g_distM, g_pktRate);
    }
}

float    initiatorDistanceM()   { return g_distM; }
float    initiatorRssi()        { return g_rssiFilt; }
uint16_t initiatorPacketCount() { return g_lastN; }
bool     initiatorHasFix()      { return g_lastN > 0; }
float    initiatorPacketRate()  { return g_pktRate; }
bool     initiatorRanging()     { return g_ranging; }
