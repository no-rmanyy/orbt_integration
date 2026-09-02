//////////////////////////////////////////////////////////////////
//
// ORBT Sensor Node -- reduced firmware
//
// A trimmed-down version of orbt-core-firmware, keeping only:
//   - WiFi (station mode)
//   - BLE (joystick controller client, via NimBLE)
//   - BLE ranging initiator (continuous scan alongside the controller)
//   - UDP Telemetry (CBOR or Teleplot, switchable via TELEMETRY_MODE)
//   - IMU + Sensor Fusion (IMS module: ICM42688 + Madgwick/VQF)
//   - Status LED (battery level + charging indicator)
//
// All motor control, offset-mass, audio, ESP-NOW, drive-state machine, etc.
// have been removed. Use this as a starting point for new bot/board
// experiments that only need sensing + comms.
//
// Board: ESP32-C6, ORBT PCB Mk5.0 only.
//
// BLE RADIO OWNERSHIP (read before touching setup() ordering):
//   - BLE_Client_Joystick::begin() is the ONLY place NimBLEDevice::init()
//     is called. Never init NimBLE anywhere else.
//   - initiatorBegin() must run AFTER bleJoystick.begin().
//   - If ESP-NOW is reintroduced later, init it LAST, after all BLE setup --
//     NimBLE init resets radio/channel state that ESP-NOW depends on.
//
//////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <cmath>
#include "esp_timer.h"
#include <esp_wifi.h>

#include "config.h"
#include "config_pins.h"
#include "config_wifi.h"
#include "utils/simple-udp.h"
#include "utils/logging.h"
#include "utils/imu_helper.h"
#include "utils/initiator.h"

#include "Telemetry.h"
#include "TeleplotTelemetry.h"
#include "CborTelemetry.h"

#include "IMS/ims.h"

#include "OrbtLED.h"
#include "StatusLed/StatusLed.h"
#include "PowerManagement/PowerManagement.h"
#include "PowerManagement/PM_PCB_Mk5_0.h"
#include "SingleShotADCManager.h"

#define TELEMETRY_EVERY_USECS               20000   // Log ALL Telemetry every usecs.  Can run up to 10000, depending on volume of UDP calls

// Ranging target: the XIAO responder advertising this name.
// Must match DEV_NAME in the responder sketch, and the name must be in the
// ADVERTISING payload (not the scan response) -- the ranging scan is passive.

// Serial-only bring-up: WiFi is not connected, so telemetry is disabled and
// the coexistence metrics are printed to serial instead. Set to 0 once WiFi
// telemetry is working again.
#define SERIAL_DIAGNOSTICS                  0

#if BLE_ENABLE
#include "BLE_Client_Joystick.h"

// Defined in BLE_Client_Joystick.cpp; set whenever a joystick connects. This
// declaration MUST stay at global scope (not inside the anonymous namespace
// below) -- an `extern` inside an anonymous namespace declares a distinct,
// per-translation-unit symbol and will not bind to the real global one,
// causing an "undefined reference to debugBLEClient" link error.
extern NimBLEClient* debugBLEClient;
#endif

namespace {

constexpr uint32_t kWifiRetryDelayMs = 500;
constexpr uint8_t kWifiMaxRetries = 20;
constexpr uint32_t kTelemetryIntervalMs = 20;   // ~50Hz telemetry rate
constexpr uint32_t kPowerMgmtIntervalMs = 100;  // 10Hz battery/charge sampling
constexpr uint32_t kDiagIntervalMs = 1000;      // 1Hz serial coexistence report

Telemetry* telemetry = nullptr;
bool wifiConnected = false;
uint64_t lastTelemetryMs = 0;
uint64_t lastPowerMgmtMs = 0;
uint64_t lastDiagMs = 0;

// IMS / IMU (background task samples the IMU + runs sensor fusion)
IMS ims;
LegacyIMU IMU;

// Status LED: blue while charging, colour-coded by battery level otherwise
OrbtLED orbtLed(static_cast<gpio_num_t>(PIN_COB_LEDS), 1);
StatusLed statusLed(&orbtLed);

// Allocated in setup() (after Arduino init()), not as static globals -- their
// constructors touch pinMode()/GPIO config, which shouldn't run during
// static initialization.
SingleShotADCManager* adcManager = nullptr;
PM_PCB_Mk5_0* powerManagement = nullptr;

#if BLE_ENABLE
BLE_Client_Joystick bleJoystick;

// Latest joystick state, updated from BLE callbacks.
// Field names match the original controllerMovement(x, y, z, rx, ry, rz, s1):
// x/trigger and y/wheel are the primary drive axes, rx/ry are auxiliary axes
// (e.g. a second stick), z/rz/s1 are exposed too in case your controller uses them.
volatile int16_t joyTriggerX = 0;   // x
volatile int16_t joyWheelY = 0;     // y
volatile int16_t joyZ = 0;          // z
volatile int16_t joyAux1 = 0;       // rx
volatile int16_t joyAux2 = 0;       // ry
volatile int16_t joyRz = 0;         // rz
volatile int16_t joyS1 = 0;         // s1
volatile bool joyConnected = false;
volatile bool joyButtonA = false;
volatile bool joyButtonB = false;
volatile bool joyButtonC = false;
volatile bool joyButtonD = false;

#if PAIRING_VISUALISATION_ENABLE
// Pairing is triggered by holding the power button continuously from boot
// for 2 seconds (same gesture as the full firmware) -- see checkBlePairingTrigger().
uint64_t startupTimeUs = 0;
bool buttonReleasedSinceBoot = false;
bool blePairingTriggered = false;
bool isPairingModeActive = false;
#endif

void onJoystickConnect(bool connected) {
    joyConnected = connected;
    Serial.printf("BLE Joystick %s\n", connected ? "connected" : "disconnected");
}

void onJoystickMovement(int x, int y, int z, int rx, int ry, int rz, int s1) {
    joyTriggerX = x;
    joyWheelY = y;
    joyZ = z;
    joyAux1 = rx;
    joyAux2 = ry;
    joyRz = rz;
    joyS1 = s1;
}

void onJoystickButtonA(bool pressed) { joyButtonA = pressed; }
void onJoystickButtonB(bool pressed) { joyButtonB = pressed; }
void onJoystickButtonC(bool pressed) { joyButtonC = pressed; }
void onJoystickButtonD(bool pressed) { joyButtonD = pressed; }

#if PAIRING_VISUALISATION_ENABLE
// If the power button has been held continuously since boot (never seen
// released) for more than 2 seconds, clear BLE pairing and enter pairing
// mode -- same gesture/logic as the full firmware's power-button handling.
void checkBlePairingTrigger() {
    bool pressed = (digitalRead(PIN_POWER_BUTTON) == POWER_BUTTON_ACTIVE_STATE);
    if (!pressed) {
        buttonReleasedSinceBoot = true;
    }

    if (!buttonReleasedSinceBoot && !blePairingTriggered && pressed &&
        (esp_timer_get_time() - startupTimeUs) > 2000000) {
        blePairingTriggered = true;
        isPairingModeActive = true;

        bleJoystick.clearPairing();
        LOG_INFO("BLE Pairing Attempted.");
    }

    // Pairing visualisation ends once a joystick actually connects
    if (isPairingModeActive && debugBLEClient && debugBLEClient->isConnected()) {
        isPairingModeActive = false;
    }
}
#endif  // PAIRING_VISUALISATION_ENABLE
#endif  // BLE_ENABLE

bool connectWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, true);
    WiFi.begin(WIFI_SSID, WIFI_SECRET);

    Serial.printf("Connecting to WiFi SSID: %s\n", WIFI_SSID);

    uint8_t retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < kWifiMaxRetries) {
        delay(kWifiRetryDelayMs);
        Serial.print(".");
        retries++;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("WiFi connected. IP: ");
        Serial.println(WiFi.localIP());
        return true;
    }

    Serial.println("WiFi connection failed.");
    return false;
}

// Never forward NaN/Inf onto the wire -- a diverged sensor-fusion result (or
// any other bad float) becomes illegible garbage in the telemetry stream
// otherwise. Substitute 0 and keep going.
double safeMetric(double value) {
    return std::isfinite(value) ? value : 0.0;
}

// Map the current PowerManagement state onto the status LED. Charging is
// always blue (breathing while charging, solid once full); otherwise the LED
// colour reflects battery level (red/yellow/green, flashing red if critical).
void updateStatusLed() {
#if BLE_ENABLE && PAIRING_VISUALISATION_ENABLE
    if (isPairingModeActive) {
        statusLed.setStatus(StatusLed::STATUS_BLE_PAIRING);
        return;
    }
#endif

    if (!powerManagement) {
        return;
    }

    if (powerManagement->getChargeState() == PowerManagement::CHARGE_STATE_CHARGING) {
        statusLed.setStatus(StatusLed::STATUS_PM_CHARGING);
        return;
    }
    if (powerManagement->getChargeState() == PowerManagement::CHARGE_STATE_CHARGING_FULL) {
        statusLed.setStatus(StatusLed::STATUS_PM_CHARGING_FULL);
        return;
    }

    switch (powerManagement->getBatteryLevel()) {
        case PowerManagement::BATTERY_LEVEL_CRITICAL:
            statusLed.setStatus(StatusLed::STATUS_PM_BATTERY_CRITICAL);
            break;
        case PowerManagement::BATTERY_LEVEL_LOW:
            statusLed.setStatus(StatusLed::STATUS_PM_BATTERY_LOW);
            break;
        case PowerManagement::BATTERY_LEVEL_MEDIUM:
            statusLed.setStatus(StatusLed::STATUS_PM_BATTERY_MEDIUM);
            break;
        case PowerManagement::BATTERY_LEVEL_HIGH:
            statusLed.setStatus(StatusLed::STATUS_PM_BATTERY_HIGH);
            break;
        case PowerManagement::BATTERY_LEVEL_INIT:
        default:
            // Haven't sampled the ADC enough yet to know battery level; leave
            // the LED as-is (set to STATUS_POWER_ON at boot).
            break;
    }
}

#if SERIAL_DIAGNOSTICS
// Serial-only coexistence report. This is the whole experiment: watch pps
// (ranging packets/sec) against connInt (controller connection interval).
//
//   connInt should sit at 16 (16 * 1.25ms = 20ms), matching
//   setConnectionParams(16, 16, 0, 51) in BLE_Client_Joystick.cpp.
//   If it drifts upward, or the joystick feels laggy, the scan is starving
//   the controller -- reduce RANGE_WINDOW_MS in initiator.cpp.
void printDiagnostics() {
#if BLE_ENABLE
    if (debugBLEClient && debugBLEClient->isConnected()) {
        NimBLEConnInfo ci = debugBLEClient->getConnInfo();
        Serial.printf("[diag] connInt=%u ctrlRssi=%d | ranging=%d pps=%.1f n=%u rssi=%.1f dist=%.3f\n",
                      ci.getConnInterval(),
                      debugBLEClient->getRssi(),
                      initiatorRanging() ? 1 : 0,
                      initiatorPacketRate(),
                      initiatorPacketCount(),
                      initiatorRssi(),
                      initiatorDistanceM());
    } else {
        Serial.println("[diag] controller disconnected - discovery scan, ranging idle");
    }
#endif
}
#endif  // SERIAL_DIAGNOSTICS

void sendTelemetry() {
    if (!telemetry) {
        return;
    }

    IMSComputeResult result;
    bool haveImu = true;//.ims.getLatestComputeResult(result);

    telemetry->beginFrame();

    telemetry->output_metric("Heap", (uint64_t)ESP.getFreeHeap());
    telemetry->output_metric("WiFi_RSSI", wifiConnected ? WiFi.RSSI() : 0);

    if (haveImu) {
        telemetry->output_metric("AccelX", safeMetric(result.accX), "MPS^2");
        telemetry->output_metric("AccelY", safeMetric(result.accY), "MPS^2");
        telemetry->output_metric("AccelZ", safeMetric(result.accZ), "MPS^2");
        telemetry->output_metric("GyroX", safeMetric(result.gyrX), "DPS");
        telemetry->output_metric("GyroY", safeMetric(result.gyrY), "DPS");
        telemetry->output_metric("GyroZ", safeMetric(result.gyrZ), "DPS");

#if IMU_SENSOR_FUSION_MODE != IMU_SENSOR_FUSION_MODE_OFF
        EulerDeg euler = result.eulerOrientation();
        telemetry->output_metric("Roll", safeMetric(euler.roll), "Degrees");
        telemetry->output_metric("Pitch", safeMetric(euler.pitch), "Degrees");
        telemetry->output_metric("Yaw", safeMetric(euler.yaw), "Degrees");
#endif
    }

#if BLE_ENABLE
    // telemetry->output_metric("Joy_Connected", joyConnected ? 1 : 0);
    // telemetry->output_metric("Joy_TriggerX", joyTriggerX);
    // telemetry->output_metric("Joy_WheelY", joyWheelY);
    // telemetry->output_metric("Joy_Z", joyZ);
    // telemetry->output_metric("Joy_Aux1", joyAux1);
    // telemetry->output_metric("Joy_Aux2", joyAux2);
    // telemetry->output_metric("Joy_Rz", joyRz);
    // telemetry->output_metric("Joy_S1", joyS1);
    // telemetry->output_metric("Joy_ButtonA", joyButtonA ? 1 : 0);
    // telemetry->output_metric("Joy_ButtonB", joyButtonB ? 1 : 0);
    // telemetry->output_metric("Joy_ButtonC", joyButtonC ? 1 : 0);
    // telemetry->output_metric("Joy_ButtonD", joyButtonD ? 1 : 0);

    if (debugBLEClient && debugBLEClient->isConnected()) {
        telemetry->output_metric("BLE_RSSI", debugBLEClient->getRssi(), "dBm");

        NimBLEConnInfo connInfo = debugBLEClient->getConnInfo();
        telemetry->output_metric("BLE_ConnInterval", connInfo.getConnInterval(), "1.25ms");
        telemetry->output_metric("BLE_ConnTimeout", connInfo.getConnTimeout(), "10ms");
        telemetry->output_metric("BLE_ConnLatency", connInfo.getConnLatency(), "intervals");
    }

    // Ranging metrics
    telemetry->output_metric("Range_D1", initiatorDistanceM(0), "m");
    telemetry->output_metric("Range_D2", initiatorDistanceM(1), "m");
    telemetry->output_metric("Range_D3", initiatorDistanceM(2), "m");
    telemetry->output_metric("Range_N1", initiatorPacketCount(0));
    telemetry->output_metric("Range_N2", initiatorPacketCount(1));
    telemetry->output_metric("Range_N3", initiatorPacketCount(2));
    telemetry->output_metric("Range_PPS", initiatorPacketRate(), "pkt/s");
#endif

    if (powerManagement) {
        powerManagement->reportTelemetry(telemetry);
    }

    telemetry->sendFrame();
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(STARTUP_DEBUG_DELAY);
    Serial.println();
    Serial.println("ORBT Sensor Node -- reduced firmware starting up");
    Serial.print("Robot STA MAC: ");
    Serial.println(WiFi.macAddress());

#if BLE_ENABLE && PAIRING_VISUALISATION_ENABLE
    startupTimeUs = esp_timer_get_time();
    pinMode(PIN_POWER_BUTTON, INPUT);
#endif

    // WiFi + UDP telemetry
    //
    // Serial-only bring-up: WiFi stays down entirely. A failed connect
    // (connectWifi() -> fail -> WiFi.disconnect()) disturbs the radio, which
    // is exactly what we don't want while measuring BLE coexistence. Leave
    // both the connect and the disconnect out until telemetry is revisited.
    //
    // wifiConnected = connectWifi();
    if (wifiConnected) {
        simple_udp_init();

        // Same selection the original firmware uses: TELEMETRY_MODE is set in
        // include/config/config_telemetry.h (defaults to TELEMETRY_MODE_CBOR).
#if TELEMETRY_MODE == TELEMETRY_MODE_TELEPLOT
        telemetry = new CborTelemetry(WIFI_TELEMTRY_HOST, 1202, CborTelemetry::ConnectionType::UDP);
#else
        telemetry = new TeleplotTelemetry(WIFI_TELEMTRY_HOST, WIFI_TELEMTRY_PORT);
#endif

        LOG_INFO("WiFi + Telemetry ready. IP: %s", WiFi.localIP().toString().c_str());
    }

    // IMU + Sensor Fusion
    if (!ims.begin()) {
        Serial.println("IMS failed to start. Check IMU wiring.");
    }
    ims.registerLegacy(&IMU);

    // Status LED + battery/charge monitoring
    orbtLed.begin();
    statusLed.begin();
    statusLed.setStatus(StatusLed::STATUS_POWER_ON);

    adcManager = new SingleShotADCManager();
    powerManagement = new PM_PCB_Mk5_0(adcManager);
    powerManagement->begin();
    adcManager->start();

    // BLE Joystick controller (NimBLE central, connects to a paired gamepad).
    // This is the ONLY NimBLEDevice::init() in the firmware.
#if BLE_ENABLE
    bleJoystick.set_connect_callback(onJoystickConnect);
    bleJoystick.set_movement_callback(onJoystickMovement);
    bleJoystick.set_button_callback(0, onJoystickButtonA);
    bleJoystick.set_button_callback(1, onJoystickButtonB);
    bleJoystick.set_button_callback(2, onJoystickButtonC);
    bleJoystick.set_button_callback(3, onJoystickButtonD);
    bleJoystick.begin();

    // Ranging initiator -- MUST be after bleJoystick.begin() (needs NimBLE up).
    // Arms only; the scan stays in discovery mode until the controller
    // connects, then switches itself to ranging parameters.
    initiatorBegin();
#endif

    lastTelemetryMs = millis();
    lastPowerMgmtMs = lastTelemetryMs;
    lastDiagMs = lastTelemetryMs;
    Serial.println("Setup complete.");
}

void loop() {
    // Pull the latest IMU + sensor-fusion result out of the background tasks
    ims.update();

#if BLE_ENABLE
    bleJoystick.loop();
    initiatorUpdate();
#if PAIRING_VISUALISATION_ENABLE
    checkBlePairingTrigger();
#endif
#endif

    const uint64_t nowUs = esp_timer_get_time();
    orbtLed.handle(nowUs);

    const uint64_t now = millis();

    if (now - lastPowerMgmtMs >= kPowerMgmtIntervalMs) {
        lastPowerMgmtMs = now;
        if (adcManager) {
            adcManager->handle();
        }
        if (powerManagement) {
            powerManagement->handle(nowUs);
        }
        updateStatusLed();
    }

    if (now - lastTelemetryMs >= kTelemetryIntervalMs) {
        lastTelemetryMs = now;
        sendTelemetry();
    }

#if SERIAL_DIAGNOSTICS
    if (now - lastDiagMs >= kDiagIntervalMs) {
        lastDiagMs = now;
        printDiagnostics();
    }
#endif

    delay(1);
}
