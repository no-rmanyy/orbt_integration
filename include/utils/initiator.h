#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

// ---------------------------------------------------------------------------
// Robot-side BLE ranging initiator.
//
// DESIGN NOTE -- why this module does NOT own the scan callbacks:
//   NimBLEDevice::getScan() is a SINGLETON. BLE_Client_Joystick::begin()
//   already calls setScanCallbacks(new AdvertisedDeviceCallbacks(), true).
//   That `true` transfers ownership, so a second setScanCallbacks() call
//   would DELETE the joystick's callback object and permanently break
//   gamepad discovery.
//
//   Instead: BLE_Client_Joystick's onResult() calls initiatorOnAdvert()
//   as its first line. One callback object, two jobs.
//
// GOLDEN RULES:
//   - Never calls NimBLEDevice::init(). The joystick client owns that.
//   - Call initiatorBegin() AFTER bleJoystick.begin().
//   - If ESP-NOW is added later, init it LAST, after all BLE setup.
// ---------------------------------------------------------------------------

// targetName must match the responder's ADVERTISED name (not scan response) --
// the ranging scan is passive and never requests scan responses.
bool initiatorBegin(const char *targetName);

// Call from BLE_Client_Joystick's onResult(), as the FIRST line:
//     initiatorOnAdvert(advertisedDevice);
// Returns true if this advert was the ranging target (caller may return early).
bool initiatorOnAdvert(const NimBLEAdvertisedDevice *dev);

// Call every loop() iteration, after bleJoystick.loop(). Non-blocking.
void initiatorUpdate();

float    initiatorDistanceM();     // filtered distance, metres
float    initiatorRssi();          // filtered RSSI, dBm
uint16_t initiatorPacketCount();   // n in the most recent window
bool     initiatorHasFix();        // last window had n > 0
float    initiatorPacketRate();    // packets/sec, rolling -- coexistence health
bool     initiatorRanging();       // true when in ranging mode (controller up)
