#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

// ---------------------------------------------------------------------------
// Robot-side BLE ranging initiator -- 3 targets (orbt_ble1/2/3).
//
// One passive scan serves all three beacons. BLE adverts are broadcasts, so
// a single scan window receives whichever beacons happen to transmit during
// it; the airtime cost is identical to the 1-to-1 case. Packets are sorted
// per-beacon in software by advertised name (then latched address).
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

#define RANGING_N_TARGETS 3

// Beacon names are compiled in (see TARGET_NAMES in initiator.cpp).
// Index 0/1/2 correspond to orbt_ble1/2/3 -- this mapping must match the
// physical anchor layout when trilateration is added, or the position comes
// out geometrically scrambled even with perfect distances.
bool initiatorBegin();

// Call from BLE_Client_Joystick's onResult(), as the FIRST line:
//     initiatorOnAdvert(advertisedDevice);
// Returns true if this advert was a ranging target (caller may return early).
bool initiatorOnAdvert(const NimBLEAdvertisedDevice *dev);

// Call every loop() iteration, after bleJoystick.loop(). Non-blocking.
void initiatorUpdate();

// Per-beacon accessors. idx is 0..RANGING_N_TARGETS-1.
float    initiatorDistanceM(uint8_t idx);    // filtered distance, m; -1 if never seen
float    initiatorRssi(uint8_t idx);         // filtered RSSI, dBm
uint16_t initiatorPacketCount(uint8_t idx);  // n in the most recent window

bool  initiatorAllValid();     // true when all three had n > 0 this window
float initiatorPacketRate();   // total packets/sec across all beacons
bool  initiatorRanging();      // true when in ranging mode