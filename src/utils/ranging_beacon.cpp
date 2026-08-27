#include <Arduino.h>
#include "utils/ranging_beacon.h"
#include <NimBLEDevice.h>

// Advertising interval in 0.625ms units. Conservative here (30ms) because
// this board also holds a latency-sensitive joystick connection + WiFi
// telemetry on the one radio; too-fast advertising can starve those.
static const uint16_t ADV_INTERVAL_UNITS = 48;   // 30 ms

void rangingBeaconBegin(const char *devName) {
  // NimBLE is ALREADY initialised by the joystick client. Do NOT call
  // NimBLEDevice::init() again - that would tear down the joystick
  // connection. Reuse the existing instance.

  NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();

  NimBLEAdvertisementData advData;
  advData.setName(devName);
  advData.setFlags(0x06);
  pAdv->setAdvertisementData(advData);

  pAdv->setMinInterval(ADV_INTERVAL_UNITS);
  pAdv->setMaxInterval(ADV_INTERVAL_UNITS);

  bool ok = pAdv->start();
  Serial.printf("Ranging beacon '%s' advertising: %s (%.1fms)\n",
                devName, ok ? "OK" : "FAILED", ADV_INTERVAL_UNITS * 0.625);
}