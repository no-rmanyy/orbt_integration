#pragma once

#include <Arduino.h>
#include <OrbtLED.h>

// A trimmed-down copy of the full firmware's StatusLed: same colour/effect
// mapping for power, BLE pairing, and battery/charging states, but without
// the drive-state machine or ESC-passthrough states that don't exist in this
// reduced project. Call setStatus() directly whenever PowerManagement's
// charge state / battery level, or BLE pairing mode, changes.
class StatusLed
{
public:
    StatusLed(OrbtLED *orbtLeds);
    ~StatusLed(void);

    void begin(void);

    void setEnabled(bool enabled);

    enum Status {
        STATUS_POWER_OFF,
        STATUS_POWER_ON,
        STATUS_BLE_PAIRING,
        STATUS_PM_BATTERY_CRITICAL,
        STATUS_PM_BATTERY_LOW,
        STATUS_PM_BATTERY_MEDIUM,
        STATUS_PM_BATTERY_HIGH,
        STATUS_PM_CHARGING,
        STATUS_PM_CHARGING_FULL,
    };

    void setStatus(Status status);

protected:
    OrbtLED *_orbtLeds;
    Status _status;
    bool _enabled;
};
