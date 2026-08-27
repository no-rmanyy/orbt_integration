#include "StatusLed/StatusLed.h"

#define STATUS_LED_ID 0

StatusLed::StatusLed(OrbtLED *orbtLeds)
{
    _orbtLeds = orbtLeds;
    _status = STATUS_POWER_OFF;
    _enabled = true;
}

StatusLed::~StatusLed(void)
{
}

void StatusLed::begin(void)
{
    // Half Brightness
    _orbtLeds->setBrightness(STATUS_LED_ID, 512);
}

void StatusLed::setEnabled(bool enabled)
{
    _enabled = enabled;

    if (enabled) {
        Status prevStatus = _status;
        _status = STATUS_POWER_OFF;

        // Restore the previous status
        setStatus(prevStatus);
    } else {
        // Force the Led off
        _orbtLeds->setLedSolid(STATUS_LED_ID, {0, 0, 0, 0});
    }
}

void StatusLed::setStatus(Status status)
{
    // Avoid Busy Work
    if (_status == status) {
        return;
    }

    _status = status;

    if (!_enabled) {
        return;
    }

    switch (_status) {
        case STATUS_POWER_OFF:
            _orbtLeds->setLedSolid(STATUS_LED_ID, {0, 0, 0, 0});  // Off
            break;
        case STATUS_POWER_ON:
            _orbtLeds->setLedSolid(STATUS_LED_ID, {0, 0, 0, 255}); // White
            break;
        case STATUS_BLE_PAIRING:
            _orbtLeds->setLedFlash(STATUS_LED_ID, {0, 127, 127, 0}, 200); // Fast Flash Cyan - 100ms on / 100ms off
            break;
        case STATUS_PM_BATTERY_CRITICAL:
            _orbtLeds->setLedFlash(STATUS_LED_ID, {255, 0, 0, 0}, 200); // Fast Flash Red - 100ms on / 100ms off
            break;
        case STATUS_PM_BATTERY_LOW:
            _orbtLeds->setLedSolid(STATUS_LED_ID, {255, 0, 0, 0}); // Red
            break;
        case STATUS_PM_BATTERY_MEDIUM:
            _orbtLeds->setLedSolid(STATUS_LED_ID, {255, 255, 0, 0}); // Yellow
            break;
        case STATUS_PM_BATTERY_HIGH:
            _orbtLeds->setLedSolid(STATUS_LED_ID, {0, 255, 0, 0}); // Green
            break;
        case STATUS_PM_CHARGING:
            _orbtLeds->setLedBreath(STATUS_LED_ID, {0, 0, 255, 0}, 1720); // Breath Blue - 860ms inhale / 860ms exhale
            break;
        case STATUS_PM_CHARGING_FULL:
            _orbtLeds->setLedSolid(STATUS_LED_ID, {0, 0, 255, 0}); // Blue
            break;
    }
}
