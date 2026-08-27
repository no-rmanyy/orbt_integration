#pragma once

#include <Arduino.h>
#include "OrbtADCManager.h"

class SingleShotADCManager : public OrbtADCManager {
protected:
    uint8_t adc_channel_count = 0;

public:
    SingleShotADCManager();

    void start() override;
    void handle() override;
    void addPin(uint8_t pin) override;

    uint16_t getPinValue(uint8_t pin) override;
    uint32_t getPinValueMilliVolts(uint8_t pin) override;
    bool hasNewPinValue(uint8_t pin) override;
};