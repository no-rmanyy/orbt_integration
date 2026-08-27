#pragma once

#include <Arduino.h>
#include "config_pins.h"
#include "PowerManagement/PowerManagement.h"
#include "OrbtADCManager.h"

// Thresholds values of battery level, and charging/charged states
//
// Used to determine if USB-C is connected (and we're charging or charged)
// Used to set LED colour battery level indicator
// Used to trigger low-battery shutdown
// !! May vary slightly depending on battery used
#define CHARGE_STATE_CHARGED_THRESHOLD      0.5f  // below this, we assume charger chip is signalling FULL
#define CHARGE_STATE_BATTERY_MIN            1.5f  // the min value we'd even consider it a battery level
#define CHARGE_STATE_BATTERY_MAX            4.3f  // the max value we'd even consider it a battery level
#define CHARGE_STATE_CHARGING_THRESHOLD     4.5f  // ABOVE this assume USB-C is connected and CHARGING

#define BATTERY_LEVEL_CRITICAL_THRESHOLD    3.35f // once we fall from LOW to CRITICAL, we force initiate shutdown
#define BATTERY_LEVEL_LOW_THRESHOLD         3.50f // boundary between LOW and MEDIUM
#define BATTERY_LEVEL_MEDIUM_THRESHOLD      3.65f // boundary between MEDIUM and HIGH

#define BATTERY_LEVEL_CHANGE_SCHMITT_TRIGGER 0.05f  // require this extra voltage diff to acknowledge a change in battery voltage level

#define ADC_BATTERY_LEVEL_SMOOTHING_FACTOR  0.05f         // how much we low-pass-filter / smooth ADC


class PM_PCB_Mk5_0 : public PowerManagement {
public:
    PM_PCB_Mk5_0(OrbtADCManager* adcManager);
    ~PM_PCB_Mk5_0() = default;

    void begin() override;
    void handle(uint64_t currentTime_us) override;
    void reportTelemetry(Telemetry *telemetry) override;
private:
    OrbtADCManager* _adcManager = nullptr;
    uint32_t _adcbatteryLevelInput_mV = 0;
    uint32_t _adcBatteryNTCInput_mV = 0;
    uint32_t _adcChargerStatusInput_mV = 0;

    double _chargerStatusSmoothed_mV = 0;
};