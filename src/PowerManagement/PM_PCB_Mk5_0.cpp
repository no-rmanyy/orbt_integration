#include <Arduino.h>
#include "PowerManagement/PM_PCB_Mk5_0.h"
#include "utils/logging.h"

PM_PCB_Mk5_0::PM_PCB_Mk5_0(OrbtADCManager* adcManager) : _adcManager(adcManager)
{
    adcManager->addPin(PIN_ADC_POWER_LEVEL);
    adcManager->addPin(PIN_ADC_BATTERY_NTC);
    adcManager->addPin(PIN_ADC_CHARGER_STATUS);
}

void PM_PCB_Mk5_0::begin()
{
}

void PM_PCB_Mk5_0::handle(uint64_t currentTime_us)
{
    static uint64_t lastCurrentTime_us = 0;
    uint64_t timeDelta_us = currentTime_us - lastCurrentTime_us;
    lastCurrentTime_us = currentTime_us;

    // keep track of changes in any of these key power/battery variables
    _prevBatteryLevel = _batteryLevel;
    _prevChargeState = _chargeState;

    // read analog voltage value from battery level sensor
    //_adcbatteryLevelInput = _adcManager->getPinValue(PIN_ADC_POWER_LEVEL);
    // _batteryVoltage = _adcbatteryLevelInput * ADC_VOLTAGE_MULTIPLIER;

    // Use the calibrated value instead of the raw value
    _adcbatteryLevelInput_mV = _adcManager->getPinValueMilliVolts(PIN_ADC_POWER_LEVEL);
    _batteryVoltage = _adcbatteryLevelInput_mV * BATTERY_VOLTAGE_SCALE / 1000.0;

    _adcBatteryNTCInput_mV = _adcManager->getPinValueMilliVolts(PIN_ADC_BATTERY_NTC);
    _adcChargerStatusInput_mV = _adcManager->getPinValueMilliVolts(PIN_ADC_CHARGER_STATUS);

    // 5 second smoothing of charger status
    double timeConstant = 1 - exp(-(double)timeDelta_us / (1.0*1000*1000));
    _chargerStatusSmoothed_mV = (timeConstant * _adcChargerStatusInput_mV) + ((1 - timeConstant) * _chargerStatusSmoothed_mV);

    // battery disconnected: low 257 ish - high 500 ish. -- 500ms period
    // done -- Measured 257  -- no wave
    // low voltage charger - low 257 ish - high 500 ish - 5 second period

    // _chargerStatusSmoothed

    if(_chargerStatusSmoothed_mV < 300) {  // theory 250mv - was raw 300
        // Done - finished charging or abormal
        // USB Connected
        _chargeState = CHARGE_STATE_CHARGING_FULL;
    } else if(_chargerStatusSmoothed_mV < 600) { // theory 460mv - was raw 600 - Measured 480
        // Charging or abnormal
        // USB Connected
        _chargeState = CHARGE_STATE_CHARGING;
    } else if(_chargerStatusSmoothed_mV > 1000) { // theory 3300mv - was raw 1000 - Discharging
        // On Battery
        _chargeState = CHARGE_STATE_BATTERY;
    }

    if (_batteryVoltageFiltered != -1)
    {
        _batteryVoltageFiltered = (ADC_BATTERY_LEVEL_SMOOTHING_FACTOR * _batteryVoltage) + ((1 - ADC_BATTERY_LEVEL_SMOOTHING_FACTOR) * _batteryVoltageFiltered);
    }
    else
    {
        _batteryVoltageFiltered = _batteryVoltage;  // start at exact first input value, if we have no history yet
    }

    // set or update battery level based on init voltage
    if ((_batteryVoltageFiltered != -1) && (_batteryVoltageFiltered > CHARGE_STATE_CHARGED_THRESHOLD)
        && (_batteryVoltageFiltered < CHARGE_STATE_CHARGING_THRESHOLD))
    {
        if (_batteryLevel == BATTERY_LEVEL_INIT)
        {
            if (_batteryVoltageFiltered > CHARGE_STATE_CHARGED_THRESHOLD)
                _batteryLevel = BATTERY_LEVEL_CRITICAL;
            if (_batteryVoltageFiltered > BATTERY_LEVEL_CRITICAL_THRESHOLD)
                _batteryLevel = BATTERY_LEVEL_LOW;
            if (_batteryVoltageFiltered > BATTERY_LEVEL_LOW_THRESHOLD)
                _batteryLevel = BATTERY_LEVEL_MEDIUM;
            if (_batteryVoltageFiltered > BATTERY_LEVEL_MEDIUM_THRESHOLD)
                _batteryLevel = BATTERY_LEVEL_HIGH;
        }
        // Schmitt triggers, at specific rise/fall voltages, all changes to batteryLevel after first init
        else if (_batteryLevel == BATTERY_LEVEL_CRITICAL)
        {
            if (_batteryVoltageFiltered > (BATTERY_LEVEL_CRITICAL_THRESHOLD + BATTERY_LEVEL_CHANGE_SCHMITT_TRIGGER))
                _batteryLevel = BATTERY_LEVEL_LOW;
            if (_batteryVoltageFiltered > (BATTERY_LEVEL_LOW_THRESHOLD + BATTERY_LEVEL_CHANGE_SCHMITT_TRIGGER))
                _batteryLevel = BATTERY_LEVEL_MEDIUM;
            if (_batteryVoltageFiltered > (BATTERY_LEVEL_MEDIUM_THRESHOLD + BATTERY_LEVEL_CHANGE_SCHMITT_TRIGGER))
                _batteryLevel = BATTERY_LEVEL_HIGH;
        }
        else if (_batteryLevel == BATTERY_LEVEL_LOW)
        {
            if (_batteryVoltageFiltered < (BATTERY_LEVEL_CRITICAL_THRESHOLD - BATTERY_LEVEL_CHANGE_SCHMITT_TRIGGER))
                _batteryLevel = BATTERY_LEVEL_CRITICAL;
            else if (_batteryVoltageFiltered > (BATTERY_LEVEL_LOW_THRESHOLD + BATTERY_LEVEL_CHANGE_SCHMITT_TRIGGER))
                _batteryLevel = BATTERY_LEVEL_MEDIUM;
            if (_batteryVoltageFiltered > (BATTERY_LEVEL_MEDIUM_THRESHOLD + BATTERY_LEVEL_CHANGE_SCHMITT_TRIGGER))
                _batteryLevel = BATTERY_LEVEL_HIGH;
        }
        else if (_batteryLevel == BATTERY_LEVEL_MEDIUM)
        {
            if (_batteryVoltageFiltered > (BATTERY_LEVEL_MEDIUM_THRESHOLD + BATTERY_LEVEL_CHANGE_SCHMITT_TRIGGER))
                _batteryLevel = BATTERY_LEVEL_HIGH;
            else if (_batteryVoltageFiltered < (BATTERY_LEVEL_LOW_THRESHOLD - BATTERY_LEVEL_CHANGE_SCHMITT_TRIGGER))
                _batteryLevel = BATTERY_LEVEL_LOW;
            if (_batteryVoltageFiltered < (BATTERY_LEVEL_CRITICAL_THRESHOLD - BATTERY_LEVEL_CHANGE_SCHMITT_TRIGGER))
                _batteryLevel = BATTERY_LEVEL_CRITICAL;
        }
        else if (_batteryLevel == BATTERY_LEVEL_HIGH)
        {
            if (_batteryVoltageFiltered < (BATTERY_LEVEL_MEDIUM_THRESHOLD - BATTERY_LEVEL_CHANGE_SCHMITT_TRIGGER))
                _batteryLevel = BATTERY_LEVEL_MEDIUM;
            if (_batteryVoltageFiltered < (BATTERY_LEVEL_LOW_THRESHOLD - BATTERY_LEVEL_CHANGE_SCHMITT_TRIGGER))
                _batteryLevel = BATTERY_LEVEL_LOW;
            if (_batteryVoltageFiltered < (BATTERY_LEVEL_CRITICAL_THRESHOLD - BATTERY_LEVEL_CHANGE_SCHMITT_TRIGGER))
                _batteryLevel = BATTERY_LEVEL_CRITICAL;
        }
    }

}

void PM_PCB_Mk5_0::reportTelemetry(Telemetry *telemetry)
{
    telemetry->output_metric("Power V", _batteryVoltage, "V");
#ifdef POWER_EXTENDED_TELEMETRY
    telemetry->output_metric("Power V (s)", _batteryVoltageFiltered, "V");
    telemetry->output_metric("Battery NTC", (uint64_t)_adcBatteryNTCInput_mV, "mV");
    telemetry->output_metric("Charger Status", (uint64_t)_adcChargerStatusInput_mV, "mV");
    telemetry->output_metric("Charger Status Smoothed", _chargerStatusSmoothed_mV, "mV");
#endif
}