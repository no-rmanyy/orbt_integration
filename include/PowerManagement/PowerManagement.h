#pragma once

#include <Arduino.h>
#include "Telemetry.h"

/**
 * @brief Abstract base class for PowerManagement implementations
 *
 * This class provides the interface for the different power management control systems.
 */
class PowerManagement {
public:
    // enum representing our battery's charge level. (Only when reading in relative charge range, 2 - 4.3V)
    // OR if it's charging, or connected to charger (USB-C) but full.
    // Technically charging and full are not 'battery levels', but they're all measured on the single ADC input level so makes sense to enumerate together.
    enum BatteryLevel
    {
        BATTERY_LEVEL_CRITICAL,  // LED flashing red
        BATTERY_LEVEL_LOW,       // LED solid red
        BATTERY_LEVEL_MEDIUM,    // LED solid yellow
        BATTERY_LEVEL_HIGH,      // LED solid green
        BATTERY_LEVEL_INIT       // for a moment at startup, haven't had chance to read and smooth ADC yet
    };
    enum ChargeState
    {
        CHARGE_STATE_BATTERY,       // LED colours as per BatteryLevel enum
        CHARGE_STATE_CHARGING,      // LED pulsing blue
        CHARGE_STATE_CHARGING_FULL,  // LED solid blue
        CHARGE_STATE_INIT           // for a moment at startup, haven't had chance to read and smooth ADC yet
    };

    /**
     * @brief Virtual destructor to ensure proper cleanup of derived classes
     */
    virtual ~PowerManagement() = default;

    /**
     * @brief Initialize the linear mass system
     *
     * This method should be called once during system initialization to set up
     * any required hardware, configurations, or initial states.
     */
    virtual void begin() = 0;

    /**
     * @brief Handle periodic processing of the power management system
     *
     * This method should be called regularly (e.g., in the main loop) to
     * perform any ongoing processing, control updates, or state management.
     */
    virtual void handle(uint64_t currentTime_us) = 0;

     /**
     * @brief report telemetry data for the power management system
     * @param telemetry the telemetry instance
     */
    virtual void reportTelemetry(Telemetry *telemetry) = 0;

// Shared
    /**
     * @brief get the current charge state
     * @return the current charge state
     */
    ChargeState getChargeState();

    /**
     * @brief get the current battery level
     * @return the current battery level
     */
    BatteryLevel getBatteryLevel();

    /**
     * @brief get the previous charge state
     * @return the previous charge state
     */
    ChargeState getPrevChargeState();

    /**
     * @brief get the previous battery level
     * @return the previous battery level
     */
    BatteryLevel getPrevBatteryLevel();


    /**
     * @brief get the current battery voltage
     * @return the current battery voltage
     */
    float getBatteryVoltage();

    /**
     * @brief get the current battery voltage filtered
     * @return the current battery voltage filtered
     */
    float getBatteryVoltageFiltered();

protected:
    /**
     * @brief Protected constructor to prevent direct instantiation
     *
     * Only derived classes can be instantiated.
     */
    PowerManagement() = default;

    // Used to store the raw and filtered battery voltage
    float _batteryVoltage = -1;
    float _batteryVoltageFiltered = -1;

    // Used to store current battery level or charging state
    BatteryLevel _batteryLevel = BATTERY_LEVEL_INIT;
    ChargeState _chargeState = CHARGE_STATE_INIT;

    //keep track of changes in any of these key power/battery variables
    BatteryLevel _prevBatteryLevel = BATTERY_LEVEL_INIT;
    ChargeState _prevChargeState = CHARGE_STATE_INIT;
};
