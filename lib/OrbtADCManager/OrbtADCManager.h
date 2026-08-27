#pragma once

#include <Arduino.h>

#define MAX_ADC_CHANNEL_COUNT  2

class OrbtADCManager {
public:
    /**
     * @brief Virtual destructor to ensure proper cleanup of derived classes
     */
    virtual ~OrbtADCManager() = default;

     /**
     * @brief Start the ADC system
     *
     * This method should be called once during system initialization after
     * any required pins have been configured.
     */
    virtual void start() = 0;

    /**
     * @brief This method should be called in the main loop to handle the ADC system.
     *
     * This method should be called in the main loop to handle the ADC system.
     */
    virtual void handle() = 0;

    /**
     * @brief Add a pin to the ADC system
     *
     * This method should be called to add a pin to the ADC system.
     *
     * @param pin the pin to add
     */
    virtual void addPin(uint8_t pin) = 0;

    /**
     * @brief Get the value of an ADC pin
     *
     * This method should be called to get the value of an ADC pin.
     *
     * @param pin the pin to get the value of
     * @return the value of the ADC pin
     */
    virtual uint16_t getPinValue(uint8_t pin) = 0;

    /**
     * @brief Get the value of an ADC pin in millivolts (calibrated)
     *
     * This method should be called to get the value of an ADC pin in millivolts.
     *
     * @param pin the pin to get the value of
     * @return the value of the ADC pin in millivolts
     */
    virtual uint32_t getPinValueMilliVolts(uint8_t pin) = 0;

    /**
     * @brief Check if an ADC pin has new data
     *
     * This method should be called to check if an ADC pin has new data.
     *
     * @param pin the pin to check
     * @return whether there is new data for the pin
     */
    virtual bool hasNewPinValue(uint8_t pin) = 0;

protected:
    /**
     * @brief Protected constructor to prevent direct instantiation
     *
     * Only derived classes can be instantiated.
     */
    OrbtADCManager() = default;
};