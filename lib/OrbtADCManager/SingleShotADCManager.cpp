#include "SingleShotADCManager.h"

SingleShotADCManager::SingleShotADCManager()
{
}

void SingleShotADCManager::start()
{
}

void SingleShotADCManager::handle()
{
}

void SingleShotADCManager::addPin(uint8_t pin)
{
    // Add a pin to the ADC manager
#if ORBT_PCB_VERSION == 50
    pinMode(pin, INPUT);
#elif ORBT_PCB_VERSION == 42
    pinMode(pin, INPUT_PULLDOWN);       // Still don't know why these are pull down
#endif

}

uint16_t SingleShotADCManager::getPinValue(uint8_t pin)
{
    return analogRead(pin);
}

uint32_t SingleShotADCManager::getPinValueMilliVolts(uint8_t pin)
{
    return analogReadMilliVolts(pin);
}

bool SingleShotADCManager::hasNewPinValue(uint8_t pin)
{
    return true;
}