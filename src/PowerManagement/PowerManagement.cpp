#include "PowerManagement/PowerManagement.h"

PowerManagement::ChargeState PowerManagement::getChargeState()
{
    return _chargeState;
}

PowerManagement::BatteryLevel PowerManagement::getBatteryLevel()
{
    return _batteryLevel;
}

PowerManagement::ChargeState PowerManagement::getPrevChargeState()
{
    return _prevChargeState;
}

PowerManagement::BatteryLevel PowerManagement::getPrevBatteryLevel()
{
    return _prevBatteryLevel;
}

float PowerManagement::getBatteryVoltage()
{
    return _batteryVoltage;
}

float PowerManagement::getBatteryVoltageFiltered()
{
    return _batteryVoltageFiltered;
}