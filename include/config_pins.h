#ifndef __CONFIG_PINS_H__
#define __CONFIG_PINS_H__

#include <Arduino.h>

// Pin definitions trimmed down to what this reduced firmware actually uses
// (IMU SPI bus, status LED, battery/charger ADC inputs). Extend this file if
// you add more peripherals back in.
//
// This project targets ESP32-C6 / ORBT PCB Mk5.0 only.

#if ORBT_PCB_VERSION != 50
#error "This reduced project only supports ORBT_PCB_VERSION == 50 (ESP32-C6, Mk5.0)"
#endif

#define PIN_IMU_CS                      16   // SPI CS   -> IMU
#define PIN_IMU_SCK                     6    // SPI SCK  -> IMU
#define PIN_IMU_MISO                    2    // SPI MISO -> IMU
#define PIN_IMU_MOSI                    7    // SPI MOSI -> IMU

#define PIN_COB_LEDS                    4    // Status LED (single SK6812 GRBW)

#define PIN_ADC_BATTERY_NTC             0    // Battery thermistor
#define PIN_ADC_POWER_LEVEL             1    // Battery voltage sense
#define PIN_ADC_CHARGER_STATUS          5    // Charger IC status output

// Battery voltage divider scale factor (raw ADC mV * this = actual battery mV)
#define BATTERY_VOLTAGE_SCALE           2.0

// Power button -- only used here to detect "hold at boot" as the trigger to
// clear BLE pairing and enter pairing mode (same gesture as the full firmware).
#define PIN_POWER_BUTTON                3
#define POWER_BUTTON_ACTIVE_STATE       HIGH

#endif
