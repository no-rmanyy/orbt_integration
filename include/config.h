#ifndef __CONFIG_H__
#define __CONFIG_H__

/*
 * Reduced ORBT config.
 *
 * This is a trimmed-down copy of the full orbt-core-firmware config system,
 * keeping only what's needed for: WiFi, BLE (joystick client), UDP telemetry,
 * and the IMU / sensor-fusion (IMS) pipeline.
 */

// !! BLE Controller (joystick) enable
#if !defined(BLE_ENABLE)
#define BLE_ENABLE                   true
#endif

// !! BLE pairing LED visualisation (cyan fast-flash while pairing is active)
#if !defined(PAIRING_VISUALISATION_ENABLE)
#define PAIRING_VISUALISATION_ENABLE                true
#endif

#include "config/config_debug.h"
#include "config/config_imu.h"
#include "config/config_ims.h"
#include "config/config_telemetry.h"

#endif
