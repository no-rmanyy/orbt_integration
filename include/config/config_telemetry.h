// Defined Values -- Do not change these values
#define TELEMETRY_MODE_TELEPLOT     0
#define TELEMETRY_MODE_V2           1
#define TELEMETRY_MODE_CBOR         2

#ifndef __DEFINES_ONLY
// !! Enable wifi for telemetry, diagnostics/debug, udp tuning, orbtmc
#if !defined(WIFI_ENABLED)
#define WIFI_ENABLED                        true
#endif

// !! Switch on UDP Wifi Logging diagnostics/debug
#if !defined(WIFI_UDP_LOGGING)
    #if WIFI_ENABLED
    #define WIFI_UDP_LOGGING                    true
    #else
    #define WIFI_UDP_LOGGING                    false
    #endif
#endif

#if !defined(WIFI_UDP_TUNING_READ)
    #if WIFI_ENABLED
    #define WIFI_UDP_TUNING_READ                true
    #else
    #define WIFI_UDP_TUNING_READ                false
    #endif
#endif

// Reduce WiFi retries to reduce power consumption
#if !defined(WIFI_REDUCE_WIFI_RETRIES)
#define WIFI_REDUCE_WIFI_RETRIES            false
#endif

#if !defined(RADIO_PREFERENCE_BLE)
#define RADIO_PREFERENCE_BLE                false
#endif

#if !defined(WIFI_POWER_SAVING)
#define WIFI_POWER_SAVING                   false
#endif

// Telemetry Configuration
#if !defined(TELEMETRY_ENABLED)
    #if WIFI_ENABLED
    #define TELEMETRY_ENABLED                   true
    #else
    #define TELEMETRY_ENABLED                   false
    #endif
#endif

#if !defined(TELEMETRY_MODE)
#define TELEMETRY_MODE                      TELEMETRY_MODE_TELEPLOT//TELEMETRY_MODE_CBOR
#endif

// Log Telemetry Every X usecs
#if !defined(TELEMETRY_EVERY_USECS)
#define TELEMETRY_EVERY_USECS               20000   // Log ALL Telemetry every usecs.  Can run up to 10000, depending on volume of UDP calls
                                                    // May need to increase this, less frequent telemetry, if running more than one of the above telemetry options
#endif

// Local port for receiving tuning commands we may receive (on ESP32)
#if !defined(WIFI_TELEMTRY_LOCAL_PORT)
#define WIFI_TELEMTRY_LOCAL_PORT            47270   // port WE are going to listen on, for receiving tuning commands we may receive (on ESP32)
#endif

// to control UDP data rate, and simplify/focus graphs, recommended to only use 1-2 of below telemetry sets.
// !! Choose just telemetry you need for specific testing area
// #define POWER_TELEMETRY   // !! enable power telemetry, including power level and voltage
// #define POWER_EXTENDED_TELEMETRY     // !! enable extended power telemetry, including battery NTC and charger status
// #define YAW_TELEMETRY     // !!enable telemetry for all Yaw PID and rotational control parameters, including wheel/rotational input
// #define OFFSETMASS_TELEMETRY   // !! enable all offsetmass telemetry, including offsetmass custom telemetry and trigger input
// #define OFFSETMASS_TELEMETRY_EXTENDED   // !! enable extended offsetmass telemetry, including offsetmass custom telemetry and trigger input
// #define IMU_TELEMETRY     // !! if set, force output telemetry for ALL 6 IMU parameters
// #define BLE_TELEMETRY     // !! enable telemetry for all BLE parameters
// #define BLE_TELEMETRY_COUNTER     // !! enable telemetry for BLE counter
// #define YAW_PID_TELEMETRY     // !! enable telemetry for all Yaw PID sub componets (Error, P, I, D, Accumulator)
// #define BOT_STATE_TELEMETRY     // !! enable telemetry for bot state / behaviour categorisation
// #define POSE_DETECTOR_TELEMETRY     // !! enable telemetry for pose detector
// #define IMPACT_DETECT_TELEMETRY     // !! enable telemetry for impact detector
// #define ESP_NOW_TELEMETRY     // !! enable telemetry for esp-now

// #define MAINLOOP_TELEMETRY     // !! enable telemetry for mainloop performance
// #define MEMORY_TELEMETRY     // !! enable telemetry for memory usage
// #define TASK_TELEMETRY     // !! enable telemetry for all tasks
// #define TASK_MEMORY_TELEMETRY     // !! enable telemetry for task stack high water mark

#endif // __DEFINES_ONLY