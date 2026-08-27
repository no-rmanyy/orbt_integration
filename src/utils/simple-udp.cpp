#include <Arduino.h>
#include <WiFiUdp.h>
#include <stdarg.h>
#include "config.h"
#include "config_wifi.h"

WiFiUDP UDP;

void simple_udp_init() 
{
    UDP.begin(WIFI_TELEMTRY_LOCAL_PORT);
}

void simple_udp_printf(const char *format, ...) 
{
    va_list args;
    va_start(args, format);

    uint64_t currentTime = esp_timer_get_time();
    uint32_t seconds = currentTime / 1000000;  // Convert microseconds to seconds
    uint32_t minutes = seconds  / 60;
    uint32_t remainingSeconds = seconds % 60;

    UDP.beginPacket(WIFI_TELEMTRY_HOST, WIFI_TELEMTRY_PORT);
    UDP.printf(">%02d.%02d ", minutes, remainingSeconds);

    //UDP.printf(format, va_arg(args, const char*));
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);
    UDP.print(buffer);
    va_end(args);

    // Add a newline if the format string does not end with a newline
    if (format[strlen(format)-1] != '\n') {
        UDP.print("\n");
    }

    UDP.endPacket();

// this doesn't actually do what flush should do, it clears the input buffer, no wonder they want to rename it.
// UDP.flush();
}
