#pragma once

#include "config.h"
#include "simple-udp.h"

#if WIFI_UDP_LOGGING
#define LOG_INFO(...)       simple_udp_printf(__VA_ARGS__)
#else
#define LOG_INFO(...)       {}
#endif

