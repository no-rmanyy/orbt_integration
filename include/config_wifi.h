#ifndef __CONFIG_WIFI_H__
#define __CONFIG_WIFI_H__

// !! Fill these in for your own network before building.
//
// WIFI_TELEMTRY_HOST is the IP address of the machine receiving telemetry.
//
// Which telemetry backend gets used is controlled by TELEMETRY_MODE in
// include/config/config_telemetry.h (defaults to TELEMETRY_MODE_CBOR) --
// same switch the original firmware uses:
//   - TELEMETRY_MODE_CBOR:     CBOR-encoded UDP, always sent to port 1202.
//                              View with test/cbor_recv.py (pip install cbor2)
//                              or another CBOR-aware tool (e.g. OrbtViz).
//   - TELEMETRY_MODE_TELEPLOT: plain-text UDP, sent to WIFI_TELEMTRY_PORT below.
//                              View with Teleplot (https://teleplot.fr/).

#define WIFI_SSID           "Normanyy"//"OrbtBotTest2"
#define WIFI_SECRET         "12345678"//"orbtbot123"
#define WIFI_TELEMTRY_HOST  "172.20.10.12"//"192.168.1.10"  // Set to IP address of receiver, i.e. laptop/PC running Teleplot. token change.
#define WIFI_TELEMTRY_PORT  47269

#endif
