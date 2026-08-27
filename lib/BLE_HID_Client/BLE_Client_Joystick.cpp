#include "BLE_Client_Joystick.h"

/** NimBLE_Server Demo:
 *
 *  Demonstrates many of the available features of the NimBLE client library.
 *
 *  Created: on March 24 2020
 *      Author: H2zero
 *
*/

/*
 * This program is based on https://github.com/h2zero/NimBLE-Arduino/tree/master/examples/NimBLE_Client.
 * My changes are covered by the MIT license.
 */

/*
 * MIT License
 *
 * Copyright (c) 2022 esp32beans@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// The button offsets and HID report struct are for the
// "Fortune Key/Game" BLE joystick. Other similar looking devices may use
// different values or not support analog.
//
// Note: This code is for the more expensive upgrade version.
// The device supports analog thumbstick in Game mode. I do not know if the
// cheaper version does or not.
// https://www.amazon.com/dp/B09QJLV6JJ

// 19 bytes
// 00 80 ff 7f ff 7f ff 7f ff 7f ff 7f ff 7f ff 7f ff 7f 04
// 10 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00

// 2 Button Bytes
// 16 bytes x/y/z rz/rx/ry s1/s2
// 1 byte hat

// Disable this to disable whitelist for testing
#define BLE_ONLY_CONNECT_TO_WHITE_LISTED_DEVICES

// Enable debug to verify notifications
// #define BLE_CLIENT_JOYSTICK_DEBUG

// Enable packet timing debug
// These both require BLE_CLIENT_JOYSTICK_DEBUG to be enabled
// BLE_DEBUG_PER_PACKET_TIMING requires BLE_DEBUG_PACKET_TIMING
// #define BLE_DEBUG_PACKET_TIMING
// #define BLE_DEBUG_PACKET_TIMING_PRINT
// #define BLE_DEBUG_PER_PACKET_TIMING_PRINT
#define BLE_DEBUG_PACKET_TIMING_EXPECTED_PERIOD_MS        20      // Expected packet period in ms

//#define BLE_WIFI_UDP_LOGGING
#ifdef BLE_WIFI_UDP_LOGGING
#include <WiFiUdp.h>
#include "../../include/config.h"
#include "../../include/config_wifi.h"
#endif


#ifdef BLE_CLIENT_JOYSTICK_DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(...)
#endif

// Joystick HID report format
typedef struct __attribute__((__packed__)) {
  uint8_t buttons[2];
  uint16_t x;
  uint16_t y;
  uint16_t z;
  uint16_t rz;
  uint16_t rx;
  uint16_t ry;
  uint16_t s1;
  uint16_t s2;
  uint8_t hat;
} joystick_t;

static const char HID_SERVICE[] = "1812";
static const char HID_REPORT_MAP[] = "2A4B";
static const char HID_REPORT_DATA[] = "2A4D";
static const uint32_t scanTime = 0; /** 0 = scan forever */
static void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic,
    uint8_t* pData, size_t length, bool isNotify);
static void scanEndedCB(NimBLEScanResults results);
static bool doConnect = false;
static const NimBLEAdvertisedDevice* advDevice;
static joystick_t Joystick_Report;
static bool Joystick_New_Data = false;

#ifdef BLE_WIFI_UDP_LOGGING
/**
 * Log raw BLE packet data and parsed values for debugging
 * Logs when trigger (X axis) value changes
 */
static void logBLERawPacket(uint8_t* pData, size_t length) {
  static uint16_t last_x_logged = 0;

  if (Joystick_Report.x != last_x_logged) {
    extern WiFiUDP UDP;
    extern const char *WIFI_TELEMTRY_HOST;
    extern const int WIFI_TELEMTRY_PORT;

    UDP.beginPacket(WIFI_TELEMTRY_HOST, WIFI_TELEMTRY_PORT);
    UDP.print(">BLE RAW: ");
    for(uint8_t i = 0; i < length && i < 19; i++) {
      UDP.printf("%02x ", pData[i]);
    }
    UDP.printf("| X=%d (0x%04X) Y=%d Z=%d\n",
               Joystick_Report.x, Joystick_Report.x,
               Joystick_Report.y, Joystick_Report.z);
    UDP.endPacket();
    last_x_logged = Joystick_Report.x;
  }
}
#else
static void logBLERawPacket(uint8_t* pData, size_t length) {
  // No-op when UDP logging is disabled
  (void)pData;
  (void)length;
}
#endif

static uint16_t reportHandle = 0;

/** Define a class to handle the callbacks when advertisments are received */
class AdvertisedDeviceCallbacks: public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) {
    if (advertisedDevice->haveServiceUUID() &&
        advertisedDevice->isAdvertisingService(NimBLEUUID(HID_SERVICE))) {
      DEBUG_PRINT("Advertised HID Device found: ");
      DEBUG_PRINTLN(advertisedDevice->toString().c_str());

      if(NimBLEDevice::getWhiteListCount() > 0) {
        if(NimBLEDevice::onWhiteList(advertisedDevice->getAddress())) {
          DEBUG_PRINTLN("Device is on the white list");
        } else {
          DEBUG_PRINTLN("Device is not on the white list");
#ifdef BLE_ONLY_CONNECT_TO_WHITE_LISTED_DEVICES
          DEBUG_PRINTLN("Ignoring Continuing scan");
          return;
 #endif
          }
      } else {
        DEBUG_PRINTLN("No white list, adding new device to white list");
        NimBLEDevice::whiteListAdd(advertisedDevice->getAddress());
      }

      /** stop scan before connecting */
      NimBLEDevice::getScan()->stop();
      /** Save the device reference in a global for the client to use*/
      advDevice = advertisedDevice;
      /** Read to connect now */
      doConnect = true;
    }
  }
};

/** Create a single global instance of the callback class to be used by all
 * clients
 */

/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pClient) {
    DEBUG_PRINTLN("Connected");
    /** After connection we should change the parameters if we don't need fast response times.
     *  These settings are 150ms interval, 0 latency, 450ms timout.
     *  Timeout should be a multiple of the interval, minimum is 100ms.
     *  I find a multiple of 3-5 * the interval works best for quick response/reconnect.
     *  Min interval: 120 * 1.25ms = 150, Max interval: 120 * 1.25ms = 150, 0 latency, 60 * 10ms = 600ms timeout
     */
    //pClient->updateConnParams(120, 120, 0, 60);   // tylerj this is rubbish.
    //pClient->updateConnParams(6, 7, 0, 600);   // tylerj this is what the controller uses.
    //pClient->updateConnParams(12, 12, 0, 600);   // tylerj 15ms min/max with 6 seconds timeout
    pClient->updateConnParams(16, 16, 0, 51);   // tylerj 20ms min/max with 510ms timeout
  }

  void onDisconnect(NimBLEClient* pClient, int reason) {
    DEBUG_PRINT(pClient->getPeerAddress().toString().c_str());
    DEBUG_PRINTF(" Disconnected: %d - Starting scan\n", reason);
    NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
  }

  /** Called when the peripheral requests a change to the connection parameters.
   *  Return true to accept and apply them or false to reject and keep
   *  the currently used parameters. Default will return true.
   */
  bool onConnParamsUpdateRequest(NimBLEClient* pClient,
      const ble_gap_upd_params* params) {
    // Failing to accepts parameters may result in the remote device
    // disconnecting.
    return true;
  }

  /********************* Security handled here **********************
   ****** Note: these are the same return values as defaults ********/
  uint32_t onPassKeyRequest() {
    DEBUG_PRINTLN("Client Passkey Request");
    /** return the passkey to send to the server */
    return 123456;
  }

  bool onConfirmPIN(uint32_t pass_key) {
    DEBUG_PRINT("The passkey YES/NO number: ");
    DEBUG_PRINTLN(pass_key);
    /** Return false if passkeys don't match. */
    return true;
  }

  /** Pairing process complete, we can check the results in ble_gap_conn_desc */
  void onAuthenticationComplete(ble_gap_conn_desc* desc){
    if (!desc->sec_state.encrypted) {
      DEBUG_PRINTLN("Encrypt connection failed - disconnecting");
      /** Find the client with the connection handle provided in desc */
      NimBLEDevice::getClientByHandle(desc->conn_handle)->disconnect();
      return;
    }
  }
};

ClientCallbacks *clientCB = nullptr;

NimBLEClient* debugBLEClient = nullptr;
uint32_t invalid_packet_count = 0;

/** Handles the provisioning of clients and connects / interfaces with
 * the server
 */
bool connectToServer() {
  NimBLEClient* pClient = nullptr;

  if (!clientCB) {
    clientCB = new ClientCallbacks();
  }

  /** Check if we have a client we should reuse first **/
  if (NimBLEDevice::getCreatedClientCount()) {
    /** Special case when we already know this device, we send false as the
     *  second argument in connect() to prevent refreshing the service database.
     *  This saves considerable time and power.
     */
    pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
    if (pClient) {
      if (!pClient->connect(advDevice, false)) {
        DEBUG_PRINTLN("Reconnect failed");
        //return false;
      } else {
        DEBUG_PRINTLN("Reconnected client");
      }
    }
    /** We don't already have a client that knows this device,
     *  we will check for a client that is disconnected that we can use.
     */
    else {
      pClient = NimBLEDevice::getDisconnectedClient();
    }
  }

  /** No client to reuse? Create a new one. */
  if (!pClient) {
    if (NimBLEDevice::getCreatedClientCount() >= NIMBLE_MAX_CONNECTIONS) {
      DEBUG_PRINTLN("Max clients reached - no more connections available");
      return false;
    }

    pClient = NimBLEDevice::createClient();

    DEBUG_PRINTLN("New client created");

    pClient->setClientCallbacks(clientCB, false);
    /** Set initial connection parameters: These settings are 15ms interval, 0 latency, 120ms timout.
     *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
     *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
     *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 51 * 10ms = 510ms timeout
     */
    //pClient->setConnectionParams(12, 12, 0, 51);    //tylerj the default, but also had good results with 24, 48, 0, 200
    pClient->setConnectionParams(16, 16, 0, 51);    //tylerj the default, but also had good results with 24, 48, 0, 200
    
    /** Set how long we are willing to wait for the connection to complete
     * (seconds), default is 30.
     */
    pClient->setConnectTimeout(300);  // tylerj the default was: 5


    if (!pClient->connect(advDevice)) {
      /** Created a client but failed to connect, don't need to keep it as it
       * has no data
       */
      NimBLEDevice::deleteClient(pClient);
      DEBUG_PRINTLN("Failed to connect, deleted client");
      return false;
    }
  }

  if (!pClient->isConnected()) {
    if (!pClient->connect(advDevice)) {
      DEBUG_PRINTLN("Failed to connect");
      return false;
    }
  }

  DEBUG_PRINT("Connected to: ");
  DEBUG_PRINTLN(pClient->getPeerAddress().toString().c_str());
  DEBUG_PRINT("RSSI: ");
  DEBUG_PRINTLN(pClient->getRssi());
  debugBLEClient = pClient;

  /** Now we can read/write/subscribe the charateristics of the services we
   * are interested in
   */
  NimBLERemoteService* pSvc = nullptr;
  NimBLERemoteCharacteristic* pChr = nullptr;
  NimBLERemoteDescriptor* pDsc = nullptr;


  pSvc = pClient->getService(HID_SERVICE);
  if (pSvc) {     /** make sure it's not null */
    // Subscribe to characteristics HID_REPORT_DATA.
    // One real device reports 2 with the same UUID but
    // different handles. Using getCharacteristic() results
    // in subscribing to only one.

    std::vector<NimBLERemoteCharacteristic*>charvector;
    charvector = pSvc->getCharacteristics(true);
    for (auto &it : charvector) {
      //std::vector<NimBLERemoteDescriptor*>*descvector = it->getDescriptors(true);
      //DEBUG_PRINTLN(it->toString().c_str());
      // for (auto &desc_it : *descvector) {
      //   DEBUG_PRINTLN(it->toString().c_str());
      // }

      // if (it->getUUID() == NimBLEUUID(HID_REPORT_MAP)) {
      //   DEBUG_PRINT("Found HID Report Map: ");
      //   DEBUG_PRINTLN(it->toString().c_str());

      //   std::string map = it->readValue();

      //   for(uint8_t mi = 0; mi < map.length(); mi++) {
      //     DEBUG_PRINTF("%2.2x ", map[mi]);
      //     if(mi % 8 == 7) {
      //       DEBUG_PRINTLN("");
      //     }
      //   }
      //   DEBUG_PRINTLN("\n");
      // }

      if (it->getUUID() == NimBLEUUID(HID_REPORT_DATA)) {
        DEBUG_PRINT("Found: ");
        DEBUG_PRINTLN(it->toString().c_str());

        if (it->canNotify()) {
          DEBUG_PRINTF("Subscribing to handle %d...\n", it->getHandle());
          if (!it->subscribe(true, notifyCB, true)) {
            /** Disconnect if subscribe failed */
            DEBUG_PRINTLN("ERROR: subscribe notification failed");
            pClient->disconnect();
            return false;
          } else {
            reportHandle = it->getHandle();
            DEBUG_PRINTF("Subscribed successfully to handle %d\n", reportHandle);

            // Small delay to let subscription settle
            delay(50);

            // Verify CCCD was written
            NimBLERemoteDescriptor* pDesc = it->getDescriptor(NimBLEUUID("2902"));
            if (pDesc) {
              std::string value = pDesc->readValue();
              if (value.length() >= 2) {
                uint16_t cccd = (uint8_t)value[1] << 8 | (uint8_t)value[0];
                DEBUG_PRINTF("CCCD readback: 0x%04x\n", cccd);
              } else {
                DEBUG_PRINTLN("CCCD readback failed");
              }
            } else {
              DEBUG_PRINTLN("CCCD not found");
            }

            // Only trigger the "readValue()" to provoke notifications if not connecting to "ESP32 BLE Gamepad"
            std::string deviceName = advDevice->getName();
            if (deviceName != "ESP32 BLE Gamepad") {
                DEBUG_PRINTLN("Reading characteristic to trigger notifications...");
                std::string initialValue = it->readValue();
                DEBUG_PRINTF("Initial read: %d bytes\n", initialValue.length());
            } else {
                DEBUG_PRINTLN("Skipping initial read for ESP32 BLE Gamepad.");
            }
          }
        }
      }
    }
  }
  DEBUG_PRINTLN("Done with this device!");
  return true;
}

// tylerj - restarting after an end hasn't been tested
void BLE_Client_Joystick::end() {
  if(debugBLEClient != nullptr) {
    debugBLEClient->disconnect();
    NimBLEDevice::deleteClient(debugBLEClient);
    debugBLEClient = nullptr;
  }

  NimBLEDevice::getScan()->stop();

  //tylerj the next line is causing -- multi_heap_free multi_heap_poisoning.c on MK5.0 PCB
#if ORBT_PCB_VERSION != 50
  NimBLEDevice::deinit(true);
#endif
  debugBLEClient = nullptr;
}

void BLE_Client_Joystick::begin() {
  /** Initialize NimBLE, no device name spcified as we are not advertising */
  NimBLEDevice::init("");

  /** Set the IO capabilities of the device, each option will trigger a different pairing method.
   *  BLE_HS_IO_KEYBOARD_ONLY    - Passkey pairing
   *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
   *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
   */
  //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_KEYBOARD_ONLY); // use passkey
  //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison

  /** 2 different ways to set security - both calls achieve the same result.
   *  no bonding, no man in the middle protection, secure connections.
   *
   *  These are the default values, only shown here for demonstration.
   */
  NimBLEDevice::setSecurityAuth(true, false, true);
  //NimBLEDevice::setSecurityAuth(/*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);

  /** Optional: set the transmit power, default is 3db */
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  //NimBLEDevice::setPower(ESP_PWR_LVL_P12); /** +12db */

  /** Optional: set any devices you don't want to get advertisments from */
  // NimBLEDevice::addIgnored(NimBLEAddress ("aa:bb:cc:dd:ee:ff"));

  if(NimBLEDevice::getNumBonds() > 0) {
    for(int i = 0; i < NimBLEDevice::getNumBonds(); i++) {
      NimBLEAddress dev = NimBLEDevice::getBondedAddress(i);
      DEBUG_PRINTF("Adding Bonded Device To White List: %s\n", dev.toString().c_str());
      NimBLEDevice::whiteListAdd(dev);
    }
  }

  /** create new scan */
  NimBLEScan* pScan = NimBLEDevice::getScan();

  /** create a callback that gets called when advertisers are found */
  pScan->setScanCallbacks(new AdvertisedDeviceCallbacks(), true);

  /** Set scan interval (how often) and window (how long) in milliseconds */
  pScan->setInterval(45);
  pScan->setWindow(15);

  /** Active scan will gather scan response data from advertisers
   *  but will use more energy from both devices
   */
  pScan->setActiveScan(true);
  /** Start scanning for advertisers for the scan time specified (in seconds) 0 = forever
   *  Optional callback for when scanning stops.
   */
  pScan->start(scanTime, scanEndedCB);
}

void BLE_Client_Joystick::clearPairing() 
{
  if(debugBLEClient != nullptr) {
    DEBUG_PRINTLN("Disconnecting from Remote Device");
    debugBLEClient->disconnect();
    NimBLEDevice::deleteClient(debugBLEClient);
    debugBLEClient = nullptr;
  }

  DEBUG_PRINTLN("Stopping Scan");
  NimBLEDevice::getScan()->stop();

  DEBUG_PRINTLN("Deleting All Bonds");
  NimBLEDevice::deleteAllBonds();

  for(int i = 0; i < NimBLEDevice::getWhiteListCount(); i++) {
    NimBLEAddress dev = NimBLEDevice::getWhiteListAddress(i);
    DEBUG_PRINTF("Deleting White List: %s\n", dev.toString().c_str());
    NimBLEDevice::whiteListRemove(dev);
  }

  DEBUG_PRINTLN("Delaying 500ms");
  delay(500);

  DEBUG_PRINTLN("Restarting Scan");
  NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
}

void BLE_Client_Joystick::loop() {
  /** Loop here until we find a device we want to connect to */
  if (doConnect) {
    doConnect = false;

    /** Found a device we want to connect to, do it now */
    if (!connectToServer()) {
      NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
    }
  }
  // Processing incoming notifications/indications
  if (Joystick_New_Data) {
    static uint16_t last_x = 0;
    static uint16_t last_y = 0;
    static uint16_t last_z = 0;
    static uint16_t last_rx = 0;
    static uint16_t last_ry = 0;
    static uint16_t last_rz = 0;
    static uint16_t last_s1 = 0;
    movement_callback_t f = this->get_movement_callback();
    if (f) {
      if ((last_x != Joystick_Report.x) || (last_y != Joystick_Report.y) || (last_z != Joystick_Report.z) ||
          (last_rx != Joystick_Report.rx) || (last_ry != Joystick_Report.ry) || (last_rz != Joystick_Report.rz) ||
          (last_s1 != Joystick_Report.s1)) {
        (*f)(Joystick_Report.x, Joystick_Report.y, Joystick_Report.z, Joystick_Report.rx, Joystick_Report.ry, Joystick_Report.rz, Joystick_Report.s1);
        last_x = Joystick_Report.x;
        last_y = Joystick_Report.y;
        last_z = Joystick_Report.z;
        last_rx = Joystick_Report.rx;
        last_ry = Joystick_Report.ry;
        last_rz = Joystick_Report.rz;
        last_s1 = Joystick_Report.s1;
      }
    }

    static uint16_t last_buttons = 0;
    uint16_t curr_buttons = Joystick_Report.buttons[1]<<8 | Joystick_Report.buttons[0];
    uint16_t changed = last_buttons ^ curr_buttons;
    uint16_t buttons = curr_buttons;

    for (size_t i = 0; i < 16; i++) {
      button_callback_t f = this->get_button_callback(i);
      if (f && (changed & 1)) {
        (*f)(buttons & 1);
      }
      changed >>= 1;
      buttons >>= 1;
    }

    last_buttons = curr_buttons;
    Joystick_New_Data = false;
  }
}

/*
**  ble_packet_time_diff_ms
**  Calculates the time difference in milliseconds between BLE stream packets
**  Expected report rate: 50Hz (20ms between packets)
**  @param last_packet_time_ms Timestamp of the last packet in milliseconds
**  @param current_packet_time_ms Timestamp of the current packet in milliseconds
**  @return Time difference in milliseconds, handles rollover at UINT32_MAX
*/
uint32_t ble_packet_time_diff_ms(uint32_t last_packet_time_ms, uint32_t current_packet_time_ms)
{
    // Handle rollover case: when current time is smaller than last time
    // This occurs when millis() rolls over at UINT32_MAX (~49.7 days)
    if (current_packet_time_ms < last_packet_time_ms) {
        // Calculate difference accounting for rollover
        return (UINT32_MAX - last_packet_time_ms) + current_packet_time_ms + 1;
    } else {
        // Normal case: simple subtraction
        return current_packet_time_ms - last_packet_time_ms;
    }
}

/*
**  ble_packet_rate_hz
**  Calculates the instantaneous packet rate in Hz based on time difference
**  Expected rate: 50Hz for normal operation
**  @param time_diff_ms Time difference between packets in milliseconds
**  @return Packet rate in Hz (0 if time_diff_ms is 0 to avoid division by zero)
*/
float ble_packet_rate_hz(uint32_t time_diff_ms)
{
    // Avoid division by zero
    if (time_diff_ms == 0) {
        return 0.0f;
    }

    // Convert ms to Hz: 1000ms / time_diff_ms = packets per second
    return 1000.0f / (float)time_diff_ms;
}

/**
 * Measures and logs BLE packet timing statistics
 * Expected packet rate: 50Hz (20ms intervals)
 */
static void measure_packet_timing() {
  static uint32_t last_packet_time_ms = 0;
  static uint32_t packet_counter = 0;
  static uint32_t total_time_sum_ms = 0;
  static uint32_t min_interval_ms = UINT32_MAX;
  static uint32_t max_interval_ms = 0;
  static uint32_t out_of_spec_count = 0;
  static uint32_t out_of_spec_count_longer = 0;
  static uint32_t out_of_spec_count_shorter = 0;
  static uint32_t out_of_spec_total_longer = 0;
  static uint32_t out_of_spec_total_shorter = 0;
  bool out_of_spec = false;

  uint32_t current_time_ms = millis();

  if (last_packet_time_ms != 0) {
    uint32_t time_diff_ms = ble_packet_time_diff_ms(last_packet_time_ms, current_time_ms);
    float packet_rate_hz = ble_packet_rate_hz(time_diff_ms);

    // Track statistics
    total_time_sum_ms += time_diff_ms;
    if (time_diff_ms < min_interval_ms) min_interval_ms = time_diff_ms;
    if (time_diff_ms > max_interval_ms) max_interval_ms = time_diff_ms;

    // Count packets outside acceptable range (15-25ms = 40-66Hz)
    // 15ms update rate to match intervals
    if (time_diff_ms < BLE_DEBUG_PACKET_TIMING_EXPECTED_PERIOD_MS - 5) {
      out_of_spec_total_shorter += time_diff_ms;
      out_of_spec_count_shorter++;
      out_of_spec_count++;
      out_of_spec = true;
    }
    if (time_diff_ms > BLE_DEBUG_PACKET_TIMING_EXPECTED_PERIOD_MS + 5) {
      out_of_spec_total_longer += time_diff_ms;
      out_of_spec_count_longer++;
      out_of_spec_count++;
      out_of_spec = true;
    }

    packet_counter++;

#ifdef BLE_DEBUG_PER_PACKET_TIMING_PRINT
    if(!out_of_spec) {
      DEBUG_PRINT("20ms,");
    } else {
      DEBUG_PRINTF("%lums,", time_diff_ms);
    }
#endif

#ifdef BLE_DEBUG_PACKET_TIMING_PRINT
    // Log detailed statistics every 100 packets
    if (packet_counter % 100 == 0) {
      float avg_interval_ms = (float)total_time_sum_ms / packet_counter;
      float avg_rate_hz = 1000.0f / avg_interval_ms;
      float jitter_percent = ((float)(max_interval_ms - min_interval_ms) / avg_interval_ms) * 100.0f;

      DEBUG_PRINTF("BLE Timing Stats [%lu packets]:\n", packet_counter);
      DEBUG_PRINTF("  Current: %lu ms, %.2f Hz\n", time_diff_ms, packet_rate_hz);
      DEBUG_PRINTF("  Average: %.2f ms, %.2f Hz\n", avg_interval_ms, avg_rate_hz);
      DEBUG_PRINTF("  Min/Max: %lu/%lu ms\n", min_interval_ms, max_interval_ms);
      DEBUG_PRINTF("  Jitter: %.1f%% | Out-of-spec: %lu (%.1f%%)\n",
                   jitter_percent, out_of_spec_count,
                   (float)out_of_spec_count / packet_counter * 100.0f);
      DEBUG_PRINTF("  Out-of-spec shorter: %lu (%.1f%%)\n", out_of_spec_count_shorter, (float)out_of_spec_count_shorter / packet_counter * 100.0f);
      DEBUG_PRINTF("  Out-of-spec longer: %lu (%.1f%%)\n", out_of_spec_count_longer, (float)out_of_spec_count_longer / packet_counter * 100.0f);
      DEBUG_PRINTF("  Out-of-spec avg shorter: %lu ms\n", out_of_spec_total_shorter / out_of_spec_count_shorter);
      DEBUG_PRINTF("  Out-of-spec avg longer: %lu ms\n", out_of_spec_total_longer / out_of_spec_count_longer);
    }
#endif
  }

  last_packet_time_ms = current_time_ms;
}

/** Notification / Indication receiving handler callback */
// WARNING: This device has 4 Characteristics = 0x2a4d but with different
// handle values.
static void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
  uint16_t handle = pRemoteCharacteristic->getHandle();

  // Measure packet timing for stream frequency monitoring
  //measure_packet_timing();

  // First notification received
  static bool firstNotification = true;
  if (firstNotification) {
    DEBUG_PRINTLN("*** FIRST NOTIFICATION RECEIVED! ***");
    firstNotification = false;
  }

  if(handle == reportHandle) {
    if (length != sizeof(Joystick_Report)) {
      DEBUG_PRINTF("BLE Unexpected Report Size: got %d, expected %d\n", length, sizeof(Joystick_Report));
      invalid_packet_count++;
      return;
    }

#ifdef BLE_DEBUG_PACKET_TIMING
    // Measure packet timing for stream frequency monitoring
    measure_packet_timing();
#endif

    // DEBUG_PRINT("notifyCB handle:");
    // DEBUG_PRINT(handle);
    // DEBUG_PRINT(" size: ");
    // DEBUG_PRINT(length);
    // DEBUG_PRINT(" data: ");

    // for(uint8_t i = 0; i < length; i++) {
    //   DEBUG_PRINTF("%2.2x ", pData[i]);
    // }
    // DEBUG_PRINTLN("\n");

    memcpy(&Joystick_Report, pData, sizeof(Joystick_Report));
    Joystick_New_Data = true;

    // Print first 3 reports to verify data format
    static uint8_t reportCount = 0;
    if (reportCount < 3) {
      DEBUG_PRINTF("Report #%d: ", reportCount);
      for(uint8_t i = 0; i < length && i < 19; i++) {
        DEBUG_PRINTF("%02x ", pData[i]);
      }
      DEBUG_PRINTLN("");
      reportCount++;
    }

    // Log raw BLE packet for debugging
    // logBLERawPacket(pData, length);

    // Print button presses
    #if 0
    if (Joystick_Report.buttons[0] != 0 || Joystick_Report.buttons[1] != 0) {
      DEBUG_PRINTF("BUTTON: 0x%02x 0x%02x (combined: 0x%04x)\n",
                   Joystick_Report.buttons[0], Joystick_Report.buttons[1],
                   (Joystick_Report.buttons[1]<<8 | Joystick_Report.buttons[0]));
    }
    #endif

    // tylerj use motor 1 esc output for debug
    // static bool toggle = false;
    // digitalWrite(GPIO_NUM_1, toggle);
    // toggle = !toggle;

    // if (Joystick_Report.buttons[0] != 0 || Joystick_Report.buttons[1] != 0) {
    //   UDP.beginPacket(udpAddress, remoteUDPPort);
    //   UDP.printf(">BLE ButtonPressed 0x%x\n", Joystick_Report.buttons[0]);

    //   UDP.print(">notifyCB handle ");
    //   UDP.print(handle);
    //   UDP.print(" size ");
    //   UDP.print(length);
    //   UDP.print(" data ");

    //   for(uint8_t i = 0; i < length; i++) {
    //     UDP.printf("%2.2x ", pData[i]);
    //   }
    //   UDP.print("\n");
    //   UDP.endPacket();

    // }

    // DEBUG_PRINTF("Buttons: 0x%2.2x 0x%2.2x\n", Joystick_Report.buttons[0], Joystick_Report.buttons[1]);
    // DEBUG_PRINTF("X: %d - Y: %d - Z: %d\n", Joystick_Report.x, Joystick_Report.y, Joystick_Report.z);
    // DEBUG_PRINTF("rX: %d - rY: %d - rZ: %d\n", Joystick_Report.rx, Joystick_Report.ry, Joystick_Report.rz);
    // DEBUG_PRINTF("s1: %d - s2: %d\n", Joystick_Report.s1, Joystick_Report.s2);
    // DEBUG_PRINTF("hat: 0x%2.2x: %d\n", Joystick_Report.hat);
    // DEBUG_PRINTLN("");
  }
}

/** Callback to process the results of the last scan or restart it */
static void scanEndedCB(NimBLEScanResults results) {
  DEBUG_PRINTLN("Scan Ended");
}
