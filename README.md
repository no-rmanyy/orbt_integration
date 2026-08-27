# ORBT Sensor Node (reduced firmware)

A stripped-down copy of `orbt-core-firmware`, keeping five subsystems, targeting
a single board: **ESP32-C6, ORBT PCB Mk5.0**.

- **WiFi** — station mode connect + UDP telemetry transport
- **BLE** — NimBLE joystick/gamepad client (`BLE_Client_Joystick`): all 7 movement
  axes, all 4 buttons, RSSI + connection info, and the original power-button-hold-at-
  boot pairing gesture with LED visualisation
- **Telemetry** — UDP, both original backends included (`OrbtTelemetry` lib):
  `CborTelemetry` (default) and `TeleplotTelemetry`, switchable via `TELEMETRY_MODE`
- **IMU / IMS / Sensor Fusion** — ICM42688 driver + background sampling task +
  Madgwick/VQF orientation fusion (`IMS` module)
- **Status LED** — single addressable RGBW LED: cyan while pairing, blue while
  charging, colour-coded by battery level otherwise (same mapping as the full
  firmware's `StatusLed`)

Everything else from the full firmware has been removed: motor control (DSHOT/stepper),
offset-mass actuators, audio effects, ESP-NOW, the drive-state machine, impact/pose
detection, OrbtMC command handling, etc.

## What's kept, and where it comes from

| Subsystem       | Files |
|-----------------|-------|
| WiFi + Telemetry | `src/main.cpp`, `include/utils/simple-udp.*`, `lib/OrbtTelemetry/` (Cbor backend) |
| BLE joystick     | `lib/BLE_HID_Client/` |
| IMU + Sensor Fusion | `include/IMS/ims.h`, `src/IMS/ims.cpp`, `lib/OrbtICM42688/`, `lib/OrbtDSP/`, `include/utils/imu_helper.*` |
| Status LED       | `lib/OrbtLED/`, `include/StatusLed/`, `src/StatusLed/` (trimmed: no drive-state/pairing/passthrough LED states) |
| Battery/charging | `lib/OrbtADCManager/` (SingleShot only), `include/PowerManagement/`, `src/PowerManagement/` (Mk5.0 only) |
| Config           | `include/config.h`, `include/config_pins.h`, `include/config_wifi.h`, `include/config/config_{imu,ims,telemetry,debug}.h` |

`NeoPixelBus` is pulled in as a normal PlatformIO registry dependency
(`makuna/NeoPixelBus@^2.8.4`) rather than the vendored copy in the full project, to
keep this repo small — the public release works fine for driving a single SK6812.

## Setup

1. Edit `include/config_wifi.h` with your WiFi SSID/password and the IP address of the
   machine that will receive telemetry. **Double-check this IP is correct and
   reachable** — a stale/placeholder IP here is the most common reason telemetry
   appears to "not work".
2. Build & upload with PlatformIO (single environment, `sensor-node-mk50`):
   ```
   pio run -e sensor-node-mk50 -t upload
   ```
3. Pair a BLE HID gamepad (the joystick client is written for the "Fortune Key/Game"
   style controller — see comments at the top of `BLE_Client_Joystick.cpp` if you're
   using a different device).
4. To view telemetry, run the included test receiver (requires `pip install cbor2`):
   ```
   python3 test/cbor_recv.py --port 1202 -v
   ```
   **CBOR is a binary format** — a plain-text tool like Teleplot will show garbled
   binary bytes if you point it at a CBOR stream. Use `cbor_recv.py` (or another
   CBOR-aware tool, e.g. OrbtViz) instead. To use Teleplot instead, set
   `TELEMETRY_MODE` to `TELEMETRY_MODE_TELEPLOT` in
   `include/config/config_telemetry.h` and rebuild — see "Extending" below.

## Status LED behaviour

- **BLE pairing mode**: cyan, fast flash (see "Pairing a BLE joystick" below)
- **Charging**: blue, breathing (slow pulse)
- **Charging, full**: blue, solid
- **On battery**, by voltage:
  - Critical: red, flashing
  - Low: red, solid
  - Medium: yellow, solid
  - High: green, solid

Battery level and charge state are sampled every 100ms from two ADC pins
(`PIN_ADC_POWER_LEVEL`, `PIN_ADC_CHARGER_STATUS`) using the exact same voltage
thresholds as the full firmware's `PM_PCB_Mk5_0` — see
`include/PowerManagement/PM_PCB_Mk5_0.h` if you need to retune them for a different
battery/charger IC.

## Pairing a BLE joystick

Same gesture as the full firmware: **hold the power button down while powering on the
board, and keep holding it for 2 seconds.** This calls `bleJoystick.clearPairing()` and
puts the LED into pairing mode (cyan, fast flash) until a joystick connects, at which
point it drops back to the normal battery/charging display.

This only fires if the button is held continuously from boot — if it's ever seen
released, the trigger is disarmed for that boot cycle (so a stray button press later on
won't accidentally start pairing). It only fires once per boot.

This needs `PIN_POWER_BUTTON` wired up (pin 3 on Mk5.0, active-high) even though this
reduced project doesn't otherwise use the power button for anything else (no
power-on/off state machine). Set `PAIRING_VISUALISATION_ENABLE` to `false` in
`include/config.h` to disable this entirely.

## What you'll see

On boot the board connects to WiFi, starts the IMU sampling + sensor-fusion task, and
begins scanning for a paired BLE joystick. Every ~20ms it sends a CBOR-encoded UDP
telemetry frame containing:

- Free heap, WiFi RSSI
- Raw accel/gyro (`AccelX/Y/Z`, `GyroX/Y/Z`)
- Fused orientation (`Roll`, `Pitch`, `Yaw`) — from Madgwick or VQF, see
  `include/config/config_imu.h` (`IMU_SENSOR_FUSION_MODE`)
- BLE joystick: connection state, all 7 movement axes (`Joy_TriggerX`, `Joy_WheelY`,
  `Joy_Z`, `Joy_Aux1`, `Joy_Aux2`, `Joy_Rz`, `Joy_S1`), all 4 buttons
  (`Joy_ButtonA`-`Joy_ButtonD`), and once connected: `BLE_RSSI` plus connection
  interval/timeout/latency (from `NimBLEConnInfo`)
- `Power V` — battery voltage (plus more under `POWER_EXTENDED_TELEMETRY`, see
  `include/config/config_telemetry.h`)




