#include <Arduino.h>
#include <NimBLEDevice.h>

typedef void (*button_callback_t)(bool);
typedef void (*movement_callback_t)(int, int, int, int, int, int, int);
typedef void (*connect_callback_t)(bool);

class BLE_Client_Joystick {
 public:
    BLE_Client_Joystick() {
      movement_function = NULL;
      connection_function = NULL;
      memset(button_functions, 0, sizeof(button_functions));
    }

    ~BLE_Client_Joystick() {}
    void begin();
    void end();
    void loop();
    void clearPairing();
    void set_connect_callback(connect_callback_t f) { connection_function = f; }
    connect_callback_t get_connect_callback() { return connection_function; }

    void set_button_callback(uint8_t btn, button_callback_t f) {
      button_functions[btn] = f;
    }

    button_callback_t get_button_callback(size_t button) {
        return button_functions[button];
    }

    void set_movement_callback(movement_callback_t f) { movement_function = f; }
    movement_callback_t get_movement_callback() { return movement_function; }

 private:
    button_callback_t button_functions[16];
    movement_callback_t movement_function;
    connect_callback_t connection_function;
};

