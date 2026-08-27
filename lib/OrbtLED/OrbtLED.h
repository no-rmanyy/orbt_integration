#pragma once

#include "Arduino.h"
#include <NeoPixelBus.h>

#define ORBTLED_MAX_LEDS                        2
#define ORBTLED_DURATION_MAX                    UINT32_MAX
#define ORBTLED_BACKGROUND_TASK_INTERVAL_MS     10
#define ORBTLED_QUEUE_DEPTH                     8
#define ORBTLED_BRIGHTNESS_MAX                  1024    // full-scale brightness (2^10)
#define ORBTLED_FIXED_SHIFT                     13      // fractional bits of the internal colour
#define ORBTLED_COLOUR_MAX                      (255 << ORBTLED_FIXED_SHIFT)
#define ORBTLED_OUTPUT_SHIFT                    23      // ORBTLED_FIXED_SHIFT (13) + brightness norm (10)
#define ORBTLED_FORCED_REFRESH_MS               1000    // re-latch all pixels at least this often
#define ORBTLED_GAMMA                           2.2f

class OrbtLED
{
public:
    enum Effect {
        EFFECT_SOLID,
        EFFECT_ALTERNATE,
        EFFECT_FADE,
    };

    struct colour_t {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
        uint8_t white;
    };

    OrbtLED(gpio_num_t dataPin, uint8_t ledCount);
    ~OrbtLED(void);

    void begin(void);
    void handle(uint64_t currentTime_us);

    // Brightness commands (0 - 1024, where 1024 is full scale)
    void setBrightness(uint8_t led_id, uint16_t brightness);

    // Base commands
    void setLedSolid(uint8_t led_id, colour_t colour);
    void setLedAlternate(uint8_t led_id, colour_t colour1, uint32_t duration1_ms, colour_t colour2, uint32_t duration2_ms);
    void setLedFade(uint8_t led_id, colour_t colour1, uint32_t fade1_duration_ms, uint32_t pause1_duration_ms, colour_t colour2, uint32_t fade2_duration_ms, uint32_t pause2_duration_ms);

    // Helper commands (these are just shortcuts for the base commands)
    void setLedFlash(uint8_t led_id, colour_t colour, uint32_t duration_ms);
    void setLedBlink(uint8_t led_id, colour_t colour, uint32_t duration_ms);
    void setLedBreath(uint8_t led_id, colour_t colour, uint32_t duration_ms);
    void setLedPulse(uint8_t led_id, colour_t colour, uint32_t duration_ms);
    // void setLedFadeIn(uint8_t led_id, colour_t colour, uint32_t duration_ms);
    // void setLedFadeOut(uint8_t led_id, colour_t colour, uint32_t duration_ms);

protected:
    gpio_num_t _dataPin;
    uint8_t _ledCount;
    NeoPixelBus<NeoGrbwFeature, NeoSk6812Method> *_leds;     // GRBW wire order (matches our COB hardware)

    uint16_t _brightness[ORBTLED_MAX_LEDS];

    TaskHandle_t _backgroundTaskHandle;
    QueueHandle_t _commandQueueHandle;

    TickType_t _lastWakeTime;           // xTaskDelayUntil anchor (per-instance)
    uint32_t _lastShowTime_ms;          // last Show(), for the forced refresh
    uint8_t _gammaLut[256];             // linear 8-bit -> gamma-corrected 8-bit

    struct colour_u32_t {
        uint32_t red;
        uint32_t green;
        uint32_t blue;
        uint32_t white;
    };

    struct colour_s32_t {
        int32_t red;
        int32_t green;
        int32_t blue;
        int32_t white;
    };

    union command_effect_t {
        struct {
            colour_t colour;
        } solid;
        struct {
            colour_t colour1;
            uint32_t duration1_ms;
            colour_t colour2;
            uint32_t duration2_ms;
        } alternate;
        struct {
            colour_t colour1;
            uint32_t fade1_duration_ms;
            uint32_t pause1_duration_ms;
            colour_t colour2;
            uint32_t fade2_duration_ms;
            uint32_t pause2_duration_ms;
        } fade;
    };

    struct command_t {
        enum Command {
            COMMAND_NONE,
            COMMAND_SET_LED,
            COMMAND_SET_BRIGHTNESS,
        };

        // Common
        Command command;
        uint8_t led_id;

        // Brightness Command
        uint16_t brightness;

        // Set Led Command
        Effect effect;

        // Effect Command Data
        command_effect_t effect_data;
    };

    struct effect_state_t {
        bool isNew;
        Effect effect;
        colour_u32_t colour_current;        // live working colour, shared by all effects
        colour_t current_colour_u8;         // last 8-bit value written to the pixel (render gating)

        // Current State of the effect
        union {
            struct {
                enum AlternateEffectState {
                    COLOUR1,
                    COLOUR2,
                };
                AlternateEffectState state;
                uint32_t stateEndTime_ms;   // time of the next flip

                // Effect Configuration
                colour_u32_t colour1;
                uint32_t duration1_ms;
                colour_u32_t colour2;
                uint32_t duration2_ms;
            } alternate;
            struct {
                enum FadeEffectState {
                    COLOUR1_PAUSE,
                    COLOUR1_FADE,       // Transitioning from colour1 to colour2
                    COLOUR2_PAUSE,
                    COLOUR2_FADE,       // Transitioning from colour2 to colour1
                };
                FadeEffectState state;
                colour_s32_t colour_step;
                uint32_t stateEndTime_ms;

                // Effect Configuration
                colour_u32_t colour1;
                uint32_t fade1_duration_ms;
                uint32_t pause1_duration_ms;
                colour_u32_t colour2;
                uint32_t fade2_duration_ms;
                uint32_t pause2_duration_ms;
            } fade;
        } effect_state;
    };

    effect_state_t _effect_state[ORBTLED_MAX_LEDS];

    void _backgroundProcessCommands(void);
    bool _backgroundUpdateLeds(void);

    // Helpers
    void _enqueue(const command_t &cmd);
    void _applyCommand(const command_t &cmd);
    colour_u32_t _promote(colour_t c);
    colour_t _toOutput(uint8_t led_id, colour_u32_t c);
    colour_s32_t _fadeStep(colour_u32_t start, colour_u32_t target, uint32_t dur_ms);
    colour_u32_t _fadeColourAt(colour_u32_t start, colour_s32_t step, uint32_t step_number);
    void _fadeBeginSubstate(effect_state_t &es, int substate, uint32_t base_ms);
    void _updateAlternate(uint8_t led_id, uint32_t now_ms);
    void _updateFade(uint8_t led_id, uint32_t now_ms);

public:
    void _backgroundTaskHandler(void);
};
