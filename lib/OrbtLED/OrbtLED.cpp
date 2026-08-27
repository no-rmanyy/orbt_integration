#include "OrbtLED.h"
#include <math.h>

static void BackgroundTaskStub(void *args)
{
    OrbtLED *self = (OrbtLED *)args;
    self->_backgroundTaskHandler();
}

OrbtLED::OrbtLED(gpio_num_t dataPin, uint8_t ledCount)
{
    if(ledCount > ORBTLED_MAX_LEDS) {
        Serial.printf("ERROR OrbtLED: ledCount is greater than ORBTLED_MAX_LEDS, setting to ORBTLED_MAX_LEDS\n");
        ledCount = ORBTLED_MAX_LEDS;
    }

    _dataPin = dataPin;
    _ledCount = ledCount;
    _backgroundTaskHandle = nullptr;
    _commandQueueHandle = nullptr;
    _lastShowTime_ms = 0;

    _leds = new NeoPixelBus<NeoGrbwFeature, NeoSk6812Method>(_ledCount, dataPin);
}

OrbtLED::~OrbtLED(void)
{
    // Ownership rule: anything created by this class is destroyed by this class.
    if(_backgroundTaskHandle != nullptr) {
        vTaskDelete(_backgroundTaskHandle);
        _backgroundTaskHandle = nullptr;
    }
    if(_commandQueueHandle != nullptr) {
        vQueueDelete(_commandQueueHandle);
        _commandQueueHandle = nullptr;
    }
    if(_leds != nullptr) {
        delete _leds;
        _leds = nullptr;
    }
}

void OrbtLED::begin(void)
{
    _leds->Begin();

    // Build the gamma LUT once (per §2: calculate-once-and-store).
    for(int i = 0; i < 256; ++i) {
        float g = powf((float)i / 255.0f, ORBTLED_GAMMA);
        int v = (int)lroundf(g * 255.0f);
        if(v < 0)   v = 0;
        if(v > 255) v = 255;
        _gammaLut[i] = (uint8_t)v;
    }

    // Default state: every LED solid black (off) at full brightness.
    for(uint8_t i = 0; i < ORBTLED_MAX_LEDS; ++i) {
        _brightness[i] = ORBTLED_BRIGHTNESS_MAX;
        _effect_state[i] = effect_state_t{};
        _effect_state[i].effect = EFFECT_SOLID;
        _effect_state[i].colour_current = colour_u32_t{0, 0, 0, 0};
        _effect_state[i].current_colour_u8 = colour_t{0, 0, 0, 0};
        _effect_state[i].isNew = false;
    }

    // Push an initial all-black frame so the strip starts in a known state.
    if(_leds != nullptr) {
        for(uint8_t i = 0; i < _ledCount; ++i) {
            _leds->SetPixelColor(i, RgbwColor(0, 0, 0, 0));
        }
        _leds->Show();
    }
    _lastShowTime_ms = millis();

    _commandQueueHandle = xQueueCreate(ORBTLED_QUEUE_DEPTH, sizeof(command_t));
    if(_commandQueueHandle == nullptr) {
        Serial.printf("ERROR OrbtLED: failed to create command queue\n");
        return;
    }

    BaseType_t taskCreated = xTaskCreate(BackgroundTaskStub, "OrbtLED_BT", 4096, this, 1, &_backgroundTaskHandle);
    if(taskCreated != pdPASS) {
        Serial.printf("ERROR OrbtLED: failed to create background task\n");
        _backgroundTaskHandle = nullptr;
    }
}

void OrbtLED::handle(uint64_t currentTime_us)
{
    /* nothing to do here, all processing is done in the background task */
}

/* --------------------------------------------------------------------------
 * Public command API (producer side) — all just enqueue a command_t.
 * ------------------------------------------------------------------------ */

void OrbtLED::_enqueue(const command_t &cmd)
{
    if(_commandQueueHandle == nullptr) {
        return;
    }
    if(xQueueSend(_commandQueueHandle, &cmd, 0) != pdTRUE) {
        // Queue full: drop the oldest entry so the newest still wins.
        command_t discard;
        xQueueReceive(_commandQueueHandle, &discard, 0);
        xQueueSend(_commandQueueHandle, &cmd, 0);
    }
}

void OrbtLED::setBrightness(uint8_t led_id, uint16_t brightness)
{
    if(led_id >= _ledCount) {
        return;
    }
    command_t c = {};
    c.command = command_t::COMMAND_SET_BRIGHTNESS;
    c.led_id = led_id;
    c.brightness = brightness;
    _enqueue(c);
}

void OrbtLED::setLedSolid(uint8_t led_id, colour_t colour)
{
    if(led_id >= _ledCount) {
        return;
    }
    command_t c = {};
    c.command = command_t::COMMAND_SET_LED;
    c.led_id = led_id;
    c.effect = EFFECT_SOLID;
    c.effect_data.solid.colour = colour;
    _enqueue(c);
}

void OrbtLED::setLedAlternate(uint8_t led_id, colour_t colour1, uint32_t duration1_ms, colour_t colour2, uint32_t duration2_ms)
{
    if(led_id >= _ledCount) {
        return;
    }
    command_t c = {};
    c.command = command_t::COMMAND_SET_LED;
    c.led_id = led_id;
    c.effect = EFFECT_ALTERNATE;
    c.effect_data.alternate.colour1 = colour1;
    c.effect_data.alternate.duration1_ms = duration1_ms;
    c.effect_data.alternate.colour2 = colour2;
    c.effect_data.alternate.duration2_ms = duration2_ms;
    _enqueue(c);
}

void OrbtLED::setLedFade(uint8_t led_id, colour_t colour1, uint32_t fade1_duration_ms, uint32_t pause1_duration_ms, colour_t colour2, uint32_t fade2_duration_ms, uint32_t pause2_duration_ms)
{
    if(led_id >= _ledCount) {
        return;
    }
    command_t c = {};
    c.command = command_t::COMMAND_SET_LED;
    c.led_id = led_id;
    c.effect = EFFECT_FADE;
    c.effect_data.fade.colour1 = colour1;
    c.effect_data.fade.fade1_duration_ms = fade1_duration_ms;
    c.effect_data.fade.pause1_duration_ms = pause1_duration_ms;
    c.effect_data.fade.colour2 = colour2;
    c.effect_data.fade.fade2_duration_ms = fade2_duration_ms;
    c.effect_data.fade.pause2_duration_ms = pause2_duration_ms;
    _enqueue(c);
}

/* Helper commands — shortcuts that expand onto the base commands using black
 * as the second colour. Duration mapping (see Spec.md §6.4):
 *   Flash:  alternate, duration_ms = full period, 50/50 on:off.
 *   Blink:  alternate, duration_ms = full period, ~20% on.
 *   Breath: fade black<->colour, duration_ms = full inhale+exhale cycle.
 *   Pulse:  fade, snap on (fade-in = 0) then fade off over duration_ms.
 */

void OrbtLED::setLedFlash(uint8_t led_id, colour_t colour, uint32_t duration_ms)
{
    colour_t black = {0, 0, 0, 0};
    uint32_t on = duration_ms / 2;
    uint32_t off = duration_ms - on;
    setLedAlternate(led_id, colour, on, black, off);
}

void OrbtLED::setLedBlink(uint8_t led_id, colour_t colour, uint32_t duration_ms)
{
    colour_t black = {0, 0, 0, 0};
    uint32_t on = duration_ms / 5;
    uint32_t off = duration_ms - on;
    setLedAlternate(led_id, colour, on, black, off);
}

void OrbtLED::setLedBreath(uint8_t led_id, colour_t colour, uint32_t duration_ms)
{
    colour_t black = {0, 0, 0, 0};
    uint32_t half = duration_ms / 2;
    setLedFade(led_id, black, half, 0, colour, half, 0);
}

void OrbtLED::setLedPulse(uint8_t led_id, colour_t colour, uint32_t duration_ms)
{
    colour_t black = {0, 0, 0, 0};
    setLedFade(led_id, black, 0, 0, colour, duration_ms, 0);
}

/* --------------------------------------------------------------------------
 * Colour pipeline helpers (see Spec.md §5).
 * ------------------------------------------------------------------------ */

OrbtLED::colour_u32_t OrbtLED::_promote(colour_t c)
{
    colour_u32_t o;
    o.red   = (uint32_t)c.red   << ORBTLED_FIXED_SHIFT;
    o.green = (uint32_t)c.green << ORBTLED_FIXED_SHIFT;
    o.blue  = (uint32_t)c.blue  << ORBTLED_FIXED_SHIFT;
    o.white = (uint32_t)c.white << ORBTLED_FIXED_SHIFT;
    return o;
}

OrbtLED::colour_t OrbtLED::_toOutput(uint8_t led_id, colour_u32_t c)
{
    uint32_t b = _brightness[led_id];
    // out8 = GAMMA_LUT[ (channel * brightness) >> 23 ], clamped to 255.
    auto sc = [&](uint32_t v) -> uint8_t {
        uint32_t lin = (v * b) >> ORBTLED_OUTPUT_SHIFT;
        if(lin > 255) lin = 255;
        return _gammaLut[lin];
    };
    colour_t out;
    out.red   = sc(c.red);
    out.green = sc(c.green);
    out.blue  = sc(c.blue);
    out.white = sc(c.white);
    return out;
}

OrbtLED::colour_s32_t OrbtLED::_fadeStep(colour_u32_t start, colour_u32_t target, uint32_t dur_ms)
{
    uint32_t steps = dur_ms / ORBTLED_BACKGROUND_TASK_INTERVAL_MS;
    if(steps < 1) {
        steps = 1;  // guard sub-tick durations / divide-by-zero
    }
    colour_s32_t s;
    s.red   = ((int32_t)target.red   - (int32_t)start.red)   / (int32_t)steps;
    s.green = ((int32_t)target.green - (int32_t)start.green) / (int32_t)steps;
    s.blue  = ((int32_t)target.blue  - (int32_t)start.blue)  / (int32_t)steps;
    s.white = ((int32_t)target.white - (int32_t)start.white) / (int32_t)steps;
    return s;
}

OrbtLED::colour_u32_t OrbtLED::_fadeColourAt(colour_u32_t start, colour_s32_t step, uint32_t step_number)
{
    auto ch = [](uint32_t s0, int32_t st, uint32_t n) -> uint32_t {
        int32_t v = (int32_t)s0 + st * (int32_t)n;
        if(v < 0) {
            v = 0;
        }
        if(v > ORBTLED_COLOUR_MAX) {
            v = ORBTLED_COLOUR_MAX;
        }
        return (uint32_t)v;
    };
    colour_u32_t o;
    o.red   = ch(start.red,   step.red,   step_number);
    o.green = ch(start.green, step.green, step_number);
    o.blue  = ch(start.blue,  step.blue,  step_number);
    o.white = ch(start.white, step.white, step_number);
    return o;
}

/* --------------------------------------------------------------------------
 * Command processing (consumer side, background task).
 * ------------------------------------------------------------------------ */

void OrbtLED::_applyCommand(const command_t &cmd)
{
    if(cmd.led_id >= _ledCount) {
        return;
    }
    effect_state_t &es = _effect_state[cmd.led_id];
    uint32_t now = millis();

    switch(cmd.command) {
        case command_t::COMMAND_SET_BRIGHTNESS: {
            uint16_t b = cmd.brightness;
            if(b > ORBTLED_BRIGHTNESS_MAX) {
                b = ORBTLED_BRIGHTNESS_MAX;     // clamp keeps the output multiply within uint32 (§5.4)
            }
            _brightness[cmd.led_id] = b;
            break;
        }
        case command_t::COMMAND_SET_LED: {
            es.isNew = true;
            es.effect = cmd.effect;
            switch(cmd.effect) {
                case EFFECT_SOLID:
                    es.colour_current = _promote(cmd.effect_data.solid.colour);
                    break;

                case EFFECT_ALTERNATE: {
                    auto &a = es.effect_state.alternate;
                    a.colour1 = _promote(cmd.effect_data.alternate.colour1);
                    a.duration1_ms = cmd.effect_data.alternate.duration1_ms;
                    a.colour2 = _promote(cmd.effect_data.alternate.colour2);
                    a.duration2_ms = cmd.effect_data.alternate.duration2_ms;
                    a.state = (decltype(a.state))0;     // COLOUR1
                    es.colour_current = a.colour1;
                    a.stateEndTime_ms = now + a.duration1_ms;
                    break;
                }

                case EFFECT_FADE: {
                    auto &f = es.effect_state.fade;
                    f.colour1 = _promote(cmd.effect_data.fade.colour1);
                    f.fade1_duration_ms = cmd.effect_data.fade.fade1_duration_ms;
                    f.pause1_duration_ms = cmd.effect_data.fade.pause1_duration_ms;
                    f.colour2 = _promote(cmd.effect_data.fade.colour2);
                    f.fade2_duration_ms = cmd.effect_data.fade.fade2_duration_ms;
                    f.pause2_duration_ms = cmd.effect_data.fade.pause2_duration_ms;
                    _fadeBeginSubstate(es, 1, now);     // start in COLOUR1_FADE (colour1 -> colour2)
                    break;
                }
            }
            break;
        }

        default:
            break;
    }
}

void OrbtLED::_backgroundProcessCommands(void)
{
    command_t command;
    while(xQueueReceive(_commandQueueHandle, &command, 0) == pdPASS) {
        _applyCommand(command);
    }
}

/* --------------------------------------------------------------------------
 * Effect engines.
 * Sub-state ints (match FadeEffectState order):
 *   0 = COLOUR1_PAUSE (hold colour1, pause2)   1 = COLOUR1_FADE (c1 -> c2, fade1)
 *   2 = COLOUR2_PAUSE (hold colour2, pause1)   3 = COLOUR2_FADE (c2 -> c1, fade2)
 * Cycle: 1 -> 2 -> 3 -> 0 -> 1 ...
 * ------------------------------------------------------------------------ */

void OrbtLED::_fadeBeginSubstate(effect_state_t &es, int substate, uint32_t base_ms)
{
    auto &f = es.effect_state.fade;
    f.state = (decltype(f.state))substate;

    uint32_t dur;
    switch(substate) {
        case 1: // COLOUR1_FADE: colour1 -> colour2
            dur = f.fade1_duration_ms;
            es.colour_current = f.colour1;
            f.colour_step = _fadeStep(f.colour1, f.colour2, dur);
            break;
        case 2: // COLOUR2_PAUSE: hold colour2
            dur = f.pause1_duration_ms;
            es.colour_current = f.colour2;
            break;
        case 3: // COLOUR2_FADE: colour2 -> colour1
            dur = f.fade2_duration_ms;
            es.colour_current = f.colour2;
            f.colour_step = _fadeStep(f.colour2, f.colour1, dur);
            break;
        case 0: // COLOUR1_PAUSE: hold colour1
        default:
            dur = f.pause2_duration_ms;
            es.colour_current = f.colour1;
            break;
    }
    f.stateEndTime_ms = base_ms + dur;
}

void OrbtLED::_updateAlternate(uint8_t led_id, uint32_t now_ms)
{
    effect_state_t &es = _effect_state[led_id];
    auto &a = es.effect_state.alternate;

    for(int guard = 0; guard < 8; ++guard) {
        if((int32_t)(now_ms - a.stateEndTime_ms) < 0) {
            break;  // current sub-state still active
        }
        if((int)a.state == 0) {     // COLOUR1 -> COLOUR2
            a.state = (decltype(a.state))1;
            es.colour_current = a.colour2;
            a.stateEndTime_ms += (a.duration2_ms ? a.duration2_ms : 1);
        } else {                    // COLOUR2 -> COLOUR1
            a.state = (decltype(a.state))0;
            es.colour_current = a.colour1;
            a.stateEndTime_ms += (a.duration1_ms ? a.duration1_ms : 1);
        }
    }
}

void OrbtLED::_updateFade(uint8_t led_id, uint32_t now_ms)
{
    effect_state_t &es = _effect_state[led_id];
    auto &f = es.effect_state.fade;

    // Advance any expired sub-state(s), chaining end times for drift-free timing.
    for(int guard = 0; guard < 8; ++guard) {
        int st = (int)f.state;
        bool isPause = (st == 0 || st == 2);
        uint32_t dur;
        switch(st) {
            case 0:  dur = f.pause2_duration_ms; break;
            case 1:  dur = f.fade1_duration_ms;  break;
            case 2:  dur = f.pause1_duration_ms; break;
            default: dur = f.fade2_duration_ms;  break;
        }
        if(isPause && dur == ORBTLED_DURATION_MAX) {
            break;  // hold forever (fade-in / fade-out)
        }
        if((int32_t)(now_ms - f.stateEndTime_ms) < 0) {
            break;  // current sub-state still active
        }
        _fadeBeginSubstate(es, (st + 1) & 0x3, f.stateEndTime_ms);
    }

    // Compute the working colour for the current sub-state.
    int st = (int)f.state;
    if(st == 1 || st == 3) { // FADE: recompute via Formula 2 (multiplication)
        uint32_t dur = (st == 1) ? f.fade1_duration_ms : f.fade2_duration_ms;
        uint32_t steps = dur / ORBTLED_BACKGROUND_TASK_INTERVAL_MS;
        if(steps < 1) {
            steps = 1;
        }
        uint32_t start_ms = f.stateEndTime_ms - dur;
        uint32_t step_number = (now_ms - start_ms) / ORBTLED_BACKGROUND_TASK_INTERVAL_MS;
        if(step_number > steps) {
            step_number = steps;
        }
        colour_u32_t start = (st == 1) ? f.colour1 : f.colour2;
        es.colour_current = _fadeColourAt(start, f.colour_step, step_number);
    }
    // PAUSE states keep colour_current as set on sub-state entry.
}

bool OrbtLED::_backgroundUpdateLeds(void)
{
    bool changed = false;
    uint32_t now = millis();

    for(uint8_t i = 0; i < _ledCount; ++i) {
        effect_state_t &es = _effect_state[i];

        switch(es.effect) {
            case EFFECT_SOLID:                          break;  // colour_current is constant
            case EFFECT_ALTERNATE: _updateAlternate(i, now);    break;
            case EFFECT_FADE:      _updateFade(i, now);         break;
        }

        colour_t out = _toOutput(i, es.colour_current);

        // Render gating: only rewrite the pixel when the 8-bit output changed.
        if(out.red   != es.current_colour_u8.red   ||
           out.green != es.current_colour_u8.green ||
           out.blue  != es.current_colour_u8.blue  ||
           out.white != es.current_colour_u8.white) {
            es.current_colour_u8 = out;
            _leds->SetPixelColor(i, RgbwColor(out.red, out.green, out.blue, out.white));
            changed = true;
        }
        es.isNew = false;
    }

    return changed;
}

void OrbtLED::_backgroundTaskHandler(void)
{
    _lastWakeTime = xTaskGetTickCount();
    while(1) {
        xTaskDelayUntil(&_lastWakeTime, pdMS_TO_TICKS(ORBTLED_BACKGROUND_TASK_INTERVAL_MS));

        _backgroundProcessCommands();
        bool changed = _backgroundUpdateLeds();

        // Show() on any change, plus a periodic forced refresh to re-latch the strip.
        uint32_t now = millis();
        bool forced = ((uint32_t)(now - _lastShowTime_ms) >= ORBTLED_FORCED_REFRESH_MS);
        if(changed || forced) {
            _leds->Show();
            _lastShowTime_ms = now;
        }
    }
}
