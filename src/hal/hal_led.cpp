#include "hal_led.h"

HAL_LED::HAL_LED() 
    : current_state(DISARMED), last_toggle_time(0), blink_interval(500) {
}

void HAL_LED::init() {
    // Virtual LED, no GPIO init
}

void HAL_LED::on() {
    // NOP
}

void HAL_LED::off() {
    // NOP
}

void HAL_LED::toggle() {
    // NOP
}

void HAL_LED::set_state(SYSTEM_STATE state) {
    current_state = state;
    switch (state) {
        case INITIALIZING:
            blink_interval = 100;
            break;
        case ARMED:
            blink_interval = 1000;
            break;
        case DISARMED:
            blink_interval = 0; // solid off or on?
            break;
    }
}

void HAL_LED::update() {
    // Logic for virtual blink if needed, but display will read state
}
