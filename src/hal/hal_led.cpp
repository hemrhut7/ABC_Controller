#include "hal_led.h"

HAL_LED::HAL_LED(uint8_t pin) : pin(pin) {
    current_state = STOP;
    last_toggle_time = 0;
    blink_interval = 0;
    init();
}

void HAL_LED::init() {
    pinMode(pin, OUTPUT);
    off(); // Initialize LED to off state
}

void HAL_LED::on() {
    digitalWrite(pin, HIGH);
}

void HAL_LED::off() {
    digitalWrite(pin, LOW);
}

void HAL_LED::toggle() {
    digitalWrite(pin, !digitalRead(pin));
}

void HAL_LED::set_state(SYSTEM_STATE state) {
    if (current_state == state) return;

    current_state = state;
    switch (state) {
        case INITIALIZING:
            blink_interval = 100;
            break;
        case WORKING:
            blink_interval = 1000;
            break;
        default:
            blink_interval = 0;
            off();
            break;
    }
}

void HAL_LED::update() {
    if (blink_interval > 0 && (millis() - last_toggle_time >= blink_interval)) {
        toggle();
        last_toggle_time = millis();
    }
}
