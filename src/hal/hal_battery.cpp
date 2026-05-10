#include "hal_battery.h"

HAL_Battery::HAL_Battery(uint8_t pin, float factor) : _pin(pin), _factor(factor) {}

void HAL_Battery::init() {
    pinMode(_pin, INPUT);
    // Set attenuation to 11dB to allow measuring up to ~3.1V
    analogSetAttenuation(ADC_11db);
    // Initial read for the starting voltage
    _voltage = analogRead(_pin) * _factor;
}

void HAL_Battery::update(float dt) {
    // Current raw reading
    float raw_v = (float)analogRead(_pin) * _factor;
    _voltage = _voltage * (1.0f - _alpha) + raw_v * _alpha;
}
