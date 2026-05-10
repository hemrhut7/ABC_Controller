#include "hal_battery.h"

HAL_Battery::HAL_Battery(uint8_t pin, float slope, float offset) 
    : _pin(pin), _slope(slope), _offset(offset) {}

void HAL_Battery::init() {
    pinMode(_pin, INPUT);
    // Set attenuation to 11dB to allow measuring up to ~3.1V
    analogSetAttenuation(ADC_11db);
    // Initial read for the starting voltage
    int raw = analogRead(_pin);
    _voltage = (float)raw * _slope + _offset;
    if (_voltage < 0) _voltage = 0;
}

void HAL_Battery::update(float dt) {
    // Current raw reading
    int raw = analogRead(_pin);
    float raw_v = (float)raw * _slope + _offset;
    if (raw_v < 0) raw_v = 0;
    
    _voltage = _voltage * (1.0f - _alpha) + raw_v * _alpha;
}
