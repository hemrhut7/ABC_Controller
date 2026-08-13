#include "hal_battery.h"

static const BatteryPoint battery_lut[] = {
    {12.60f, 100},
    {12.30f,  95},
    {12.00f,  85},
    {11.70f,  75},
    {11.40f,  60},
    {11.10f,  50},
    {10.80f,  40},
    {10.50f,  30},
    {10.20f,  20},
    { 9.90f,  15},
    { 9.60f,  10},
    { 9.30f,   5},
    { 9.00f,   0}
};

static const size_t lut_size = sizeof(battery_lut) / sizeof(battery_lut[0]);

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

uint8_t HAL_Battery::voltage_to_percentage(float voltage) {
    if (voltage >= battery_lut[0].voltage) return 100;
    if (voltage <= battery_lut[lut_size - 1].voltage) return 0;

    for (size_t i = 0; i < lut_size - 1; i++) {
        if (voltage <= battery_lut[i].voltage && voltage >= battery_lut[i + 1].voltage) {
            float v_high = battery_lut[i].voltage;
            float v_low = battery_lut[i + 1].voltage;
            uint8_t p_high = battery_lut[i].percentage;
            uint8_t p_low = battery_lut[i + 1].percentage;

            // 線性內插計算
            return (uint8_t)(p_low + (voltage - v_low) * (p_high - p_low) / (v_high - v_low));
        }
    }
    return 0;
}
