#include "hal_battery.h"

HAL_Battery::HAL_Battery(uint8_t pin, float factor) : _pin(pin), _factor(factor) {}

void HAL_Battery::init() {
    pinMode(_pin, INPUT);
    // Initial read for the starting voltage
    _voltage = analogRead(_pin) * _factor;
}

void HAL_Battery::update(float dt) {
    // Current raw reading
    float raw_v = (float)analogRead(_pin) * _factor;
    
    // Simple LPF (exponential moving average)
    // _alpha = 0.1 means 10% new value, 90% old value
    // _voltage = _voltage * (1.0f - _alpha) + raw_v * _alpha;
    
    // Time-dependent LPF (to be more robust for varying dt)
    // float cutoff_freq_hz = 0.5f; // low cutoff frequency for battery
    // float tau = 1.0f / (2.0f * PI * cutoff_freq_hz);
    // float alpha_dt = dt / (tau + dt);
    
    // For now we use the simple one since dt is fairly constant in Comm_Task or Control_Task
    _voltage = _voltage * (1.0f - _alpha) + raw_v * _alpha;
}
