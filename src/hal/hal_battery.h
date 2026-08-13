#pragma once

#include <Arduino.h>

#include "config.h"

#define BAT_V_SLOPE   0.00680089f
#define BAT_V_OFFSET  3.43306268f

typedef struct {
    float voltage;
    uint8_t percentage;
} BatteryPoint;

class HAL_Battery {
public:
    /**
     * @brief Constructor
     * @param pin ADC pin to read from
     * @param factor Calibration factor (V = raw * factor)
     */
    HAL_Battery(uint8_t pin = BAT_ADC_PIN, float slope = BAT_V_SLOPE, float offset = BAT_V_OFFSET);

    /**
     * @brief Initialize the ADC pin
     */
    void init();

    /**
     * @brief Update the voltage reading with LPF
     * @param dt Time delta since last update (seconds)
     */
    void update(float dt);

    /**
     * @brief Get the filtered voltage
     * @return float filtered voltage in Volts
     */
    float get_voltage() const { return _voltage; }

    /**
     * @brief Get the battery percentage
     * @return uint8_t battery percentage (0-100)
     */
    uint8_t get_percentage() const { return voltage_to_percentage(_voltage); }

    /**
     * @brief Convert voltage to battery percentage using LUT linear interpolation
     * @param voltage Input voltage in Volts
     * @return uint8_t battery percentage (0-100)
     */
    static uint8_t voltage_to_percentage(float voltage);

private:
    uint8_t _pin;
    float _slope;
    float _offset;
    float _voltage = 0.0f;
    float _alpha = 0.01f; // LPF alpha, lower = smoother
};
