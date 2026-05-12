#pragma once

#include <Arduino.h>


typedef enum {
  INITIALIZING,   // 快閃
  ARMED,        // 慢閃
  DISARMED        // 解鎖狀態
} SYSTEM_STATE;

class HAL_LED {
public:
    HAL_LED();
    void init();
    void on();
    void off();
    void toggle();
    void set_state(SYSTEM_STATE state);
    void update();
    SYSTEM_STATE get_state() const { return current_state; }

private:
    uint8_t pin;
    SYSTEM_STATE current_state;
    uint32_t last_toggle_time;
    uint32_t blink_interval;
};
