#pragma once

#include "hal/hal_motor.h"
#include "hal/hal_type_define.h"

class Processing_Motor {
public:
    Processing_Motor(uint32_t period_ms);
    ~Processing_Motor();

    void init();
    void set_target_rpms(int left_rpm, int right_rpm);
    void set_enable(bool enable);
    void update_rpms();
    void get_motor_state(motor_state_t *state);

private:
    HAL_Motor motor_l;
    HAL_Motor motor_r;
};