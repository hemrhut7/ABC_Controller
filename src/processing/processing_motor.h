#pragma once

#include "hal/hal_motor.h"

class Processing_Motor {
public:
    Processing_Motor();
    ~Processing_Motor();

    void init();
    void set_target_rpms(int left_rpm, int right_rpm);
    void update_rpms();
    int get_left_rpm();
    int get_right_rpm();

private:
    HAL_Motor motor_l;
    HAL_Motor motor_r;
};