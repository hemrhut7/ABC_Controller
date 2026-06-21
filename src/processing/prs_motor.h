#pragma once

#include "hal/hal_motor.h"
#include "hal/hal_type_define.h"

class Processing_Motor {
public:
    Processing_Motor(float period_ms);
    ~Processing_Motor();

    void init();
    void reset();
    void set_target_rpms(float left_rpm, float right_rpm);
    void set_enable(bool enable);
    void set_pwm(int left_pwm, int right_pwm);
    void update_rpms(float dt);
    void get_motor_state(motor_state_t *state);
    void set_pid_gains(float p, float i, float d);
    int16_t get_right_wheel_count() const { return motor_r.get_count(); }

private:
    HAL_Motor motor_l;
    HAL_Motor motor_r;
};
