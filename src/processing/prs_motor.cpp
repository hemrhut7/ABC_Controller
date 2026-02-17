#include "prs_motor.h"

Processing_Motor::Processing_Motor(uint32_t period_ms) : motor_l(LEFT_MOTOR, period_ms), motor_r(RIGHT_MOTOR, period_ms) {
    // Constructor initializes the motors using member initializer list
}

Processing_Motor::~Processing_Motor() {
    // Destructor
}

void Processing_Motor::init() {
    setupPCNT(); // This sets up the pulse counters for both motors.
    set_target_rpms(0, 0);
}

void Processing_Motor::set_target_rpms(int left_rpm, int right_rpm) {
    motor_l.set_target_rpm(left_rpm);
    motor_r.set_target_rpm(right_rpm);
}

void Processing_Motor::set_enable(bool enable) {
    motor_l.set_enable(enable);
    motor_r.set_enable(enable);
}

void Processing_Motor::set_pwm(int left_pwm, int right_pwm) {
    motor_l.drive_moter(left_pwm);
    motor_r.drive_moter(right_pwm);
}


void Processing_Motor::update_rpms(float dt) {
    motor_l.update_rpm(dt);
    motor_r.update_rpm(dt);
}


void Processing_Motor::get_motor_state(motor_state_t *state) {
    state->rpm_L = motor_l.get_current_rpm();
    state->rpm_R = motor_r.get_current_rpm();
    state->target_rpm_L = motor_l.get_target_rpm();
    state->target_rpm_R = motor_r.get_target_rpm();
}

void Processing_Motor::set_pid_gains(float p, float i, float d) {
    motor_l.set_pid_gains(p, i, d);
    motor_r.set_pid_gains(p, i, d);
}