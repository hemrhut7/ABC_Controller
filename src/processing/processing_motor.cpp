#include "processing_motor.h"

Processing_Motor::Processing_Motor() : motor_l(LEFT_MOTOR), motor_r(RIGHT_MOTOR) {
    // Constructor initializes the motors using member initializer list
}

Processing_Motor::~Processing_Motor() {
    // Destructor
}

void Processing_Motor::init() {
    // The HAL_Motor objects are already initialized by their constructors.
    // We can add additional setup here if needed.
    setupPCNT(); // This sets up the pulse counters for both motors.
}

void Processing_Motor::set_target_rpms(int left_rpm, int right_rpm) {
    motor_l.set_target_rpm(left_rpm);
    motor_r.set_target_rpm(right_rpm);
}

void Processing_Motor::update_rpms() {
    motor_l.update_rpm();
    motor_r.update_rpm();
}


void Processing_Motor::get_motor_state(motor_state_t *state) {
    state->rpm_L = motor_l.get_current_rpm();
    state->rpm_R = motor_r.get_current_rpm();
    state->target_rpm_L = motor_l.get_target_rpm();
    state->target_rpm_R = motor_r.get_target_rpm();
}