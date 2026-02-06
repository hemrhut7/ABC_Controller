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

int Processing_Motor::get_left_rpm() {
    return motor_l.get_current_rpm();
}

int Processing_Motor::get_right_rpm() {
    return motor_r.get_current_rpm();
}
