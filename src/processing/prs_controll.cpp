#include "prs_controll.h"

// Rate_Controller Implementation
float Rate_Controller::set_target(float target, float current) {
    _target = target;
    _current = current;
    return pid.compute(_target, _current);
}

void Rate_Controller::setOutputLimits(float min, float max) {
    pid.setOutputLimits(min, max);
}

void Rate_Controller::reset() {
    pid.reset();
    _target = 0;
    _current = 0;
}

// Pitch_Controller Implementation
float Pitch_Controller::set_target(float target, float current) {
    _target = target;
    _current = current;
    return pid.compute(_target, _current);
}

void Pitch_Controller::setOutputLimits(float min, float max) {
    pid.setOutputLimits(min, max);
}

void Pitch_Controller::reset() {
    pid.reset();
    _target = 0;
    _current = 0;
}

// Velocity_Controller Implementation
float Velocity_Controller::set_target(float target, float current) {
    _target = target;
    _current = current;
    return pid.compute(_target, _current);
}

void Velocity_Controller::setOutputLimits(float min, float max) {
    pid.setOutputLimits(min, max);
}

void Velocity_Controller::reset() {
    pid.reset();
    _target = 0;
    _current = 0;
}

// Yaw_Controller Implementation
float Yaw_Controller::set_target(float target, float current) {
    _target = target;
    _current = current;
    return pid.compute(_target, _current);
}

void Yaw_Controller::setOutputLimits(float min, float max) {
    pid.setOutputLimits(min, max);
}

void Yaw_Controller::reset() {
    pid.reset();
    _target = 0;
    _current = 0;
}