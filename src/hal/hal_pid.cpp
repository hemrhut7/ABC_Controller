#include "hal_pid.h"

void PID::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

float PID::compute(float target, float current) {
    float now = millis();
    float dt = (float)(now - lastTime) / 1000.0;
    lastTime = now;

    float error = target - current;
    integral += error * dt;
    float derivative = (dt > 0) ? (error - lastError) / dt : 0;
    float output = kp * error + ki * integral + kd * derivative;
    lastError = error;

    return output;
}


float PID::compute(float dt, float target, float current) {
    float error = target - current;
    integral += error * dt;
    float derivative = (dt > 0) ? (error - lastError) / dt : 0;
    float output = kp * error + ki * integral + kd * derivative;
    lastError = error;

    return output;
}

float PID::compute(float dt, float target, float current, float derivative) {
    float error = target - current;
    integral += error * dt;
    float output = kp * error + ki * integral + kd * derivative;
    lastError = error;

    return output;
}