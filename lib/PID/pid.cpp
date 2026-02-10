#include "pid.h"

void PID::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PID::setOutputLimits(float min, float max) {
    if (min >= max) return;
    outMin = min;
    outMax = max;
}

float PID::compute(float target, float current) {
    unsigned long now = millis();
    if (lastTime == 0) {
        lastTime = now;
        return 0; // 避免第一次執行時 dt 過大導致積分暴衝
    }
    
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    float error = target - current;
    integral += error * dt;

    // Anti-windup: 限制積分項
    if (ki != 0) {
        float iTerm = integral * ki;
        if (iTerm > outMax) integral = outMax / ki;
        else if (iTerm < outMin) integral = outMin / ki;
    }

    float derivative = (dt > 0) ? (error - lastError) / dt : 0;
    float output = kp * error + ki * integral + kd * derivative;

    if (output > outMax) output = outMax;
    else if (output < outMin) output = outMin;

    lastError = error;

    return output;
}


float PID::compute(float dt, float target, float current) {
    float error = target - current;
    integral += error * dt;

    // Anti-windup: 限制積分項
    if (ki != 0) {
        float iTerm = integral * ki;
        if (iTerm > outMax) integral = outMax / ki;
        else if (iTerm < outMin) integral = outMin / ki;
    }

    float derivative = (dt > 0) ? (error - lastError) / dt : 0;
    float output = kp * error + ki * integral + kd * derivative;

    if (output > outMax) output = outMax;
    else if (output < outMin) output = outMin;

    lastError = error;

    return output;
}

float PID::compute(float dt, float target, float current, float derivative) {
    float error = target - current;
    integral += error * dt;

    // Anti-windup: 限制積分項
    if (ki != 0) {
        float iTerm = integral * ki;
        if (iTerm > outMax) integral = outMax / ki;
        else if (iTerm < outMin) integral = outMin / ki;
    }

    float output = kp * error + ki * integral + kd * derivative;

    if (output > outMax) output = outMax;
    else if (output < outMin) output = outMin;

    lastError = error;

    return output;
}