#include "pid.h"

void PID::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

float PID::compute(float target, float current) {
    unsigned long now = millis();
    float error = target - current;

    if (lastTime == 0) {
        lastTime = now;
        lastError = error;
        lastInput = current;
        hasPrevSample = true;
        return 0; // First run has no valid dt yet.
    }

    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;

    if (dt > 0) {
        integral += error * dt;
    }

    // Anti-windup: clamp integral term
    if (ki != 0) {
        float iTerm = integral * ki;
        if (max_i != 0) {
            if (iTerm > max_i) integral = max_i / ki;
            else if (iTerm < -max_i) integral = -max_i / ki;
        }
    }

    // Derivative on measurement avoids D kick from target step changes.
    float derivative = (dt > 0) ? -(current - lastInput) / dt : 0;
    float output = kp * error + ki * integral + kd * derivative;
    if (max_output > 0) output = constrain(output, -max_output, max_output);
    lastError = error;
    lastInput = current;
    hasPrevSample = true;

    return output;
}

float PID::compute(float dt, float target, float current) {
    float error = target - current;

    if (!hasPrevSample) {
        lastError = error;
        lastInput = current;
        hasPrevSample = true;
    }

    if (dt > 0) {
        integral += error * dt;
    }

    // Anti-windup: clamp integral term
    if (ki != 0) {
        float iTerm = integral * ki;
        if (max_i != 0) {
            if (iTerm > max_i) integral = max_i / ki;
            else if (iTerm < -max_i) integral = -max_i / ki;
        }
    }

    // Derivative on measurement avoids D kick from target step changes.
    float derivative = (dt > 0) ? -(current - lastInput) / dt : 0;
    float output = kp * error + ki * integral + kd * derivative;
    if (max_output > 0) output = constrain(output, -max_output, max_output);
    lastError = error;
    lastInput = current;

    return output;
}

float PID::compute(float dt, float target, float current, float derivative) {
    float error = target - current;

    if (!hasPrevSample) {
        lastError = error;
        lastInput = current;
        hasPrevSample = true;
        derivative = 0;
    }

    if (dt > 0) {
        integral += error * dt;
    }

    // Anti-windup: clamp integral term
    if (ki != 0) {
        float iTerm = integral * ki;
        if (max_i != 0) {
            if (iTerm > max_i) integral = max_i / ki;
            else if (iTerm < -max_i) integral = -max_i / ki;
        }
    }

    float output = kp * error + ki * integral + kd * derivative;
    if (max_output > 0) output = constrain(output, -max_output, max_output);
    lastError = error;
    lastInput = current;

    return output;
}
