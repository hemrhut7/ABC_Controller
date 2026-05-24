#include "pid.h"

void PID::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    // Recalculate the anti-windup limit using the new I gain on next compute().
    max_i = 0;
}

float PID::compute(float target, float current) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;

    // Check the target is valid first if ramp is set
    float currentTarget = target;
    if (ramp > 0) {
        float max_diff = ramp * dt;
        float target_diff = target - lastTarget;

        if (target_diff > 0 && target_diff > max_diff) currentTarget = lastTarget + max_diff;
        else if (target_diff < 0 && target_diff < -max_diff) currentTarget = lastTarget - max_diff;
    }

    // Calculate error by limitted target
    float error = currentTarget - current;

     // First run has no valid dt yet
    if (lastTime == 0) {
        lastTime = now;
        lastError = error;
        lastInput = current;
        lastTarget = currentTarget;
        hasPrevSample = true;
        return 0;
    }

    // Integral
    if (dt > 0) integral += error * dt;

    // Anti-windup: clamp integral term
    // Use max_output-based limit to prevent integral from exceeding useful range
    if (ki != 0) {
        float iTerm = integral * ki;
        if (max_i <= 0) max_i = (max_output > 0) ? max_output : 10.0f * abs(ki);
        if (iTerm > max_i) integral = max_i / ki;
        else if (iTerm < -max_i) integral = -max_i / ki;
    }

    // Derivative on measurement avoids D kick from target step changes.
    float derivative = (dt > 0) ? -(current - lastInput) / dt : 0;

    // Calculate final output
    float output = kp * error + ki * integral + kd * derivative;
    if (max_output > 0) output = constrain(output, -max_output, max_output);

    // Record data
    lastError = error;
    lastInput = current;
    lastTarget = currentTarget;
    hasPrevSample = true;
    lastTime = now;

    return output;
}

float PID::compute(float dt, float target, float current) {
    // Check the target is valid first if ramp is set
    float currentTarget = target;
    if (ramp > 0) {
        float max_diff = ramp * dt;
        float target_diff = target - lastTarget;

        if (target_diff > 0 && target_diff > max_diff) currentTarget = lastTarget + max_diff;
        else if (target_diff < 0 && target_diff < -max_diff) currentTarget = lastTarget - max_diff;
    }

    // Calculate error by limitted target
    float error = currentTarget - current;

    if (!hasPrevSample) {
        lastError = error;
        lastInput = current;
        lastTarget = currentTarget;
        hasPrevSample = true;
    }

    // Integral
    if (dt > 0) integral += error * dt;

    // Anti-windup: clamp integral term
    // Use max_output-based limit to prevent integral from exceeding useful range
    if (ki != 0) {
        float iTerm = integral * ki;
        if (max_i <= 0) max_i = (max_output > 0) ? max_output : 10.0f * abs(ki);
        if (iTerm > max_i) integral = max_i / ki;
        else if (iTerm < -max_i) integral = -max_i / ki;
    }

    // Derivative on measurement avoids D kick from target step changes.
    float derivative = (dt > 0) ? -(current - lastInput) / dt : 0;

    // Calculate final output
    float output = kp * error + ki * integral + kd * derivative;
    if (max_output > 0) output = constrain(output, -max_output, max_output);

    // Record data
    lastError = error;
    lastInput = current;
    lastTarget = currentTarget;

    return output;
}

float PID::compute(float dt, float target, float current, float derivative) {
    // Check the target is valid first if ramp is set
    float currentTarget = target;
    if (ramp > 0) {
        float max_diff = ramp * dt;
        float target_diff = target - lastTarget;

        if (target_diff > 0 && target_diff > max_diff) currentTarget = lastTarget + max_diff;
        else if (target_diff < 0 && target_diff < -max_diff) currentTarget = lastTarget - max_diff;
    }

    // Calculate error by limitted target
    float error = currentTarget - current;

    if (!hasPrevSample) {
        lastError = error;
        lastInput = current;
        lastTarget = currentTarget;
        hasPrevSample = true;
        derivative = 0;
    }

    // Integral
    if (dt > 0) integral += error * dt;

    // Anti-windup: clamp integral term
    // Use max_output-based limit to prevent integral from exceeding useful range
    if (ki != 0) {
        float iTerm = integral * ki;
        if (max_i <= 0) max_i = (max_output > 0) ? max_output : 10.0f * abs(ki);
        if (iTerm > max_i) integral = max_i / ki;
        else if (iTerm < -max_i) integral = -max_i / ki;
    }

    // Calculate final output
    float output = kp * error + ki * integral + kd * derivative;
    if (max_output > 0) output = constrain(output, -max_output, max_output);

    // Record data
    lastError = error;
    lastInput = current;
    lastTarget = currentTarget;

    return output;
}
