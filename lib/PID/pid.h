#pragma once 

#include "Arduino.h"


class PID {
    public:
        PID(){};
        PID(float kp, float ki, float kd, float ramp, float max_i, float max_output) : kp(kp), ki(ki), kd(kd), ramp(ramp), max_i(max_i), max_output(max_output) {}
        ~PID(){};

        void setTunings(float kp, float ki, float kd);
        void setOutputLimits(float max_output) { this->max_output = max_output; }
        void setRamp(float ramp) { this->ramp = ramp; }
        void reset() { lastError = 0; integral = 0; lastTime = 0; lastInput = 0; lastTarget = 0; max_i = 0; hasPrevSample = false; }
        float compute(float target, float current);
        float compute(float dt, float target, float current);
        float compute(float dt, float target, float current, float derivative);

    private:
        float kp = 1, ki = 1.0f/200.0f, kd = 0.1;
        float lastError = 0;
        float lastInput = 0;
        float lastTarget = 0;
        unsigned long lastTime = 0;
        float integral = 0;
        float max_i = 0;
        float max_output = 0;
        float ramp = 0;
        bool hasPrevSample = false;
};
