#pragma once 

#include "Arduino.h"


class PID {
    public:
        PID(){};
        PID(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd){}
        ~PID(){};

        void setTunings(float kp, float ki, float kd);
        void setOutputLimits(float min, float max);
        void reset() { lastError = 0; integral = 0; lastTime = 0; }
        float compute(float target, float current);
        float compute(float dt, float target, float current);
        float compute(float dt, float target, float current, float derivative);

    private:
        float kp = 1, ki = 1/200, kd = 0.1;
        float lastError = 0;
        unsigned long lastTime = 0;
        float integral = 0;
        float MAX_iTerm = 0;
};
