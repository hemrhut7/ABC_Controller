#pragma once


#include "pid.h"


class Rate_Controller {
    public:
        Rate_Controller(float p, float i, float d): pid(p, i, d) {}
        float set_target(float target, float current);
        void setOutputLimits(float min, float max);
        void reset();
    private:
        PID pid;
        float _target = 0;
        float _current = 0;
};

class Pitch_Controller {
    public:
        Pitch_Controller(float p, float i, float d): pid(p, i, d) {}
        float set_target(float target, float current);
        void setOutputLimits(float min, float max);
        void reset();
    private:
        PID pid;        
        float _target = 0;
        float _current = 0;
};

class Velocity_Controller {
    public:
        Velocity_Controller(float p, float i, float d): pid(p, i, d) {}
        float set_target(float target, float current);
        void setOutputLimits(float min, float max);
        void reset();
    private:
        PID pid;
        float _target = 0;
        float _current = 0;
};

class Yaw_Controller {
    public:
        Yaw_Controller(float p, float i, float d): pid(p, i, d) {}
        float set_target(float target, float current);
        void setOutputLimits(float min, float max);
        void reset();
    private:
        PID pid;
        float _target = 0;
        float _current = 0;
};