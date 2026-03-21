#pragma once

#include <Arduino.h>
#include <driver/pcnt.h>
#include "pid.h"
#include "LPF.h"


constexpr int PPR = 500;           // 每轉脈衝數 (300線)
constexpr float GEAR_RATIO = 30.0f; // 減速比
constexpr int QUADRATURE = 4;      // 4倍頻 (2Pin * CHANGE)
constexpr float CPR = PPR * QUADRATURE * GEAR_RATIO; // 每轉計數 (500 * 1 * 30 = 15000)
constexpr float DEG_PER_CNT = 360.0f / CPR;
constexpr float DPS_2_RPM = 60.0f / 360.0f;
constexpr int PWM_FREQ = 20000;
constexpr int PWM_RES = 10;
constexpr int MAX_PWM_DUTY = (1 << PWM_RES) - 1; // 1023


#define MOTOR_DEADZONE 55
#define MOTOR_DITHER 40
#define MOTOR_KV       0.3525f
#define MAX_MOTOR_RPM 160
#define MAX_MOTOR_RPM_RATE 2500

#define MOTOR_L_DIR1_PIN    5
#define MOTOR_L_DIR2_PIN    18
#define MOTOR_L_PWM_PIN     19
#define MOTOR_L_DTBY_PIN    21
#define MOTOR_L_E1A_PIN     32
#define MOTOR_L_E1B_PIN     33

#define MOTOR_R_DIR1_PIN    4
#define MOTOR_R_DIR2_PIN    0
#define MOTOR_R_PWM_PIN     15
#define MOTOR_R_DTBY_PIN    MOTOR_L_DTBY_PIN
#define MOTOR_R_E2A_PIN     25
#define MOTOR_R_E2B_PIN     26


enum MotorPosition {
    LEFT_MOTOR,
    RIGHT_MOTOR
};


class HAL_Motor {
    public:
        // HAL_Motor(uint8_t pwm_pin, uint8_t dir_pin1, uint8_t dir_pin2, uint8_t stdy_pin,  uint8_t enc_a_pin, uint8_t enc_b_pin);
        HAL_Motor(MotorPosition position, uint32_t period_ms);
        ~HAL_Motor();
        void set_target_rpm(float target_rpm);
        void update_rpm(float dt);
        void setup_motor_dir(bool dir_forward);
        void set_pid_gains(float p, float i, float d);
        void set_enable(bool enable);
        float get_rpm() { return current_rpm;}
        float get_target_rpm() { return last_target_rpm; }
        float get_pwm_out() { return last_pwm_out; }
        void drive_moter(int pwm);
        void resetPID() { pid.reset(); }

    private:
        uint8_t pwm_pin;
        uint8_t dir_pin1;
        uint8_t dir_pin2;
        uint8_t stdy_pin;
        uint8_t enc_a_pin;
        uint8_t enc_b_pin;
        uint16_t update_rate_hz = 100;
        float dt = 1e-2f;
        float MAX_MOTOR_DELTA_RPM = MAX_MOTOR_RPM_RATE * dt;

        volatile uint32_t last_time = 0;
        int16_t count;
        int last_pwm_out = 0;
        float last_target_rpm = 0;
        float last_rpm = 0;
        float current_rpm = 0;

        PID pid;
        int dir_forward = 1;
        pcnt_unit_t pcnt_uint = PCNT_UNIT_0;  // Default to 0 for left motor, 1 for right motor
        uint8_t pwm_channel = 0;
        bool is_enable = true;
        int8_t dither_dir = 1;

        void init();
        void getPCNTCount();
};

void setupPCNT();