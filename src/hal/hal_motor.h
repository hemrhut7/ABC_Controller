#pragma once

#include <Arduino.h>
#include <driver/pcnt.h>
#include "pid.h"


const int PPR = 500;           // 每轉脈衝數 (300線)
const float GEAR_RATIO = 30.0; // 減速比
const int QUADRATURE = 1;      // 1倍頻 (1Pin * RISING)
const float CPR = PPR * QUADRATURE * GEAR_RATIO; // 每轉計數 (500 * 1 * 30 = 15000)
const float DEG_PER_CNT = 360.0f / CPR;
const float DPS_2_RPM = 60.0f / 360.0f;
const int PWM_FREQ = 20000;
const int PWM_RES = 8;

#define MOTOR_L_DIR1_PIN    5
#define MOTOR_L_DIR2_PIN    18
#define MOTOR_L_PWM_PIN     19
#define MOTOR_L_DTBY_PIN    21
#define MOTOR_L_E1A_PIN     32
#define MOTOR_L_E1B_PIN     33

#define MOTOR_R_DIR1_PIN    4
#define MOTOR_R_DIR2_PIN    0
#define MOTOR_R_PWM_PIN     2
#define MOTOR_R_DTBY_PIN    MOTOR_L_DTBY_PIN
#define MOTOR_R_E2A_PIN     25
#define MOTOR_R_E2B_PIN     26


enum MotorPosition {
    LEFT_MOTOR,
    RIGHT_MOTOR
};


class HAL_Motor {
    public:
        HAL_Motor(uint8_t pwm_pin, uint8_t dir_pin1, uint8_t dir_pin2, uint8_t stdy_pin,  uint8_t enc_a_pin, uint8_t enc_b_pin);
        HAL_Motor(MotorPosition position, uint32_t period_ms);
        ~HAL_Motor();
        void set_target_rpm(int target_rpm);
        void update_rpm();
        void setup_motor_dir(bool dir_forward);
        void set_pid_gains(int p, int i, int d);
        int get_current_rpm();
        int get_target_rpm();

    private:
        uint8_t pwm_pin;
        uint8_t dir_pin1;
        uint8_t dir_pin2;
        uint8_t stdy_pin;
        uint8_t enc_a_pin;
        uint8_t enc_b_pin;
        uint16_t update_rate_hz = 100;
        float dt = 1e-2f;

        volatile uint32_t last_time = 0;
        int16_t count;
        int last_pwm_out = 0;
        int last_target_rpm = 0;
        int current_rpm = 0;

        PID motor_pid;
        int dir_forward = 1;
        pcnt_unit_t pcnt_uint = PCNT_UNIT_0;  // Default to 0 for left motor, 1 for right motor
        uint8_t pwm_channel = 0;

        void init();
        void getPCNTCount();
        void drive_moter(int pmw);

};

void setupPCNT();