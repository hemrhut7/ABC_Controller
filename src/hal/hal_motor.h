#pragma once

#include <Arduino.h>
#include <vector>
#include <driver/pcnt.h>

enum MotorPosition {
    LEFT_MOTOR,
    RIGHT_MOTOR
};

class HAL_Motor {
    public:
        HAL_Motor(uint8_t pwm_pin, uint8_t dir_pin1, uint8_t dir_pin2, uint8_t stdy_pin,  uint8_t enc_a_pin, uint8_t enc_b_pin);
        HAL_Motor(MotorPosition position);
        ~HAL_Motor();
        void update(int target_rpm);
        int get_rpm();
        void setup_motor_dir(bool dir_forward);
        void set_pid_gains(int p, int i, int d);

    private:
        uint8_t pwm_pin;
        uint8_t dir_pin1;
        uint8_t dir_pin2;
        uint8_t stdy_pin;
        uint8_t enc_a_pin;
        uint8_t enc_b_pin;

        volatile uint32_t last_time = 0;
        int16_t count;
        int last_target_rpm = 0;
        int current_rpm = 0;

        int p_gain = 2;
        int i_gain = 1;
        int d_gain = 0;
        int last_error = 0;
        int integral = 0;
        int dir_forward = 1;
        pcnt_unit_t pcnt_uint = PCNT_UNIT_0;  // Default to 0 for left motor, 1 for right motor

        void init();
        void encoder_isr();
        void getPCNTCount();
        

        static std::vector<HAL_Motor*> instances;
        static void isr_motor0();
        static void isr_motor1();

};

void setupPCNT();