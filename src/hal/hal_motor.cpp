#include "hal_motor.h"

HAL_Motor::HAL_Motor(uint8_t pwm_pin, uint8_t dir_pin1, uint8_t dir_pin2, uint8_t stdy_pin,  uint8_t enc_a_pin, uint8_t enc_b_pin): 
pwm_pin(pwm_pin), dir_pin1(dir_pin1), dir_pin2(dir_pin2), stdy_pin(stdy_pin), enc_a_pin(enc_a_pin), enc_b_pin(enc_b_pin) {
    init();
}

HAL_Motor::HAL_Motor(MotorPosition position) {
    if (position == LEFT_MOTOR) {
        pwm_pin = MOTOR_L_PWM_PIN;
        dir_pin1 = MOTOR_L_DIR1_PIN;
        dir_pin2 = MOTOR_L_DIR2_PIN;
        stdy_pin = MOTOR_L_DTBY_PIN;
        enc_a_pin = MOTOR_L_E1A_PIN;
        enc_b_pin = MOTOR_L_E1B_PIN;
        pcnt_uint = PCNT_UNIT_0;
        dir_forward = 1;
        pwm_channel = 0;
    } else if (position == RIGHT_MOTOR) {
        pwm_pin = MOTOR_R_PWM_PIN;
        dir_pin1 = MOTOR_R_DIR1_PIN;
        dir_pin2 = MOTOR_R_DIR2_PIN;
        stdy_pin = MOTOR_R_DTBY_PIN;
        enc_a_pin = MOTOR_R_E2A_PIN;
        enc_b_pin = MOTOR_R_E2B_PIN;
        pcnt_uint = PCNT_UNIT_1;
        dir_forward = -1;
        pwm_channel = 1;
    }
    init();
}

void HAL_Motor::init() {    
    pinMode(pwm_pin, OUTPUT);
    pinMode(dir_pin1, OUTPUT);
    pinMode(dir_pin2, OUTPUT);
    pinMode(enc_a_pin, INPUT);
    pinMode(enc_b_pin, INPUT);

    ledcSetup(pwm_channel, PWM_FREQ, PWM_RES);
    ledcAttachPin(pwm_pin, pwm_channel);
    ledcWrite(pwm_channel, 0);
    digitalWrite(dir_pin1, HIGH);
    digitalWrite(dir_pin2, LOW);
    digitalWrite(stdy_pin, HIGH);

    count = 0;
    last_time = millis();
}

HAL_Motor::~HAL_Motor() {
    ledcWrite(pwm_channel, 0);
    digitalWrite(dir_pin1, HIGH);
    digitalWrite(dir_pin2, LOW);
    digitalWrite(stdy_pin, LOW);
}

void HAL_Motor::getPCNTCount() {
    pcnt_get_counter_value(pcnt_uint, &count);
    pcnt_counter_clear(pcnt_uint); // 讀取後清零
}

void HAL_Motor::update_rpm() {
    getPCNTCount();
    float dps = (float)count * DEG_PER_CNT * 200.0f;
    current_rpm = dps * DPS_2_RPM;
}

void HAL_Motor::set_target_rpm(int target_rpm) {
    last_target_rpm = target_rpm;  // maybe can be applied with LPF later
    int error = target_rpm - current_rpm;
    integral += error;
    integral = constrain(integral, -100, 100);  // Anti-windup
    int derivative = error - last_error;
    last_error = error;

    int pwm = p_gain * error + i_gain * integral + d_gain * derivative;
    pwm = constrain(pwm, -255, 255) * dir_forward;
    drive_moter(pwm);
}

void HAL_Motor::drive_moter(int pmw) {
    if(pmw >= 0) {
        digitalWrite(dir_pin1, HIGH);
        digitalWrite(dir_pin2, LOW);
        ledcWrite(pwm_channel, pmw);
    } else {
        digitalWrite(dir_pin1, LOW);
        digitalWrite(dir_pin2, HIGH);
        ledcWrite(pwm_channel, -pmw);
    }
}

void HAL_Motor::setup_motor_dir(bool dir_forward) {
    this->dir_forward = dir_forward ? 1 : -1;
}

void HAL_Motor::set_pid_gains(int p, int i, int d) {
    p_gain = p;
    i_gain = i;
    d_gain = d;
}

int HAL_Motor::get_current_rpm() {
    return current_rpm;
}

void setupPCNT() {
    // 配置左電機 (Unit 0)
    pcnt_config_t pcnt_config_l = {
        .pulse_gpio_num = MOTOR_L_E1A_PIN, // A 相
        .ctrl_gpio_num = MOTOR_L_E1B_PIN,  // B 相
        .lctrl_mode = PCNT_MODE_REVERSE,   // B 相低電平反轉計數（減）
        .hctrl_mode = PCNT_MODE_KEEP,      // B 相高電平保持計數（增）
        .pos_mode = PCNT_COUNT_INC,        // A 相上升沿增計數
        .neg_mode = PCNT_COUNT_DIS,        // A 相下降沿禁用
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = PCNT_UNIT_0,
        .channel = PCNT_CHANNEL_0
    };
    pcnt_unit_config(&pcnt_config_l);
    pcnt_set_filter_value(PCNT_UNIT_0, 10); // 濾波 10 個 APB 週期 (~125ns @ 80MHz)
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);

    // 配置右電機 (Unit 1)
    pcnt_config_t pcnt_config_r = {
        .pulse_gpio_num = MOTOR_R_E2A_PIN,
        .ctrl_gpio_num = MOTOR_R_E2B_PIN,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = PCNT_UNIT_1,
        .channel = PCNT_CHANNEL_0
    };
    pcnt_unit_config(&pcnt_config_r);
    pcnt_set_filter_value(PCNT_UNIT_1, 10);
    pcnt_filter_enable(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
}
