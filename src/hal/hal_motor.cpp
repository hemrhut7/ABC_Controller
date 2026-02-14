#include "hal_motor.h"

// 死區補償：馬達在低 PWM 時因靜摩擦力無法轉動，需補償基礎電壓
#define MOTOR_DEADZONE 0  // 建議根據實測調整 (通常為 10~30)
// 前饋增益：V = Kv * RPM，減輕 PID 負擔
#define MOTOR_KV       0.0f 

HAL_Motor::HAL_Motor(uint8_t pwm_pin, uint8_t dir_pin1, uint8_t dir_pin2, uint8_t stdy_pin,  uint8_t enc_a_pin, uint8_t enc_b_pin): 
pwm_pin(pwm_pin), dir_pin1(dir_pin1), dir_pin2(dir_pin2), stdy_pin(stdy_pin), enc_a_pin(enc_a_pin), enc_b_pin(enc_b_pin) {
    init();
}

HAL_Motor::HAL_Motor(MotorPosition position, uint32_t period_ms) {
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
    update_rate_hz = 1000 / period_ms;
    dt = (float)period_ms * 1e-3f;
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
    pid.reset();
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
    // dps = count * degrees_per_count * frequency
    float dps = (float)count * DEG_PER_CNT * update_rate_hz;
    current_rpm = dps * DPS_2_RPM * dir_forward;
}

void HAL_Motor::set_target_rpm(int target_rpm) {
    last_target_rpm = target_rpm;
    
    // 1. PID 計算
    float output = pid.compute(dt, target_rpm, current_rpm);

    // 2. 前饋控制 (Feedforward)
    output += target_rpm * MOTOR_KV;

    // 3. 死區補償 (Deadzone Compensation)
    if (target_rpm != 0) {
        if (output > 0) output += MOTOR_DEADZONE;
        else if (output < 0) output -= MOTOR_DEADZONE;
    }

    // 4. 輸出限制
    last_pwm_out = constrain(output, -255, 255);
    drive_moter(last_pwm_out * dir_forward);
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

void HAL_Motor::set_pid_gains(float p, float i, float d) {
    pid.setTunings(p, i, d);
}

void HAL_Motor::set_enable(bool enable) {
    if (is_enable == enable) return;
    is_enable = enable;
    if (enable) {
        digitalWrite(stdy_pin, HIGH);
        pid.reset();
    } else {
        digitalWrite(stdy_pin, LOW); // 關閉 H-Bridge (高阻抗/滑行)
        ledcWrite(pwm_channel, 0);   // 確保 PWM 為 0
    }
}

int HAL_Motor::get_current_rpm() {
    return current_rpm;
}

int HAL_Motor::get_target_rpm() {
    return last_target_rpm;
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
