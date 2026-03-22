#include "hal_motor.h"


// HAL_Motor::HAL_Motor(uint8_t pwm_pin, uint8_t dir_pin1, uint8_t dir_pin2, uint8_t stdy_pin,  uint8_t enc_a_pin, uint8_t enc_b_pin): 
// pwm_pin(pwm_pin), dir_pin1(dir_pin1), dir_pin2(dir_pin2), stdy_pin(stdy_pin), enc_a_pin(enc_a_pin), enc_b_pin(enc_b_pin) {
//     init();
// }

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
    MAX_MOTOR_DELTA_RPM = MAX_MOTOR_RPM_RATE * dt;
}

void HAL_Motor::init() {    
    pinMode(pwm_pin, OUTPUT);
    pinMode(dir_pin1, OUTPUT);
    pinMode(dir_pin2, OUTPUT);
    pinMode(stdy_pin, OUTPUT);
    pinMode(enc_a_pin, INPUT);
    pinMode(enc_b_pin, INPUT);

    ledcSetup(pwm_channel, PWM_FREQ, PWM_RES);
    ledcAttachPin(pwm_pin, pwm_channel);
    ledcWrite(pwm_channel, 0);
    digitalWrite(dir_pin1, LOW);
    digitalWrite(dir_pin2, LOW);
    digitalWrite(stdy_pin, LOW);

    count = 0;
    resetControllerState();
    pid.setOutputLimits(MAX_PWM_DUTY);
}

HAL_Motor::~HAL_Motor() {
    ledcWrite(pwm_channel, 0);
    digitalWrite(dir_pin1, LOW);
    digitalWrite(dir_pin2, LOW);
    digitalWrite(stdy_pin, LOW);
}

void HAL_Motor::getPCNTCount() {
    pcnt_get_counter_value(pcnt_uint, &count);
    pcnt_counter_clear(pcnt_uint); // 讀取後清零
}

void HAL_Motor::update_rpm(float dt) {
    this->dt = dt;
    this->update_rate_hz = 1.0f / dt;
    this->MAX_MOTOR_DELTA_RPM = MAX_MOTOR_RPM_RATE * dt;

    getPCNTCount();
    float dps = (float)count * DEG_PER_CNT * update_rate_hz;
    current_rpm = dps * DPS_2_RPM * dir_forward;
}

void HAL_Motor::set_target_rpm(float target_rpm) {
    target_rpm = constrain(target_rpm, -MAX_MOTOR_RPM, MAX_MOTOR_RPM);
    if (last_target_rpm - target_rpm > MAX_MOTOR_DELTA_RPM) {
        target_rpm = last_target_rpm - MAX_MOTOR_DELTA_RPM;
    } else if (target_rpm - last_target_rpm > MAX_MOTOR_DELTA_RPM) {
        target_rpm = last_target_rpm + MAX_MOTOR_DELTA_RPM;
    }

    float derivative = (dt > 0) ? -(current_rpm - last_rpm) * update_rate_hz : 0;
    float output = pid.compute(dt, target_rpm, current_rpm, derivative); // 加入 P、I、D 計算
    output += target_rpm * MOTOR_KV;

    drive_moter(output);

    last_target_rpm = target_rpm;
    last_rpm = current_rpm;
}

void HAL_Motor::resetControllerState() {
    pid.reset();
    last_target_rpm = 0.0f;
    last_rpm = current_rpm;
    last_pwm_out = 0;
    dither_dir = 1;
}

void HAL_Motor::drive_moter(int pwm) {
    if (pwm == 0) {
        ledcWrite(pwm_channel, 0);
        digitalWrite(dir_pin1, LOW);
        digitalWrite(dir_pin2, LOW);
        return;
    }

    if (pwm < MOTOR_DEADZONE && pwm > -MOTOR_DEADZONE && current_rpm == 0) {
        pwm += dither_dir * MOTOR_DITHER;
        dither_dir *= -1;
    }
    
    last_pwm_out = constrain(pwm, -MAX_PWM_DUTY, MAX_PWM_DUTY);
    if(last_pwm_out >= 0) {
        digitalWrite(dir_pin1, HIGH);
        digitalWrite(dir_pin2, LOW);
        ledcWrite(pwm_channel, last_pwm_out);
    } else {
        digitalWrite(dir_pin1, LOW);
        digitalWrite(dir_pin2, HIGH);
        ledcWrite(pwm_channel, -last_pwm_out);
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
        resetControllerState();
    } else {
        resetControllerState();
        digitalWrite(stdy_pin, LOW); // 關閉 H-Bridge (高阻抗/滑行)
        ledcWrite(pwm_channel, 0);   // 確保 PWM 為 0
    }
}

void setupPCNT() {
    // 配置左電機 (Unit 0) - Channel 0
    pcnt_config_t pcnt_config_l_ch0 = {
        .pulse_gpio_num = MOTOR_L_E1A_PIN, // A 相
        .ctrl_gpio_num = MOTOR_L_E1B_PIN,  // B 相
        .lctrl_mode = PCNT_MODE_REVERSE,   // B 相低電平反轉計數（減）
        .hctrl_mode = PCNT_MODE_KEEP,      // B 相高電平保持計數（增）
        .pos_mode = PCNT_COUNT_INC,        // A 相上升沿增計數
        .neg_mode = PCNT_COUNT_DEC,        // A 相下降沿減計數
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = PCNT_UNIT_0,
        .channel = PCNT_CHANNEL_0
    };
    pcnt_unit_config(&pcnt_config_l_ch0);

    // 配置左電機 (Unit 0) - Channel 1 (新增以實現 4 倍頻)
    pcnt_config_t pcnt_config_l_ch1 = {
        .pulse_gpio_num = MOTOR_L_E1B_PIN, // B 相
        .ctrl_gpio_num = MOTOR_L_E1A_PIN,  // A 相
        .lctrl_mode = PCNT_MODE_KEEP,      // A 相低電平保持 (配合 Ch0 邏輯)
        .hctrl_mode = PCNT_MODE_REVERSE,   // A 相高電平反轉
        .pos_mode = PCNT_COUNT_INC,        // B 相上升沿
        .neg_mode = PCNT_COUNT_DEC,        // B 相下降沿
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = PCNT_UNIT_0,
        .channel = PCNT_CHANNEL_1
    };
    pcnt_unit_config(&pcnt_config_l_ch1);

    pcnt_set_filter_value(PCNT_UNIT_0, 100); // 濾波 10 個 APB 週期 (~125ns @ 80MHz)
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);

    // 配置右電機 (Unit 1) - Channel 0
    pcnt_config_t pcnt_config_r_ch0 = {
        .pulse_gpio_num = MOTOR_R_E2A_PIN,
        .ctrl_gpio_num = MOTOR_R_E2B_PIN,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = PCNT_UNIT_1,
        .channel = PCNT_CHANNEL_0
    };
    pcnt_unit_config(&pcnt_config_r_ch0);

    // 配置右電機 (Unit 1) - Channel 1
    pcnt_config_t pcnt_config_r_ch1 = {
        .pulse_gpio_num = MOTOR_R_E2B_PIN,
        .ctrl_gpio_num = MOTOR_R_E2A_PIN,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_REVERSE,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = PCNT_UNIT_1,
        .channel = PCNT_CHANNEL_1
    };
    pcnt_unit_config(&pcnt_config_r_ch1);

    pcnt_set_filter_value(PCNT_UNIT_1, 100);
    pcnt_filter_enable(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
}
