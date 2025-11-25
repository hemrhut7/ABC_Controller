#include "controller.h"




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


void initMotorDriver(){
  pinMode(MOTOR_L_PWM_PIN, OUTPUT);
  pinMode(MOTOR_L_DIR1_PIN, OUTPUT);
  pinMode(MOTOR_L_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_L_DTBY_PIN, OUTPUT);

  pinMode(MOTOR_L_E1A_PIN, INPUT);
  pinMode(MOTOR_L_E1B_PIN, INPUT);

  digitalWrite(MOTOR_L_DIR1_PIN, HIGH);
  digitalWrite(MOTOR_L_DIR2_PIN, LOW);
  // digitalWrite(STBY, HIGH);
  analogWrite(MOTOR_L_PWM_PIN, 0);

  pinMode(MOTOR_R_PWM_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR1_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR2_PIN, OUTPUT);
  pinMode(MOTOR_R_DTBY_PIN, OUTPUT);

  pinMode(MOTOR_R_E2A_PIN, INPUT);
  pinMode(MOTOR_R_E2B_PIN, INPUT);

  digitalWrite(MOTOR_R_DIR1_PIN, HIGH);
  digitalWrite(MOTOR_R_DIR2_PIN, LOW);
  // digitalWrite(STBY, HIGH);
  analogWrite(MOTOR_R_PWM_PIN, 0);
}



