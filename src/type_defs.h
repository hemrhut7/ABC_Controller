#pragma once

#define DEBUG_PRINT 1
#define ENABLE_BT 0

#define UART_CTL Serial2
#define UART_USB Serial

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

struct AHRS_Data {
  float time_sec;
  float gyro_dps[3];
  float acc_mps2[3];
  float euler_deg[3];
};