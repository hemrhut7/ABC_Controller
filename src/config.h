#pragma once

// --- ESP32-S3-LCD-2 Pin Mapping ---
// Based on SchDoc: P1 (Left Row) / P2 (Right Row) physical header layout
//
// ┌───────────────────────────────────────────────────┐
// │  P1 (Left Row)              P2 (Right Row)        │
// │  Pin 1:  IO2  ← L_ENC_B     3V3                   │
// │  Pin 2:  IO4  ← L_ENC_A     GND                   │
// │  Pin 3:  IO6  ← ADC_BAT     IO43 ← UART1_TX       │
// │  Pin 4:  IO16 ← R_PWM       IO44 ← UART1_RX       │
// │  Pin 5:  IO17 ← R_DIR2      IO47 ← IMU_SCL        │
// │  Pin 6:  IO18 ← R_DIR1      IO48 ← IMU_SDA        │
// │  Pin 7:  IO21 ← STBY        IO15 ← R_ENC_B        │
// │  Pin 8:  IO8  ← L_DIR1      IO13 ← R_ENC_A        │
// │  Pin 9:  IO7  ← L_DIR2      IO11 ← UART2_TX       │
// │  Pin 10: IO10 ← L_PWM       IO12 ← UART2_RX       │
// │  Pin 11: IO20 (USB_P!)      IO14 ← (free)         │
// │  Pin 12: IO19 (USB_N!)      IO9  ← (free)         │
// │  Pin 13: GND                GND                   │
// │  Pin 14: 5V                 VBAT                  │
// └───────────────────────────────────────────────────┘
// ⚠ IO19/IO20 are USB D-/D+ — DO NOT USE with USB CDC enabled

// Onboard Hardware (not on PinOut headers)
#define LCD_BL_PIN      1
#define LCD_MOSI_PIN    38
#define LCD_SCLK_PIN    39
#define LCD_MISO_PIN    40
#define LCD_DC_PIN      42
#define LCD_CS_PIN      45
#define IMU_SCL_PIN     47
#define IMU_SDA_PIN     48
#define IMU_INT1_PIN    14
#define BAT_ADC_PIN     6

// Motor Control
#define MOTOR_L_PWM_PIN     10     // P1 Pin 10
#define MOTOR_L_DIR1_PIN    8      // P1 Pin 8
#define MOTOR_L_DIR2_PIN    7      // P1 Pin 9
#define MOTOR_L_E1A_PIN     4      // P1 Pin 2
#define MOTOR_L_E1B_PIN     2      // P1 Pin 1
#define MOTOR_STBY_PIN      21     // P1 Pin 7

#define MOTOR_R_PWM_PIN     16     // P1 Pin 4
#define MOTOR_R_DIR1_PIN    18     // P1 Pin 6
#define MOTOR_R_DIR2_PIN    17     // P1 Pin 5
#define MOTOR_R_E2A_PIN     13     // P2 Pin 8
#define MOTOR_R_E2B_PIN     15     // P2 Pin 7

// UART Interfaces
#define UART1_TX_PIN       43      // P2 Pin 3
#define UART1_RX_PIN       44      // P2 Pin 4
#define UART2_TX_PIN       12      // P2 Pin 9
#define UART2_RX_PIN       11      // P2 Pin 10
// ---------------------------------

#define WIFI_SSID "TP-Link_E428"
#define WIFI_PASS "ssssssss"
#define AP_SSID "ABC_Controller_AP"
#define AP_PASS "ssssssss"
#define UDP_PORT 8000


// 物理參數定義
// 假設輪徑 65mm => 半徑 0.0325m
// 速度 (m/s) = (RPM / 60) * 2 * PI * R
// Factor = 0.0325 * 2 * 3.14159 / 60 ~= 0.003403
#define MAX_RPM      300
#define RPM_TO_MS 0.003403f 
#define MAX_PITCH 6.0f * DEG_TO_RAD
#define MAX_PITCH_RAMP 15.0f * DEG_TO_RAD
#define MAX_VELOCITY_RAMP 100.0f
#define MAX_VELOCITY MAX_RPM * 0.5f * 0.7f * RPM_TO_MS
#define MAX_STEER_RPM MAX_RPM * 0.5f * 0.65f

#define PERIOD_CONTROLL 3.0f // 333.3Hz (3ms)

#define PARM_LPF_CUTOFF_FREQ_VELOCITY 5.0f
#define PARM_LPF_CUTOFF_FREQ_STEER 5.0f
#define PARM_LPF_CUTOFF_FREQ_GYRO_Z 10.0f
#define PARM_LPF_CUTOFF_FREQ_CURRENT_VELOCITY 1.0f

#define PARM_PID_KP_MOTOR 12.0f
#define PARM_PID_KI_MOTOR 220.0f
#define PARM_PID_KD_MOTOR 0.05f

#define PARM_PID_KP_ANGLE 1000.0f
#define PARM_PID_KI_ANGLE 28000.0f
#define PARM_PID_KD_ANGLE 15.0f

#define PARM_PID_KP_VELOCITY 0.3f
#define PARM_PID_KI_VELOCITY 0.2f
#define PARM_PID_KD_VELOCITY 0.03f

#define PARM_PID_KP_STEER 1.0f
#define PARM_PID_KI_STEER 0.0f
#define PARM_PID_KD_STEER 0.0f
