#pragma once

// if enable BT_SERIAL, framework-arduinoespressif32 @ symlink://C:/PIO_Cores/esp32-bluepad32-4.1.0 in the platformio.ini should be commented
#define HAS_BT_SERIAL 0
#define HAS_WIFI_SERIAL 1

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
#define PARM_PID_KI_VELOCITY 0.1f
#define PARM_PID_KD_VELOCITY 0.03f

#define PARM_PID_KP_STEER 1.0f
#define PARM_PID_KI_STEER 0.0f
#define PARM_PID_KD_STEER 0.0f
