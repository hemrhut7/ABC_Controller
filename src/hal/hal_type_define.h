#pragma once

#include <Arduino.h>


typedef enum {
    MODE_STOP = 0,      // 安全停機
    MODE_PWM,           // 0. 直接控制馬達 PWM
    MODE_MOTOR,         // 1. 直接控制馬達轉速 (Open/Close Loop)
    MODE_RATE,          // 2. 控制角速度 (極難平衡，僅供調試 D term)
    MODE_ANGLE,         // 3. 控制傾角 (最常用的調試模式)
    MODE_VELOCITY,      // 4. 控制前進速度 (加上速度環)
    MODE_REMOTE,        // 5. 綜合遙控 (速度 + 轉向)
    MODE_FREE,          // 6. 自由模式 (無動力/滑行)
} Mode_t;

typedef enum {
    PID_MOTOR = 0,
    PID_RATE,
    PID_ANGLE,
    PID_VELOCITY,
    PID_YAW,
    PID_ID_COUNT
} PID_id_t;

typedef struct {
    int mode;
    float target_value; // 根據模式不同，單位可能是 RPM, rad/s, rad, m/s
    float target_yaw_rate; // 轉向指令 (rad/s)
} UserCommand_t;


typedef struct {
    uint64_t timestamp;
    float gyro[3];
    float accl[3];
    float temp;
} imu_data_t;

typedef struct {   
    imu_data_t imu_data;
    imu_data_t imu_data_calibrated;
    float euler[3];
} ahrs_data_t;


typedef struct
{
    float rpm_L, rpm_R;
    float pwm_out_L, pwm_out_R;
} motor_state_t;

typedef struct
{
    ahrs_data_t ahrs_data;
    motor_state_t motor_state;
    float velocity;
} ABC_state_t;

typedef struct {
    float rpm_L, rpm_R;
    float pitch;
    float velocity;
    float yaw_rate;
} PID_target_t;

typedef struct {
    ABC_state_t abc_state;
    uint32_t loop_time_ms;
    float target_val;
    PID_target_t pid_target;
    Mode_t mode;
} system_state_t;

