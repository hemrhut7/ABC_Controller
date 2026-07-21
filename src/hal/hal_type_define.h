#pragma once

#include <Arduino.h>


typedef enum {
    MODE_STOP = 0,      // 安全停機
    MODE_FREE,          // 1. 自由模式 (無動力/滑行)
    MODE_PWM,           // 2. 直接控制馬達 PWM
    MODE_MOTOR,         // 3. 直接控制馬達轉速 (Open/Close Loop)
    MODE_ANGLE,         // 4. 控制傾角 (最常用的調試模式)
    MODE_VELOCITY,      // 5. 控制前進速度 (加上速度環)
    MODE_REMOTE,        // 6. 綜合遙控 (速度 + 轉向)
} Mode_t;

typedef enum {
    PID_MOTOR = 0,
    PID_RATE,
    PID_ANGLE,
    PID_VELOCITY,
    PID_STEER,
    PID_ID_COUNT
} PID_id_t;

typedef enum {
    LPF_VELOCITY = 0,
    LPF_STEER,
    LPF_GYRO_Z,
    LPF_CURRENT_VELOCITY,
    LPF_ID_COUNT
} LPF_id_t;

typedef enum {
    PARAM_RAMP_PITCH = 0,
    PARAM_RAMP_VELOCITY,
    PARAM_RAMP_ID_COUNT
} PARAM_RAMP_id_t;


typedef struct {
    int mode;
    float target_value; // 根據模式不同，單位可能是 RPM, rad/s, rad, m/s
    float target_steer; // 轉向指令 (rad/s)
} UserCommand_t;


typedef struct {
    uint64_t timestamp;
    float gyro[3];
    float accl[3];
    float temp;
} imu_data_t;

typedef struct {
    uint64_t timestamp;
    float mag[3]; // in uT
} mag_data_t;

typedef struct {
    uint64_t timestamp;
    float pressure;    // hPa
    float temperature; // C
    float altitude;    // m
} baro_data_t;

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
    float steer_rpm;
} PID_target_t;


typedef struct {
    float distance;
    float angle;
    uint8_t intensity;
} lidar_point_t;

#define MAX_LIDAR_POINTS 32
typedef struct {
    uint64_t timestamp;
    lidar_point_t points[MAX_LIDAR_POINTS];
} lidar_scan_t;

typedef struct {
    ABC_state_t abc_state;
    UserCommand_t cmd;
    uint32_t delay_count;
    PID_target_t pid_target;
    float battery_v;
    mag_data_t mag_data;
    baro_data_t baro_data;
} system_state_t;

enum AHRS_STATE {
    IMU_FAILED,
    AHRS_INITIALIZING,
    AHRS_READY,
};
