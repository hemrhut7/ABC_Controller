#pragma once

typedef struct {
    float gyro[3];
    float accl[3];
    float temp;
} imu_data_t;

typedef struct {   
    imu_data_t imu_data;
    float euler[3];
} ahrs_data_t;


typedef struct
{
    int rpm_L, rpm_R;
    int target_rpm_L, target_rpm_R;
}motor_state_t;
