#pragma once

typedef struct {
    float gyro[3];
    float accl[3];
    float temp;
} imu_data_t;

struct ahrs_data
{
    float pitch, roll, yaw;
    float gyro_z;
};


struct motor
{
    int rpm_L, rpm_R;
    int target_rpm_L, target_rpm_R;
};
