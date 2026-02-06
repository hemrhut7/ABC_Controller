#pragma once

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
