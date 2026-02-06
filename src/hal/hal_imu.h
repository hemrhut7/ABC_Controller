#ifndef HAL_IMU_H
#define HAL_IMU_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "hal_type_define.h"


void hal_imu_init();
void hal_imu_read(imu_data_t *data);

#endif // HAL_IMU_H
