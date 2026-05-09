#ifndef HAL_IMU_H
#define HAL_IMU_H

#include <Wire.h>
#include "hal_type_define.h"
#include "config.h"


void hal_imu_init();
bool hal_imu_healthy();
void hal_imu_read(imu_data_t *data);

#endif // HAL_IMU_H
