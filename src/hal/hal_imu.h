#ifndef HAL_IMU_H
#define HAL_IMU_H

#include <Wire.h>
#include "hal_type_define.h"
#include "config.h"


void hal_imu_init();
bool hal_imu_healthy();
bool hal_imu_read(imu_data_t *data);
uint32_t hal_imu_get_error_count();
bool hal_imu_wait_for_data(uint32_t timeout_ms);

// Low-level AK09916 Magnetometer Support
void hal_imu_mag_init();
bool hal_imu_mag_healthy();
bool hal_imu_mag_read(mag_data_t *data);

// Shared I2C mutex control
void hal_imu_i2c_lock();
void hal_imu_i2c_unlock();

#endif // HAL_IMU_H
