#include "hal_imu.h"
#include <Wire.h>

Adafruit_MPU6050 mpu;
static bool is_initialized = false;


void hal_imu_init() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    return;
  }

  Wire.setClock(400000); // 提升至 400kHz Fast Mode
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  mpu.setSampleRateDivisor(0);
  is_initialized = true;
}

bool healthy() {
  return is_initialized;
}

void hal_imu_read(imu_data_t *data) {
  mpu.fastRead(&data->timestamp, &data->gyro[0], &data->accl[0], &data->temp);
}
