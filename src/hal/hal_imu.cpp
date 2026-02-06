#include "hal_imu.h"
#include <Wire.h>

Adafruit_MPU6050 mpu;

void hal_imu_init() {
  // Try to initialize!
  if (!mpu.begin()) {
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  mpu.setSampleRateDivisor(4); // 1kHz / (1 + 4) = 200Hz
}

void hal_imu_read(imu_data_t *data) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

    data->accl[0] = a.acceleration.x;
    data->accl[1] = a.acceleration.y;
    data->accl[2] = a.acceleration.z;

    data->gyro[0] = g.gyro.x;
    data->gyro[1] = g.gyro.y;
    data->gyro[2] = g.gyro.z;

  data->temp = temp.temperature;
}
