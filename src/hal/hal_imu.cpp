#include "hal_imu.h"
#include <Wire.h>

Adafruit_MPU6050 mpu;
static bool is_initialized = false;

#define SDA_PIN_MPU6050 23
#define SCL_PIN_MPU6050 22



void hal_imu_init() {
  if (!Wire.begin(SDA_PIN_MPU6050, SCL_PIN_MPU6050, 400000)) {
    Serial.println("Failed to initialize I2C bus");
    return;
  }
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    return;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  mpu.setSampleRateDivisor(0);
  is_initialized = true;
}

bool hal_imu_healthy() {
  return is_initialized;
}

void hal_imu_read(imu_data_t *data) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  data->timestamp = (float)millis() * 1e-3f;
  data->accl[0] = a.acceleration.x;
  data->accl[1] = a.acceleration.y;
  data->accl[2] = a.acceleration.z;

  data->gyro[0] = g.gyro.x;
  data->gyro[1] = g.gyro.y;
  data->gyro[2] = g.gyro.z;

  data->temp = temp.temperature;
}
