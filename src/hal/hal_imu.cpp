#include "hal_imu.h"
#include <Wire.h>

Adafruit_MPU6050 mpu;
static bool is_initialized = false;

#define SDA_PIN_MPU6050 23
#define SCL_PIN_MPU6050 22

// Accelerometer Calibration Parameters
// Matrix R
constexpr float ACC_CAL_R00 = 1.00219466f;
constexpr float ACC_CAL_R01 = -0.01182011f;
constexpr float ACC_CAL_R02 = -0.05202197f;
constexpr float ACC_CAL_R10 = -0.00749989f;
constexpr float ACC_CAL_R11 = 0.99577523f;
constexpr float ACC_CAL_R12 = 0.00332870f;
constexpr float ACC_CAL_R20 = 0.04590206f;
constexpr float ACC_CAL_R21 = -0.00101295f;
constexpr float ACC_CAL_R22 = 0.97995910f;
// Bias Vector
constexpr float ACC_CAL_B0  = 0.16521701f;
constexpr float ACC_CAL_B1  = 0.04923417f;
constexpr float ACC_CAL_B2  = 0.14537917f;

// Gyroscope Calibration Parameters
// Matrix R
constexpr float GYRO_CAL_R00 = 0.99891562f;
constexpr float GYRO_CAL_R01 = -0.00027220f;
constexpr float GYRO_CAL_R02 = -0.04655650f;
constexpr float GYRO_CAL_R10 = 0.00037159f;
constexpr float GYRO_CAL_R11 = 0.99999767f;
constexpr float GYRO_CAL_R12 = 0.00212622f;
constexpr float GYRO_CAL_R20 = 0.04655582f;
constexpr float GYRO_CAL_R21 = -0.00214122f;
constexpr float GYRO_CAL_R22 = 0.99891340f;
// Bias Vector
constexpr float GYRO_CAL_B0  = 0.02105;
constexpr float GYRO_CAL_B1  = 0.01105;
constexpr float GYRO_CAL_B2  = -0.01655f;


void hal_imu_init() {
  if (!Wire.begin(SDA_PIN_MPU6050, SCL_PIN_MPU6050, 400000)) {
    Serial.println("Failed to initialize I2C bus");
    return;
  }
  Wire.setTimeOut(5);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    return;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  mpu.setSampleRateDivisor(0);
  is_initialized = true;
  Serial.println("MPU6050 initialized");
}

bool hal_imu_healthy() {
  return is_initialized;
}

void hal_imu_read(imu_data_t *data) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  data->timestamp = micros();

  float raw_x = -a.acceleration.x;
  float raw_y = a.acceleration.y;
  float raw_z = -a.acceleration.z;

  data->accl[0] = ACC_CAL_R00 * raw_x + ACC_CAL_R01 * raw_y + ACC_CAL_R02 * raw_z + ACC_CAL_B0;
  data->accl[1] = ACC_CAL_R10 * raw_x + ACC_CAL_R11 * raw_y + ACC_CAL_R12 * raw_z + ACC_CAL_B1;
  data->accl[2] = ACC_CAL_R20 * raw_x + ACC_CAL_R21 * raw_y + ACC_CAL_R22 * raw_z + ACC_CAL_B2;

  float raw_gx = -g.gyro.x;
  float raw_gy = g.gyro.y;
  float raw_gz = -g.gyro.z;
  // with different way to calibration.
  // due to we are not able to test on the rotation table, so we implement the rotation matrix of ACCL with out bias
  data->gyro[0] = GYRO_CAL_R00 * (raw_gx - GYRO_CAL_B0) + GYRO_CAL_R01 * (raw_gy - GYRO_CAL_B1) + GYRO_CAL_R02 * (raw_gz - GYRO_CAL_B2);
  data->gyro[1] = GYRO_CAL_R10 * (raw_gx - GYRO_CAL_B0) + GYRO_CAL_R11 * (raw_gy - GYRO_CAL_B1) + GYRO_CAL_R12 * (raw_gz - GYRO_CAL_B2);
  data->gyro[2] = GYRO_CAL_R20 * (raw_gx - GYRO_CAL_B0) + GYRO_CAL_R21 * (raw_gy - GYRO_CAL_B1) + GYRO_CAL_R22 * (raw_gz - GYRO_CAL_B2);

  

  data->temp = temp.temperature;
}
