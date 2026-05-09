#include "hal_imu.h"
#include <Wire.h>

namespace {
    // QMI8658 Registers
    constexpr uint8_t kImuAddress = 0x6B;
    constexpr uint8_t kQmiWhoAmIReg = 0x00;
    constexpr uint8_t kQmiWhoAmIValue = 0x05;
    constexpr uint8_t kQmiCtrl1Reg = 0x02;
    constexpr uint8_t kQmiCtrl2Reg = 0x03;
    constexpr uint8_t kQmiCtrl3Reg = 0x04;
    constexpr uint8_t kQmiCtrl5Reg = 0x06;
    constexpr uint8_t kQmiCtrl7Reg = 0x08;
    constexpr uint8_t kQmiTempLowReg = 0x33;
    constexpr uint8_t kQmiAccelLowReg = 0x35;
    constexpr uint8_t kQmiStatus0Reg = 0x2E;
    constexpr uint8_t kQmiResetReg = 0x60;

    // Config values
    constexpr uint8_t kCtrl2Accel8gOdr2000Hz = 0x22;
    constexpr uint8_t kCtrl3Gyro512dpsOdr2000Hz = 0x52;
    constexpr uint8_t kCtrl5Lpf106Hz = 0x55;
    
    constexpr float kAccelScale = (8.0f / 32768.0f) * 9.80665f; // to m/s^2
    constexpr float kGyroScale = (512.0f / 32768.0f) * DEG_TO_RAD; // to rad/s

    void writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(kImuAddress);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }

    uint8_t readReg(uint8_t reg) {
        Wire.beginTransmission(kImuAddress);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(kImuAddress, (uint8_t)1);
        return Wire.read();
    }
}
static bool is_initialized = false;

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
    if (!Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, 400000)) {
        Serial.println("Failed to initialize I2C bus");
        return;
    }
    
    // Check WHO_AM_I
    uint8_t whoAmI = readReg(kQmiWhoAmIReg);
    if (whoAmI != kQmiWhoAmIValue) {
        Serial.printf("QMI8658 not found! WHO_AM_I=0x%02X\n", whoAmI);
        return;
    }

    // Reset
    writeReg(kQmiResetReg, 0xB0);
    delay(20);

    // Config
    writeReg(kQmiCtrl1Reg, 0x40); // Address auto-increment
    writeReg(kQmiCtrl2Reg, kCtrl2Accel8gOdr2000Hz);
    writeReg(kQmiCtrl3Reg, kCtrl3Gyro512dpsOdr2000Hz);
    writeReg(kQmiCtrl5Reg, kCtrl5Lpf106Hz);
    writeReg(kQmiCtrl7Reg, 0x03); // Enable accel and gyro

    is_initialized = true;
    Serial.println("QMI8658 initialized");
}

bool hal_imu_healthy() {
    return is_initialized;
}

void hal_imu_read(imu_data_t *data) {
    if (!is_initialized) return;

    // Read status
    if (!(readReg(kQmiStatus0Reg) & 0x03)) return; // No new data, remove status check if it makes system blocked

    // Read 14 bytes: temp(2) + accel(6) + gyro(6)
    Wire.beginTransmission(kImuAddress);
    Wire.write(kQmiTempLowReg);
    Wire.endTransmission(false);
    Wire.requestFrom(kImuAddress, (uint8_t)14);

    uint8_t buf[14];
    for (int i = 0; i < 14; i++) buf[i] = Wire.read();

    data->timestamp = micros();

    int16_t t_raw = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t ax_raw = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t ay_raw = (int16_t)(buf[5] << 8 | buf[4]);
    int16_t az_raw = (int16_t)(buf[7] << 8 | buf[6]);
    int16_t gx_raw = (int16_t)(buf[9] << 8 | buf[8]);
    int16_t gy_raw = (int16_t)(buf[11] << 8 | buf[10]);
    int16_t gz_raw = (int16_t)(buf[13] << 8 | buf[12]);

    data->temp = (float)t_raw / 256.0f;

    // Map raw data to m/s^2 and rad/s
    // Note: We keep the sign flip logic from MPU6050 if axes align similarly
    // Previous logic: ax=-ax, ay=ay, az=-az
    float raw_x = -(float)ax_raw * kAccelScale;
    float raw_y = (float)ay_raw * kAccelScale;
    float raw_z = -(float)az_raw * kAccelScale;

    data->accl[0] = ACC_CAL_R00 * raw_x + ACC_CAL_R01 * raw_y + ACC_CAL_R02 * raw_z + ACC_CAL_B0;
    data->accl[1] = ACC_CAL_R10 * raw_x + ACC_CAL_R11 * raw_y + ACC_CAL_R12 * raw_z + ACC_CAL_B1;
    data->accl[2] = ACC_CAL_R20 * raw_x + ACC_CAL_R21 * raw_y + ACC_CAL_R22 * raw_z + ACC_CAL_B2;

    float raw_gx = -(float)gx_raw * kGyroScale;
    float raw_gy = (float)gy_raw * kGyroScale;
    float raw_gz = -(float)gz_raw * kGyroScale;

    data->gyro[0] = GYRO_CAL_R00 * (raw_gx - GYRO_CAL_B0) + GYRO_CAL_R01 * (raw_gy - GYRO_CAL_B1) + GYRO_CAL_R02 * (raw_gz - GYRO_CAL_B2);
    data->gyro[1] = GYRO_CAL_R10 * (raw_gx - GYRO_CAL_B0) + GYRO_CAL_R11 * (raw_gy - GYRO_CAL_B1) + GYRO_CAL_R12 * (raw_gz - GYRO_CAL_B2);
    data->gyro[2] = GYRO_CAL_R20 * (raw_gx - GYRO_CAL_B0) + GYRO_CAL_R21 * (raw_gy - GYRO_CAL_B1) + GYRO_CAL_R22 * (raw_gz - GYRO_CAL_B2);
}
