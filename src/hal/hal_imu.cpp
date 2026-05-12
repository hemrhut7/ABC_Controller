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

    static bool is_initialized = false;

    bool writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(kImuAddress);
        Wire.write(reg);
        Wire.write(val);
        byte error = Wire.endTransmission();
        if (error != 0 && is_initialized) {
             // Only print if we were already running, to avoid spam during search
             // Serial.printf("I2C Write Error: %d at reg 0x%02X\n", error, reg);
        }
        return (error == 0);
    }

    uint8_t readReg(uint8_t reg, bool *success = nullptr) {
        Wire.beginTransmission(kImuAddress);
        Wire.write(reg);
        byte error = Wire.endTransmission(false);
        if (error != 0) {
            if (success) *success = false;
            return 0;
        }
        uint8_t count = Wire.requestFrom(kImuAddress, (uint8_t)1);
        if (count == 0) {
            if (success) *success = false;
            return 0;
        }
        if (success) *success = true;
        return Wire.read();
    }
}

// Accelerometer Calibration Parameters
// Matrix R
constexpr float ACC_CAL_R00 = 0.9996914216f;
constexpr float ACC_CAL_R01 = -0.0308126859f;
constexpr float ACC_CAL_R02 = 0.0510088819f;
constexpr float ACC_CAL_R10 = 0.0164932456f;
constexpr float ACC_CAL_R11 = 1.0006602403f;
constexpr float ACC_CAL_R12 = 0.0134431725f;
constexpr float ACC_CAL_R20 = -0.0312931478f;
constexpr float ACC_CAL_R21 = -0.0294243653f;
constexpr float ACC_CAL_R22 = 1.028111305f;
// Bias Vector
constexpr float ACC_CAL_B0  = -0.720579444f;
constexpr float ACC_CAL_B1  = -0.2742577706f;
constexpr float ACC_CAL_B2  = 0.5614902379f;

// Gyroscope Calibration Parameters
// Matrix R
constexpr float GYRO_CAL_R00 = 0.998848f;
constexpr float GYRO_CAL_R01 = -0.024033f;
constexpr float GYRO_CAL_R02 = 0.041541f;
constexpr float GYRO_CAL_R10 = 0.023254f;
constexpr float GYRO_CAL_R11 = 0.999546f;
constexpr float GYRO_CAL_R12 = 0.019138f;
constexpr float GYRO_CAL_R20 = -0.041982f;
constexpr float GYRO_CAL_R21 = -0.018150f;
constexpr float GYRO_CAL_R22 = 0.998954f;
// Bias Vector
constexpr float GYRO_CAL_B0  = -0.02208f;
constexpr float GYRO_CAL_B1  = 0.02572f;
constexpr float GYRO_CAL_B2  = 0.02238f;

void hal_imu_init() {
    if (!Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, 400000)) {
        Serial.println("Failed to initialize I2C bus");
        return;
    }
    Wire.setTimeOut(10); // 10ms timeout to prevent hanging
    
    Serial.println("IMU: Searching for QMI8658...");
    
    // Check WHO_AM_I with retries
    uint8_t whoAmI = 0;
    bool found = false;
    for (int i = 0; i < 5; i++) {
        bool success = false;
        whoAmI = readReg(kQmiWhoAmIReg, &success);
        if (success && whoAmI == kQmiWhoAmIValue) {
            found = true;
            break;
        }
        Serial.printf("IMU: WHO_AM_I check failed (attempt %d/5), got 0x%02X\n", i+1, whoAmI);
        delay(50);
    }

    if (!found) {
        Serial.printf("QMI8658 not found! Final WHO_AM_I=0x%02X\n", whoAmI);
        return;
    }

    // Reset
    if (!writeReg(kQmiResetReg, 0xB0)) {
        Serial.println("IMU: Reset command failed");
        return;
    }
    delay(50); // Give it some time to reset

    // Config
    bool ok = true;
    ok &= writeReg(kQmiCtrl1Reg, 0x40); // Address auto-increment
    ok &= writeReg(kQmiCtrl2Reg, kCtrl2Accel8gOdr2000Hz);
    ok &= writeReg(kQmiCtrl3Reg, kCtrl3Gyro512dpsOdr2000Hz);
    ok &= writeReg(kQmiCtrl5Reg, kCtrl5Lpf106Hz);
    ok &= writeReg(kQmiCtrl7Reg, 0x03); // Enable accel and gyro

    if (!ok) {
        Serial.println("IMU: Configuration failed");
        return;
    }

    is_initialized = true;
    Serial.println("QMI8658 initialized successfully");
}

bool hal_imu_healthy() {
    return is_initialized;
}

void hal_imu_read(imu_data_t *data) {
    if (!is_initialized) return;

    // Read status
    // Use readReg directly to avoid overhead, but ensure it doesn't block
    bool success = false;
    uint8_t status = readReg(kQmiStatus0Reg, &success);
    if (!success) {
        is_initialized = false; // Mark IMU unhealthy on bus/read failure.
        return;
    }
    if (!(status & 0x03)) return; // No new data

    // Read 14 bytes: temp(2) + accel(6) + gyro(6)
    Wire.beginTransmission(kImuAddress);
    Wire.write(kQmiTempLowReg);
    if (Wire.endTransmission(false) != 0) {
        is_initialized = false;
        return;
    }
    uint8_t count = Wire.requestFrom(kImuAddress, (uint8_t)14);
    if (count != 14) {
        is_initialized = false;
        return;
    }

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
    float raw_x = (float)ax_raw * kAccelScale;
    float raw_y = (float)az_raw * kAccelScale;
    float raw_z = -(float)ay_raw * kAccelScale;

    data->accl[0] = ACC_CAL_R00 * raw_x + ACC_CAL_R01 * raw_y + ACC_CAL_R02 * raw_z + ACC_CAL_B0;
    data->accl[1] = ACC_CAL_R10 * raw_x + ACC_CAL_R11 * raw_y + ACC_CAL_R12 * raw_z + ACC_CAL_B1;
    data->accl[2] = ACC_CAL_R20 * raw_x + ACC_CAL_R21 * raw_y + ACC_CAL_R22 * raw_z + ACC_CAL_B2;

    float raw_gx = (float)gx_raw * kGyroScale;
    float raw_gy = (float)gz_raw * kGyroScale;
    float raw_gz = -(float)gy_raw * kGyroScale;

    data->gyro[0] = GYRO_CAL_R00 * (raw_gx - GYRO_CAL_B0) + GYRO_CAL_R01 * (raw_gy - GYRO_CAL_B1) + GYRO_CAL_R02 * (raw_gz - GYRO_CAL_B2);
    data->gyro[1] = GYRO_CAL_R10 * (raw_gx - GYRO_CAL_B0) + GYRO_CAL_R11 * (raw_gy - GYRO_CAL_B1) + GYRO_CAL_R12 * (raw_gz - GYRO_CAL_B2);
    data->gyro[2] = GYRO_CAL_R20 * (raw_gx - GYRO_CAL_B0) + GYRO_CAL_R21 * (raw_gy - GYRO_CAL_B1) + GYRO_CAL_R22 * (raw_gz - GYRO_CAL_B2);
}
