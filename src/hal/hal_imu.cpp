#include "hal_imu.h"
#include <Wire.h>

namespace {
    // MPU6050 Registers
    constexpr uint8_t kImuAddress = 0x68; // Standard MPU6050 address (AD0 low)
    constexpr uint8_t kMpuWhoAmIReg = 0x75;
    constexpr uint8_t kMpuWhoAmIValue = 0x68;
    constexpr uint8_t kMpuPwrMgmt1Reg = 0x6B;
    constexpr uint8_t kMpuSmplrtDivReg = 0x19;
    constexpr uint8_t kMpuConfigReg = 0x1A;
    constexpr uint8_t kMpuGyroConfigReg = 0x1B;
    constexpr uint8_t kMpuAccelConfigReg = 0x1C;
    constexpr uint8_t kMpuAccelXOutHReg = 0x3B;
    constexpr uint8_t kMpuIntPinCfgReg = 0x37;
    constexpr uint8_t kMpuIntEnableReg = 0x38;

    // Config values
    constexpr uint8_t kMpuAccel8gVal = 0x10;       // Range +/- 8G
    constexpr uint8_t kMpuGyro500dpsVal = 0x08;    // Range +/- 500 deg/s
    constexpr uint8_t kMpuDpf44HzVal = 0x03;       // DLPF Bandwidth 44Hz (Fs=1kHz)
    constexpr uint8_t kMpuSmplrtDivVal = 0x03;     // 1kHz / (1 + 3) = 250Hz sample rate

    constexpr float kAccelScale = (8.0f / 32768.0f) * 9.80665f; // to m/s^2
    constexpr float kGyroScale = (500.0f / 32768.0f) * DEG_TO_RAD; // to rad/s
    constexpr float kTempScale = 1.0f / 340.0f;

    static bool is_initialized = false;
    static uint32_t i2c_error_count = 0;

    static SemaphoreHandle_t imu_sem = nullptr;

    void IRAM_ATTR imu_isr_handler() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (imu_sem) {
            xSemaphoreGiveFromISR(imu_sem, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken) {
                portYIELD_FROM_ISR();
            }
        }
    }

    bool writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(kImuAddress);
        Wire.write(reg);
        Wire.write(val);
        byte error = Wire.endTransmission();
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

// Accelerometer Calibration Parameters (Identity matrix and zero bias)
// Matrix R
constexpr float ACC_CAL_R00 = 1.0f;
constexpr float ACC_CAL_R01 = 0.0f;
constexpr float ACC_CAL_R02 = 0.0f;
constexpr float ACC_CAL_R10 = 0.0f;
constexpr float ACC_CAL_R11 = 1.0f;
constexpr float ACC_CAL_R12 = 0.0f;
constexpr float ACC_CAL_R20 = 0.0f;
constexpr float ACC_CAL_R21 = 0.0f;
constexpr float ACC_CAL_R22 = 1.0f;
// Bias Vector
constexpr float ACC_CAL_B0  = 0.0f;
constexpr float ACC_CAL_B1  = 0.0f;
constexpr float ACC_CAL_B2  = 0.0f;

// Gyroscope Calibration Parameters (Identity matrix and zero bias)
// Matrix R
constexpr float GYRO_CAL_R00 = 1.0f;
constexpr float GYRO_CAL_R01 = 0.0f;
constexpr float GYRO_CAL_R02 = 0.0f;
constexpr float GYRO_CAL_R10 = 0.0f;
constexpr float GYRO_CAL_R11 = 1.0f;
constexpr float GYRO_CAL_R12 = 0.0f;
constexpr float GYRO_CAL_R20 = 0.0f;
constexpr float GYRO_CAL_R21 = 0.0f;
constexpr float GYRO_CAL_R22 = 1.0f;
// Bias Vector
constexpr float GYRO_CAL_B0  = 0.0f;
constexpr float GYRO_CAL_B1  = 0.0f;
constexpr float GYRO_CAL_B2  = 0.0f;

void hal_imu_init() {
    if (!Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, 400000)) {
        Serial.println("Failed to initialize I2C bus");
        return;
    }
    Wire.setTimeOut(10); // 10ms timeout to prevent hanging
    
    Serial.println("IMU: Searching for MPU6050...");
    
    // Check WHO_AM_I with retries
    uint8_t whoAmI = 0;
    bool found = false;
    for (int i = 0; i < 5; i++) {
        bool success = false;
        whoAmI = readReg(kMpuWhoAmIReg, &success);
        if (success && whoAmI == kMpuWhoAmIValue) {
            found = true;
            break;
        }
        Serial.printf("IMU: MPU6050 WHO_AM_I check failed (attempt %d/5), got 0x%02X\n", i+1, whoAmI);
        delay(50);
    }

    if (!found) {
        Serial.printf("MPU6050 not found! Final WHO_AM_I=0x%02X\n", whoAmI);
        return;
    }

    // Reset device
    if (!writeReg(kMpuPwrMgmt1Reg, 0x80)) {
        Serial.println("IMU: Reset command failed");
        return;
    }
    delay(100); // Give it some time to reset

    // Config MPU6050 registers
    bool ok = true;
    ok &= writeReg(kMpuPwrMgmt1Reg, 0x03);      // Clear sleep & set clock source to PLL with Gyro X reference
    ok &= writeReg(kMpuConfigReg, kMpuDpf44HzVal);
    ok &= writeReg(kMpuSmplrtDivReg, kMpuSmplrtDivVal);
    ok &= writeReg(kMpuGyroConfigReg, kMpuGyro500dpsVal);
    ok &= writeReg(kMpuAccelConfigReg, kMpuAccel8gVal);
    ok &= writeReg(kMpuIntPinCfgReg, 0x30);   // Latch active-high, clear on any read
    ok &= writeReg(kMpuIntEnableReg, 0x01);  // Enable Data Ready interrupt

    if (!ok) {
        Serial.println("IMU: Configuration failed");
        return;
    }

    // Initialize FreeRTOS semaphore for Data Ready Interrupt
    imu_sem = xSemaphoreCreateBinary();

    // Configure GPIO for IMU INT
    pinMode(IMU_INT1_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN), imu_isr_handler, RISING);

    is_initialized = true;
    Serial.println("MPU6050 initialized successfully at 250Hz ODR / 44Hz LPF with interrupt");
}

bool hal_imu_healthy() {
    return is_initialized;
}

bool hal_imu_read(imu_data_t *data) {
    if (!is_initialized) return false;

    // Read 14 bytes directly starting from AccelXOutH register.
    Wire.beginTransmission(kImuAddress);
    Wire.write(kMpuAccelXOutHReg);
    if (Wire.endTransmission(false) != 0) {
        i2c_error_count++;
        is_initialized = false;
        return false;
    }
    uint8_t count = Wire.requestFrom(kImuAddress, (uint8_t)14);
    if (count != 14) {
        i2c_error_count++;
        is_initialized = false;
        return false;
    }

    uint8_t buf[14];
    for (int i = 0; i < 14; i++) {
        int val = Wire.read();
        if (val == -1) {
            i2c_error_count++;
            // Transient read timeout/error detected mid-stream!
            return false;
        }
        buf[i] = (uint8_t)val;
    }

    int16_t ax_raw = (int16_t)(buf[0] << 8 | buf[1]);
    int16_t ay_raw = (int16_t)(buf[2] << 8 | buf[3]);
    int16_t az_raw = (int16_t)(buf[4] << 8 | buf[5]);
    int16_t t_raw  = (int16_t)(buf[6] << 8 | buf[7]);
    int16_t gx_raw = (int16_t)(buf[8] << 8 | buf[9]);
    int16_t gy_raw = (int16_t)(buf[10] << 8 | buf[11]);
    int16_t gz_raw = (int16_t)(buf[12] << 8 | buf[13]);

    float temp_c = (float)t_raw * kTempScale + 36.53f;

    // Map raw data to m/s^2 and rad/s
    // Restored MPU6050 axis mapping from commit 05f1b1e:
    // raw_x = -ax, raw_y = ay, raw_z = -az
    // raw_gx = -gx, raw_gy = gy, raw_gz = -gz
    float raw_x = -(float)ax_raw * kAccelScale;
    float raw_y = (float)ay_raw * kAccelScale;
    float raw_z = -(float)az_raw * kAccelScale;

    float acc_val[3];
    acc_val[0] = ACC_CAL_R00 * raw_x + ACC_CAL_R01 * raw_y + ACC_CAL_R02 * raw_z + ACC_CAL_B0;
    acc_val[1] = ACC_CAL_R10 * raw_x + ACC_CAL_R11 * raw_y + ACC_CAL_R12 * raw_z + ACC_CAL_B1;
    acc_val[2] = ACC_CAL_R20 * raw_x + ACC_CAL_R21 * raw_y + ACC_CAL_R22 * raw_z + ACC_CAL_B2;

    float raw_gx = -(float)gx_raw * kGyroScale;
    float raw_gy = (float)gy_raw * kGyroScale;
    float raw_gz = -(float)gz_raw * kGyroScale;

    float gyro_val[3];
    gyro_val[0] = GYRO_CAL_R00 * (raw_gx - GYRO_CAL_B0) + GYRO_CAL_R01 * (raw_gy - GYRO_CAL_B1) + GYRO_CAL_R02 * (raw_gz - GYRO_CAL_B2);
    gyro_val[1] = GYRO_CAL_R10 * (raw_gx - GYRO_CAL_B0) + GYRO_CAL_R11 * (raw_gy - GYRO_CAL_B1) + GYRO_CAL_R12 * (raw_gz - GYRO_CAL_B2);
    gyro_val[2] = GYRO_CAL_R20 * (raw_gx - GYRO_CAL_B0) + GYRO_CAL_R21 * (raw_gy - GYRO_CAL_B1) + GYRO_CAL_R22 * (raw_gz - GYRO_CAL_B2);

    data->timestamp = micros();
    data->temp = temp_c;
    std::copy(acc_val, acc_val + 3, data->accl);
    std::copy(gyro_val, gyro_val + 3, data->gyro);

    return true;
}

uint32_t hal_imu_get_error_count() {
    return i2c_error_count;
}

bool hal_imu_wait_for_data(uint32_t timeout_ms) {
    if (!imu_sem) return false;
    return xSemaphoreTake(imu_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}
