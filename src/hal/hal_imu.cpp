#include "hal_imu.h"
#include <Arduino.h>
#include <Wire.h>

namespace {
    // ICM20948 Registers
    constexpr uint8_t kImuAddress = 0x68;
    constexpr uint8_t kRegBankSel = 0x7F;

    // User Bank 0 Registers
    constexpr uint8_t kRegWhoAmI = 0x00;
    constexpr uint8_t kRegWhoAmIVal = 0xEA;
    constexpr uint8_t kRegUserCtrl = 0x03;
    constexpr uint8_t kRegPwrMgmt1 = 0x06;
    constexpr uint8_t kRegIntPinCfg = 0x0F;
    constexpr uint8_t kRegIntEnable1 = 0x11;
    constexpr uint8_t kRegAccelXOutH = 0x2D;

    // User Bank 2 Registers
    constexpr uint8_t kRegGyroSmplrtDiv = 0x00;
    constexpr uint8_t kRegGyroConfig1 = 0x01;
    constexpr uint8_t kRegAccelSmplrtDiv2 = 0x11;
    constexpr uint8_t kRegAccelConfig = 0x14;

    // User Bank 3 Registers
    constexpr uint8_t kRegI2cSlv0Addr = 0x03;
    constexpr uint8_t kRegI2cSlv0Reg = 0x04;
    constexpr uint8_t kRegI2cSlv0Ctrl = 0x05;
    constexpr uint8_t kRegI2cSlv1Addr = 0x07;
    constexpr uint8_t kRegI2cSlv1Reg = 0x08;
    constexpr uint8_t kRegI2cSlv1Ctrl = 0x09;
    constexpr uint8_t kRegI2cSlv1Do = 0x0A;
    constexpr uint8_t kRegExtSensData00 = 0x3B;

    // AK09916 Magnetometer Registers
    constexpr uint8_t kMagAddress = 0x0C;
    constexpr uint8_t kRegMagWia1 = 0x00;
    constexpr uint8_t kRegMagWia2 = 0x01;
    constexpr uint8_t kRegMagStatus2 = 0x10;
    constexpr uint8_t kRegMagDataStart = 0x11;
    constexpr uint8_t kRegMagCntl2 = 0x31;

    // Configuration constants
    constexpr uint8_t kValPwrMgmtReset = 0x80;
    constexpr uint8_t kValPwrMgmtPll = 0x01;
    constexpr uint8_t kValIntPinCfg = 0x30;      // Latch, Active High, Push-Pull, Clear on Any Read
    constexpr uint8_t kValIntEnable1 = 0x01;     // Raw Data Ready Interrupt
    constexpr uint8_t kValI2cMstEn = 0x20;
    constexpr uint8_t kValSlvEn = 0x80;
    constexpr uint8_t kValMagMode20Hz = 0x04;

    // Scaling constants for ±8g and ±500dps
    constexpr float kAccelScale = (8.0f / 32768.0f) * 9.80665f; // to m/s^2
    constexpr float kGyroScale = (500.0f / 32768.0f) * DEG_TO_RAD; // to rad/s
    constexpr float kTempScale = 1.0f / 333.87f;

    static bool is_initialized = false;
    static bool is_mag_initialized = false;
    static uint32_t i2c_error_count = 0;

    static SemaphoreHandle_t imu_sem = nullptr;
    static SemaphoreHandle_t i2c_mutex = nullptr;

    void IRAM_ATTR imu_isr_handler() {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (imu_sem) {
            xSemaphoreGiveFromISR(imu_sem, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken) {
                portYIELD_FROM_ISR();
            }
        }
    }

    bool selectBank(uint8_t bank) {
        Wire.beginTransmission(kImuAddress);
        Wire.write(kRegBankSel);
        Wire.write(bank << 4);
        return (Wire.endTransmission() == 0);
    }

    bool writeReg(uint8_t bank, uint8_t reg, uint8_t val) {
        if (!selectBank(bank)) return false;
        Wire.beginTransmission(kImuAddress);
        Wire.write(reg);
        Wire.write(val);
        return (Wire.endTransmission() == 0);
    }

    uint8_t readReg(uint8_t bank, uint8_t reg, bool *success = nullptr) {
        if (!selectBank(bank)) {
            if (success) *success = false;
            return 0;
        }
        Wire.beginTransmission(kImuAddress);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) {
            if (success) *success = false;
            return 0;
        }
        if (Wire.requestFrom(kImuAddress, (uint8_t)1) != 1) {
            if (success) *success = false;
            return 0;
        }
        if (success) *success = true;
        return Wire.read();
    }

    bool readRegs(uint8_t bank, uint8_t start_reg, uint8_t *buf, uint8_t len) {
        if (!selectBank(bank)) return false;
        Wire.beginTransmission(kImuAddress);
        Wire.write(start_reg);
        if (Wire.endTransmission(false) != 0) return false;
        if (Wire.requestFrom(kImuAddress, len) != len) return false;
        for (uint8_t i = 0; i < len; i++) {
            buf[i] = Wire.read();
        }
        return true;
    }

    bool writeSecondary(uint8_t dev_addr, uint8_t reg, uint8_t val) {
        bool ok = true;
        ok &= writeReg(3, kRegI2cSlv1Addr, dev_addr | 0x00); // 0x00 for write
        ok &= writeReg(3, kRegI2cSlv1Reg, reg);
        ok &= writeReg(3, kRegI2cSlv1Do, val);
        ok &= writeReg(3, kRegI2cSlv1Ctrl, kValSlvEn | 1); // Enable SLV1 and write 1 byte

        // Switch to Bank 0 to enable I2C Master temporarily
        uint8_t user_ctrl = readReg(0, kRegUserCtrl, &ok);
        if (!ok) return false;
        ok &= writeReg(0, kRegUserCtrl, user_ctrl | kValI2cMstEn);
        delayMicroseconds(150); // 150us is safe for 1-byte write (was 500us)
        ok &= writeReg(0, kRegUserCtrl, user_ctrl & ~kValI2cMstEn);
        return ok;
    }

    bool readSecondary(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint8_t len) {
        bool ok = true;
        ok &= writeReg(3, kRegI2cSlv0Addr, dev_addr | 0x80); // 0x80 for read
        ok &= writeReg(3, kRegI2cSlv0Reg, reg);
        ok &= writeReg(3, kRegI2cSlv0Ctrl, kValSlvEn | len); // Enable SLV0 and read len bytes

        // Trigger read via USER_CTRL in Bank 0
        uint8_t user_ctrl = readReg(0, kRegUserCtrl, &ok);
        if (!ok) return false;
        ok &= writeReg(0, kRegUserCtrl, user_ctrl | kValI2cMstEn);
        
        // 345kHz I2C Master speed: 7 bytes read takes ~235us.
        // We set delay to 300us for 7-byte read, and 100us for 1-byte read.
        uint32_t delay_us = (len == 1) ? 100 : (100 + len * 30);
        delayMicroseconds(delay_us); 

        // Read from external sensor registers in Bank 0
        ok &= readRegs(0, kRegExtSensData00, buf, len);
        return ok;
    }
}

// Accelerometer Calibration Parameters (Identity matrix and zero bias)
constexpr float ACC_CAL_R00 = 1.0f;
constexpr float ACC_CAL_R01 = 0.0f;
constexpr float ACC_CAL_R02 = 0.0f;
constexpr float ACC_CAL_R10 = 0.0f;
constexpr float ACC_CAL_R11 = 1.0f;
constexpr float ACC_CAL_R12 = 0.0f;
constexpr float ACC_CAL_R20 = 0.0f;
constexpr float ACC_CAL_R21 = 0.0f;
constexpr float ACC_CAL_R22 = 1.0f;
constexpr float ACC_CAL_B0  = 0.0f;
constexpr float ACC_CAL_B1  = 0.0f;
constexpr float ACC_CAL_B2  = 0.0f;

// Gyroscope Calibration Parameters (Identity matrix and zero bias)
constexpr float GYRO_CAL_R00 = 1.0f;
constexpr float GYRO_CAL_R01 = 0.0f;
constexpr float GYRO_CAL_R02 = 0.0f;
constexpr float GYRO_CAL_R10 = 0.0f;
constexpr float GYRO_CAL_R11 = 1.0f;
constexpr float GYRO_CAL_R12 = 0.0f;
constexpr float GYRO_CAL_R20 = 0.0f;
constexpr float GYRO_CAL_R21 = 0.0f;
constexpr float GYRO_CAL_R22 = 1.0f;
constexpr float GYRO_CAL_B0  = 0.0f;
constexpr float GYRO_CAL_B1  = 0.0f;
constexpr float GYRO_CAL_B2  = 0.0f;

void hal_imu_init() {
    i2c_mutex = xSemaphoreCreateMutex();
    if (!Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, 400000)) {
        Serial.println("Failed to initialize I2C bus");
        return;
    }
    Wire.setTimeOut(2); // 2ms timeout is safer for 3ms control rate (was 10ms)
    
    Serial.println("IMU: Searching for ICM20948...");
    
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    
    // Check WHO_AM_I with retries
    uint8_t whoAmI = 0;
    bool found = false;
    for (int i = 0; i < 5; i++) {
        bool success = false;
        whoAmI = readReg(0, kRegWhoAmI, &success);
        if (success && whoAmI == kRegWhoAmIVal) {
            found = true;
            break;
        }
        Serial.printf("IMU: ICM20948 WHO_AM_I check failed (attempt %d/5), got 0x%02X\n", i+1, whoAmI);
        delay(50);
    }

    if (!found) {
        Serial.printf("ICM20948 not found! Final WHO_AM_I=0x%02X\n", whoAmI);
        xSemaphoreGive(i2c_mutex);
        return;
    }

    // Reset device
    if (!writeReg(0, kRegPwrMgmt1, kValPwrMgmtReset)) {
        Serial.println("IMU: Reset command failed");
        xSemaphoreGive(i2c_mutex);
        return;
    }
    delay(100); // Give it some time to reset

    // Wake up & select clock
    bool ok = true;
    ok &= writeReg(0, kRegPwrMgmt1, kValPwrMgmtPll);
    
    // Config Data Ready interrupt output
    ok &= writeReg(0, kRegIntPinCfg, kValIntPinCfg);
    ok &= writeReg(0, kRegIntEnable1, kValIntEnable1);

    // Config Sample rates and DLPF in Bank 2
    // Gyro ODR = 1125 / (1 + 1) = 562.5Hz. Config range = ±500dps, DLPF = 50.4Hz
    ok &= writeReg(2, kRegGyroSmplrtDiv, 0x01);
    ok &= writeReg(2, kRegGyroConfig1, 0x1B);
    
    // Accel ODR = 1125 / (1 + 1) = 562.5Hz. Config range = ±8g, DLPF = 50.4Hz
    ok &= writeReg(2, kRegAccelSmplrtDiv2, 0x01);
    ok &= writeReg(2, kRegAccelConfig, 0x1D);

    // Make sure we end up in Bank 0 for continuous reading
    ok &= selectBank(0);

    if (!ok) {
        Serial.println("IMU: Configuration failed");
        xSemaphoreGive(i2c_mutex);
        return;
    }

    // Initialize FreeRTOS semaphore for Data Ready Interrupt
    imu_sem = xSemaphoreCreateBinary();

    // Configure GPIO for IMU INT
    pinMode(IMU_INT1_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN), imu_isr_handler, RISING);

    is_initialized = true;
    Serial.println("ICM20948 initialized successfully at 562.5Hz ODR / 50.4Hz DLPF with interrupt");
    
    xSemaphoreGive(i2c_mutex);
}

bool hal_imu_healthy() {
    return is_initialized;
}

bool hal_imu_read(imu_data_t *data) {
    if (!is_initialized) return false;

    uint8_t buf[14];
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    
    if (!readRegs(0, kRegAccelXOutH, buf, 14)) {
        i2c_error_count++;
        xSemaphoreGive(i2c_mutex);
        return false;
    }
    
    xSemaphoreGive(i2c_mutex);

    // ICM20948 register layout is: Accel X/Y/Z -> Gyro X/Y/Z -> Temp
    int16_t ax_raw = (int16_t)(buf[0] << 8 | buf[1]);
    int16_t ay_raw = (int16_t)(buf[2] << 8 | buf[3]);
    int16_t az_raw = (int16_t)(buf[4] << 8 | buf[5]);
    int16_t gx_raw = (int16_t)(buf[6] << 8 | buf[7]);
    int16_t gy_raw = (int16_t)(buf[8] << 8 | buf[9]);
    int16_t gz_raw = (int16_t)(buf[10] << 8 | buf[11]);
    int16_t t_raw  = (int16_t)(buf[12] << 8 | buf[13]);

    float temp_c = (float)t_raw * kTempScale + 21.0f;

    // Scale raw values
    float raw_x = -(float)ay_raw * kAccelScale;
    float raw_y = (float)ax_raw * kAccelScale;
    float raw_z = (float)az_raw * kAccelScale;

    float acc_val[3];
    acc_val[0] = ACC_CAL_R00 * raw_x + ACC_CAL_R01 * raw_y + ACC_CAL_R02 * raw_z + ACC_CAL_B0;
    acc_val[1] = ACC_CAL_R10 * raw_x + ACC_CAL_R11 * raw_y + ACC_CAL_R12 * raw_z + ACC_CAL_B1;
    acc_val[2] = ACC_CAL_R20 * raw_x + ACC_CAL_R21 * raw_y + ACC_CAL_R22 * raw_z + ACC_CAL_B2;

    float raw_gx = -(float)gy_raw * kGyroScale;
    float raw_gy = (float)gx_raw * kGyroScale;
    float raw_gz = (float)gz_raw * kGyroScale;

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

// Magnetometer (AK09916) low-level support for hal_mag
void hal_imu_mag_init() {
    if (!is_initialized) return;
    
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    
    // Check WIA WIA of magnetometer
    uint8_t wia[2] = {0};
    if (readSecondary(kMagAddress, kRegMagWia1, wia, 2)) {
        if (wia[0] == kRegMagWia1 || wia[0] == 0x48) { // check whoami
            // Write Mode_20Hz
            if (writeSecondary(kMagAddress, kRegMagCntl2, kValMagMode20Hz)) {
                is_mag_initialized = true;
                Serial.println("AK09916 Magnetometer initialized successfully at 20Hz");
            }
        }
    }
    
    if (!is_mag_initialized) {
        Serial.println("AK09916 Magnetometer initialization failed");
    }
    
    // Make sure we select bank 0
    selectBank(0);
    xSemaphoreGive(i2c_mutex);
}

bool hal_imu_mag_healthy() {
    return is_mag_initialized;
}

bool hal_imu_mag_read(mag_data_t *data) {
    if (!is_mag_initialized) return false;

    uint8_t buf[7];
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    
    // Directly read ST1 + data (7 bytes starting from ST1 0x10)
    // kRegMagStatus2 is 0x10 (ST1 status register)
    bool read_ok = readSecondary(kMagAddress, kRegMagStatus2, buf, 7);
    
    if (read_ok) {
        // buf[0] is ST1, buf[1]..buf[6] is X_L, X_H, Y_L, Y_H, Z_L, Z_H
        int16_t mx = (int16_t)(buf[2] << 8 | buf[1]);
        int16_t my = (int16_t)(buf[4] << 8 | buf[3]);
        int16_t mz = (int16_t)(buf[6] << 8 | buf[5]);
        
        // AK09916 resolution is 0.15 uT per LSB
        data->mag[0] = (float)mx * 0.15f;
        data->mag[1] = (float)my * 0.15f;
        data->mag[2] = (float)mz * 0.15f;
        data->timestamp = micros();
    }
    
    selectBank(0);
    xSemaphoreGive(i2c_mutex);
    return read_ok;
}

void hal_imu_i2c_lock() {
    if (i2c_mutex) {
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    }
}

void hal_imu_i2c_unlock() {
    if (i2c_mutex) {
        xSemaphoreGive(i2c_mutex);
    }
}
