#include "hal_baro.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "config.h"
#include "hal_imu.h"

namespace {
    static uint8_t baro_address = 0x77; // Default address
    static bool is_initialized = false;

    // Registers
    constexpr uint8_t kRegCalibStart = 0x88;
    constexpr uint8_t kRegChipId = 0xD0;
    constexpr uint8_t kRegReset = 0xE0;
    constexpr uint8_t kRegStatus = 0xF3;
    constexpr uint8_t kRegControl = 0xF4;
    constexpr uint8_t kRegConfig = 0xF5;
    constexpr uint8_t kRegDataStart = 0xF7;

    // Expected Chip ID
    constexpr uint8_t kChipIdBmp280 = 0x58;

    // Calibration Parameters
    struct {
        uint16_t T1;
        int16_t T2;
        int16_t T3;
        uint16_t P1;
        int16_t P2;
        int16_t P3;
        int16_t P4;
        int16_t P5;
        int16_t P6;
        int16_t P7;
        int16_t P8;
        int16_t P9;
        int32_t t_fine;
    } calib;

    bool writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(baro_address);
        Wire.write(reg);
        Wire.write(val);
        return (Wire.endTransmission() == 0);
    }

    uint8_t readReg(uint8_t reg, bool *success = nullptr) {
        Wire.beginTransmission(baro_address);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) {
            if (success) *success = false;
            return 0;
        }
        if (Wire.requestFrom(baro_address, (uint8_t)1) != 1) {
            if (success) *success = false;
            return 0;
        }
        if (success) *success = true;
        return Wire.read();
    }

    bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
        Wire.beginTransmission(baro_address);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) return false;
        if (Wire.requestFrom(baro_address, len) != len) return false;
        for (uint8_t i = 0; i < len; i++) {
            buf[i] = Wire.read();
        }
        return true;
    }

    bool loadCalibration() {
        uint8_t buf[24];
        if (!readRegs(kRegCalibStart, buf, 24)) return false;

        calib.T1 = (uint16_t)(buf[1] << 8 | buf[0]);
        calib.T2 = (int16_t)(buf[3] << 8 | buf[2]);
        calib.T3 = (int16_t)(buf[5] << 8 | buf[4]);
        calib.P1 = (uint16_t)(buf[7] << 8 | buf[6]);
        calib.P2 = (int16_t)(buf[9] << 8 | buf[8]);
        calib.P3 = (int16_t)(buf[11] << 8 | buf[10]);
        calib.P4 = (int16_t)(buf[13] << 8 | buf[12]);
        calib.P5 = (int16_t)(buf[15] << 8 | buf[14]);
        calib.P6 = (int16_t)(buf[17] << 8 | buf[16]);
        calib.P7 = (int16_t)(buf[19] << 8 | buf[18]);
        calib.P8 = (int16_t)(buf[21] << 8 | buf[20]);
        calib.P9 = (int16_t)(buf[23] << 8 | buf[22]);
        return true;
    }

    float compensateTemperature(int32_t adc_T) {
        int64_t var1, var2, T;
        var1 = ((((adc_T >> 3) - ((int64_t)calib.T1 << 1))) * ((int64_t)calib.T2)) >> 11;
        var2 = (((((adc_T >> 4) - ((int64_t)calib.T1)) * ((adc_T >> 4) - ((int64_t)calib.T1))) >> 12) * ((int64_t)calib.T3)) >> 14;
        calib.t_fine = (int32_t)(var1 + var2);
        T = (calib.t_fine * 5 + 128) >> 8;
        return (float)T / 100.0f;
    }

    float compensatePressure(int32_t adc_P) {
        int64_t var1, var2, p;
        var1 = ((int64_t)calib.t_fine) - 128000;
        var2 = var1 * var1 * (int64_t)calib.P6;
        var2 = var2 + ((var1 * (int64_t)calib.P5) << 17);
        var2 = var2 + (((int64_t)calib.P4) << 35);
        var1 = ((var1 * var1 * (int64_t)calib.P3) >> 8) + ((var1 * (int64_t)calib.P2) << 12);
        var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.P1) >> 33;

        if (var1 == 0) {
            return 0; // avoid exception caused by division by zero
        }

        p = 1048576 - adc_P;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (((int64_t)calib.P9) * (p >> 13) * (p >> 13)) >> 25;
        var2 = (((int64_t)calib.P8) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + (((int64_t)calib.P7) << 4);
        return (float)p / 256.0f; // pressure in Pa
    }
}

void hal_baro_init() {
    Serial.println("Baro: Searching for BMP280...");
    
    // Scan address 0x77 first, then 0x76
    uint8_t chip_id = 0;
    bool found = false;
    uint8_t addrs[] = {0x77, 0x76};
    
    for (uint8_t addr : addrs) {
        baro_address = addr;
        bool success = false;
        chip_id = readReg(kRegChipId, &success);
        if (success && chip_id == kChipIdBmp280) {
            found = true;
            break;
        }
    }

    if (!found) {
        Serial.printf("BMP280 not found! Last read ID=0x%02X\n", chip_id);
        return;
    }

    Serial.printf("BMP280 found at address 0x%02X\n", baro_address);

    // Initialize BMP280 registers
    // Normal mode, 16x temp oversampling, 16x press oversampling
    bool ok = true;
    ok &= writeReg(kRegControl, 0xFF);
    // 0.5ms standby time, filter coefficient 16
    ok &= writeReg(kRegConfig, 0x14);

    if (!ok || !loadCalibration()) {
        Serial.println("BMP280 calibration load or config failed");
        return;
    }

    is_initialized = true;
    Serial.println("BMP280 initialized successfully");
}

bool hal_baro_healthy() {
    return is_initialized;
}

bool hal_baro_read(baro_data_t *data) {
    if (!is_initialized) return false;

    uint8_t buf[6];
    hal_imu_i2c_lock();
    bool ok = readRegs(kRegDataStart, buf, 6);
    hal_imu_i2c_unlock();
    if (!ok) {
        return false;
    }

    int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
    int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);

    float temp_c = compensateTemperature(adc_T);
    float press_pa = compensatePressure(adc_P);

    data->timestamp = micros();
    data->temperature = temp_c;
    data->pressure = press_pa / 100.0f; // Store in hPa
    
    // Altitude formula: 44330 * (1 - (p / p0)^0.1903)
    // Mean Sea Level Pressure = 1013.25 hPa
    data->altitude = 44330.0f * (1.0f - powf(data->pressure / 1013.25f, 0.1903f));

    return true;
}
