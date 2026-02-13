#pragma once

#include <Arduino.h>


class Failsafe {
public:
    Failsafe();

    void init();

    /**
     * @brief 核心安全檢查
     * @param pitch_rad 當前傾角 (Rad)
     * @param pitch_rate_rad 當前角速度 (Rad/s) - 用於拿起偵測
     * @param rpm_l 左輪轉速 - 用於拿起偵測
     * @param rpm_r 右輪轉速 - 用於拿起偵測
     * @param current_time_ms 當前系統時間
     * @return true: 安全 (ARMED), false: 危險 (DISARMED)
     */
    bool check(float pitch_rad, float pitch_rate_rad, int rpm_l, int rpm_r, uint32_t current_time_ms);

    bool is_armed();

    uint32_t get_loop_time_ms() { return loop_time_ms; }

private:
    bool armed_state;
    
    // 時間相關變數
    uint32_t last_check_time;
    uint32_t pickup_start_time;
    bool is_pickup_condition_met;

    // --- 閾值設定 ---

    // 1. 迴圈性能限制
    // 如果迴圈超過 20ms (50Hz) 沒更新，視為系統卡頓
    const uint32_t MAX_LOOP_TIME_MS = 20; 

    // 2. 倒地保護角度 (45度)
    const float CRITICAL_ANGLE_RAD = 45.0f * 0.0174533f; 
    const float RECOVERY_ANGLE_RAD = 5.0f * 0.0174533f;

    // 3. 拿起偵測參數
    const int PICKUP_RPM_THRESHOLD = 300;     // 轉速超過 300 RPM
    const float PICKUP_GYRO_THRESHOLD = 15.0f * 0.0174533f; // 且角速度小於 15 deg/s
    const uint32_t PICKUP_CONFIRM_MS = 500;   // 持續 0.5 秒才觸發
    uint32_t loop_time_ms = 0;

};