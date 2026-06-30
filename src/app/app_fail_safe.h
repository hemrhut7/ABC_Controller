#pragma once

#include <Arduino.h>
#include "hal/hal_type_define.h"
#include "processing/prs_ahrs.h"
#include "processing/prs_motor.h"

typedef enum {
    FS_ERROR_IMU_FAILED,        // 0 - 最致命硬體錯誤
    FS_ERROR_CRITICAL_ANGLE,    // 1 - 物理倒地錯誤
    FS_ERROR_LOOP_SLOW,         // 2 - 迴圈卡頓軟體錯誤
    FS_ERROR_AHRS_UNREADY,      // 3 - AHRS 初始化中
    FS_ERROR_PICKUP_DETECTED,   // 4 - 懸空保護
    FS_ERROR_NONE,              // 5 - 正常無錯誤，必須放在最後
} failsafe_error_t;

typedef void (*set_pending_mode_fn)(Mode_t);

class Failsafe {
public:
    Failsafe(float period_ms, Processing_Motor* motor, set_pending_mode_fn set_pending_mode_cb);

    void init();
    bool check(const ahrs_data_t &ahrs_data, AHRS_STATE ahrs_state, Mode_t current_mode);

    uint32_t get_loop_time_ms() { return loop_time_ms; }
    uint16_t get_delay_count() { return delay_counter; }
    failsafe_error_t get_error_state() const { return error_state; }
    bool is_ready_auto_start() const { return ready_auto_start; }

private:
    Processing_Motor* _motor;
    failsafe_error_t error_state = FS_ERROR_NONE;
    
    // 時間相關變數
    uint32_t last_check_time_us;
    uint16_t consecutive_slow_frames;
    const uint16_t MAX_CONSECUTIVE_SLOW_FRAMES = 5; // 容許最多連續 4 幀抖動，第 5 幀觸發保護
    uint32_t pickup_start_time;
    bool is_pickup_condition_met;

    // --- 閾值設定 ---

    // 1. 迴圈性能限制
    float MAX_LOOP_TIME_MS; 

    // 2. 倒地保護角度 (45度)
    const float CRITICAL_ANGLE_RAD = 60.0f * DEG_TO_RAD; 
    const float RECOVERY_ANGLE_RAD = 5.0f * DEG_TO_RAD;

    uint32_t loop_time_ms = 0;
    uint16_t delay_counter = 0;
    uint32_t last_disarm_time_ms = 0;
    bool ready_auto_start = false;

    set_pending_mode_fn _set_pending_mode_cb;
};
