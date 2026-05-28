#pragma once

#include <Arduino.h>
#include "hal/hal_type_define.h"
#include "processing/prs_ahrs.h"
#include "processing/prs_motor.h"

typedef enum {
    FS_ERROR_LOOP_SLOW,
    FS_ERROR_IMU_FAILED,
    FS_ERROR_AHRS_UNREADY,
    FS_ERROR_CRITICAL_ANGLE,
    FS_ERROR_PICKUP_DETECTED,
    FS_ERROR_NONE,  // should always be last
} failsafe_error_t;

typedef void (*set_pending_mode_fn)(Mode_t);

class Failsafe {
public:
    Failsafe(uint16_t period_ms, Processing_Motor* motor, set_pending_mode_fn set_pending_mode_cb);

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
    uint32_t last_check_time;
    uint32_t pickup_start_time;
    bool is_pickup_condition_met;

    // --- 閾值設定 ---

    // 1. 迴圈性能限制
    uint32_t MAX_LOOP_TIME_MS = 15; 

    // 2. 倒地保護角度 (45度)
    const float CRITICAL_ANGLE_RAD = 60.0f * DEG_TO_RAD; 
    const float RECOVERY_ANGLE_RAD = 5.0f * DEG_TO_RAD;

    uint32_t loop_time_ms = 0;
    uint16_t delay_counter = 0;
    uint32_t last_disarm_time_ms = 0;
    bool ready_auto_start = false;

    set_pending_mode_fn _set_pending_mode_cb;
};
