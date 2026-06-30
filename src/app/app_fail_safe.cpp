#include "app_fail_safe.h"

Failsafe::Failsafe(float period_ms, Processing_Motor* motor, set_pending_mode_fn set_pending_mode_cb) 
    : _motor(motor), _set_pending_mode_cb(set_pending_mode_cb) {
    last_check_time_us = 0;
    consecutive_slow_frames = 0;
    pickup_start_time = 0;
    MAX_LOOP_TIME_MS = period_ms * 2.0f; // 保持浮點數精準度 (2.5ms * 2.0 = 5.0ms)
    is_pickup_condition_met = false;
}

void Failsafe::init() {
    last_check_time_us = 0;
    consecutive_slow_frames = 0;
}

bool Failsafe::check(const ahrs_data_t &ahrs_data, AHRS_STATE ahrs_state, Mode_t current_mode) {
    motor_state_t motor_state;
    _motor->get_motor_state(&motor_state);

    float roll_rad = ahrs_data.euler[1];
    float pitch_rad = ahrs_data.euler[0];
    float pitch_rate_rad = ahrs_data.imu_data.gyro[0];
    float rpm_l = motor_state.rpm_L;
    float rpm_r = motor_state.rpm_R;

    failsafe_error_t current_error_state = FS_ERROR_NONE;
    const bool is_manual_mode = (current_mode == MODE_PWM || current_mode == MODE_MOTOR || current_mode == MODE_FREE);

    // 檢查迴圈性能 (微秒級高精度 + 連續超時判定)
    uint32_t current_time_us = micros();
    uint32_t current_time_ms = current_time_us / 1000;
    float dt_ms = 0.0f;
    if (last_check_time_us == 0) {
        dt_ms = 2.5f; // 第一幀給予名義值，防止遙測突波
    } else {
        dt_ms = (current_time_us - last_check_time_us) * 0.001f;
    }
    last_check_time_us = current_time_us;
    loop_time_ms = (uint32_t)dt_ms;

    if (last_check_time_us != 0 && dt_ms > MAX_LOOP_TIME_MS) {
        consecutive_slow_frames++;
        delay_counter++; // 統計累計抖動次數 (保留原本每幀超時皆累加的邏輯)
        if (consecutive_slow_frames >= MAX_CONSECUTIVE_SLOW_FRAMES) {
            current_error_state = min(current_error_state, FS_ERROR_LOOP_SLOW);
            if (_set_pending_mode_cb && !is_manual_mode) _set_pending_mode_cb(MODE_FREE);
        }
    } else {
        consecutive_slow_frames = 0; // 只要有一次正常幀，連續超時計數立刻重置
    }

    // 檢查 AHRS
    if (ahrs_state == IMU_FAILED) {
        current_error_state = min(current_error_state, FS_ERROR_IMU_FAILED);
    } else if (ahrs_state == AHRS_INITIALIZING) {
        current_error_state = min(current_error_state, FS_ERROR_AHRS_UNREADY);
    }

    // 倒地偵測
    if (fabsf(pitch_rad) > CRITICAL_ANGLE_RAD || fabsf(roll_rad) > CRITICAL_ANGLE_RAD) {
        current_error_state = min(current_error_state, FS_ERROR_CRITICAL_ANGLE);
        if (_set_pending_mode_cb && !is_manual_mode) _set_pending_mode_cb(MODE_FREE);
    } else if (error_state == FS_ERROR_CRITICAL_ANGLE && fabsf(pitch_rad) > RECOVERY_ANGLE_RAD) {
        current_error_state = min(current_error_state, FS_ERROR_CRITICAL_ANGLE);
    }

    error_state = current_error_state;
    switch (error_state)
    {
    case FS_ERROR_NONE:
        if (!ready_auto_start && current_time_ms - last_disarm_time_ms > 3000) 
            ready_auto_start = true;
        break;
    case FS_ERROR_AHRS_UNREADY:
        last_disarm_time_ms = current_time_ms;
        ready_auto_start = false;
        break;
    default:
        last_disarm_time_ms = current_time_ms;
        ready_auto_start = false;
        break;
    }

    if (is_manual_mode) return true; // Manual modes bypass disarm return value
    return error_state == FS_ERROR_NONE;
}
