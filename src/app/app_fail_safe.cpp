#include "app_fail_safe.h"

Failsafe::Failsafe(float period_ms, Processing_Motor* motor, set_pending_mode_fn set_pending_mode_cb) 
    : _motor(motor), _set_pending_mode_cb(set_pending_mode_cb) {
    last_check_time = 0;
    pickup_start_time = 0;
    MAX_LOOP_TIME_MS = (uint32_t)(period_ms * 2.0f); // 增加容錯空間，避免因系統抖動誤觸發
    is_pickup_condition_met = false;
}

void Failsafe::init() {
    last_check_time = 0;
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

    // 檢查迴圈性能
    uint32_t current_time_ms = millis();
    uint32_t dt = current_time_ms - last_check_time;
    if (last_check_time != 0 && dt > MAX_LOOP_TIME_MS) {
        current_error_state = min(current_error_state, FS_ERROR_LOOP_SLOW);
        if (_set_pending_mode_cb && !is_manual_mode) _set_pending_mode_cb(MODE_FREE);
        delay_counter++;
    }
    last_check_time = current_time_ms;
    loop_time_ms = dt;

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
