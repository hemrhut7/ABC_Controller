#include "app_fail_safe.h"

Failsafe::Failsafe(uint16_t period_ms, Processing_Motor* motor, HAL_LED* led) 
    : _motor(motor), _led(led) {
    last_check_time = 0;
    pickup_start_time = 0;
    MAX_LOOP_TIME_MS = period_ms * 2; // 增加容錯空間，避免因系統抖動誤觸發
    is_pickup_condition_met = false;
}

void Failsafe::init() {
    last_check_time = 0;
}

bool Failsafe::check(const ahrs_data_t &ahrs_data, bool ahrs_ready) {
    motor_state_t motor_state;
    _motor->get_motor_state(&motor_state);

    float roll_rad = ahrs_data.euler[1];
    float pitch_rad = ahrs_data.euler[0];
    float pitch_rate_rad = ahrs_data.imu_data.gyro[0];
    int rpm_l = motor_state.rpm_L;
    int rpm_r = motor_state.rpm_R;

    failsafe_error_t current_error_state = FS_ERROR_NONE;
    // 檢查迴圈性能
    uint32_t current_time_ms = millis();
    uint32_t dt = current_time_ms - last_check_time;
    if (last_check_time != 0 && dt > MAX_LOOP_TIME_MS) {
        current_error_state = min(current_error_state, FS_ERROR_LOOP_SLOW);
        delay_counter++;
    }
    last_check_time = current_time_ms;
    loop_time_ms = dt;

    // 檢查 AHRS
    if (!ahrs_ready) {
        current_error_state = min(current_error_state, FS_ERROR_AHRS_UNREADY);
    }

    // 倒地偵測
    if (abs(pitch_rad) > CRITICAL_ANGLE_RAD || abs(roll_rad) > CRITICAL_ANGLE_RAD) {
        current_error_state = min(current_error_state, FS_ERROR_CRITICAL_ANGLE);
    } else if (error_state == FS_ERROR_CRITICAL_ANGLE && abs(pitch_rad) > RECOVERY_ANGLE_RAD) {
        current_error_state = min(current_error_state, FS_ERROR_CRITICAL_ANGLE);
    }

    error_state = current_error_state;
    switch (error_state)
    {
    case FS_ERROR_NONE:
        _led->set_state(SYSTEM_STATE::ARMED);
        if (!ready_auto_start && current_time_ms - last_disarm_time_ms > 3000) 
            ready_auto_start = true;
        break;
    case FS_ERROR_AHRS_UNREADY:
        _led->set_state(SYSTEM_STATE::INITIALIZING);
        last_disarm_time_ms = current_time_ms;
        ready_auto_start = false;
        break;
    default:
        _led->set_state(SYSTEM_STATE::DISARMED);
        last_disarm_time_ms = current_time_ms;
        ready_auto_start = false;
        break;
    }

    return error_state == FS_ERROR_NONE;
}
