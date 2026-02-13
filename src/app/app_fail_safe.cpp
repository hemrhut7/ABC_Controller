#include "app_fail_safe.h"

Failsafe::Failsafe() {
    armed_state = false;
    last_check_time = 0;
    pickup_start_time = 0;
    is_pickup_condition_met = false;
}

void Failsafe::init() {
    armed_state = false;
    last_check_time = millis();
}

bool Failsafe::check(float pitch_rad, float pitch_rate_rad, int rpm_l, int rpm_r, uint32_t current_time_ms) {
    
    // --- [1. 性能檢驗 Loop Performance] ---
    uint32_t dt = current_time_ms - last_check_time;
    
    // 忽略第一次執行 (dt 會很大)
    if (last_check_time != 0 && dt > MAX_LOOP_TIME_MS) {
        // 選項：您可以選擇只印出警告，或是嚴格一點直接 Disarm
        // 這裡我們先做嚴格檢查：系統太慢就斷電
        armed_state = false; 
        // Serial.printf("Loop too slow: %d ms\n", dt);
    }
    last_check_time = current_time_ms;
    loop_time_ms = dt;


    // --- [2. 拿起偵測 Pickup Detection] ---
    // 邏輯：兩個輪子轉速都很快，但機身卻幾乎不動 (Gyro 很小)
    bool high_rpm = (abs(rpm_l) > PICKUP_RPM_THRESHOLD) && (abs(rpm_r) > PICKUP_RPM_THRESHOLD);
    bool low_motion = abs(pitch_rate_rad) < PICKUP_GYRO_THRESHOLD;

    if (high_rpm && low_motion) {
        if (!is_pickup_condition_met) {
            // 剛開始偵測到異常，記錄時間
            pickup_start_time = current_time_ms;
            is_pickup_condition_met = true;
        } else {
            // 持續偵測中，檢查時間是否超過閾值
            if (current_time_ms - pickup_start_time > PICKUP_CONFIRM_MS) {
                armed_state = false; // 確認被拿起，切斷動力
            }
        }
    } else {
        // 條件消失，重置計時器
        is_pickup_condition_met = false;
    }

    // --- [3. 狀態管理與倒地保護] ---
    
    // 如果已經 Disarmed (倒地或被拿起)
    if (!armed_state) {
        // 嘗試恢復 (Arming)：只有在「非」拿起狀態 且 角度很正 時才恢復
        // 增加一個保護：被拿起時絕對不能恢復，不然放回地上瞬間會暴衝
        bool is_safe_angle = abs(pitch_rad) < RECOVERY_ANGLE_RAD;
        
        if (is_safe_angle && !is_pickup_condition_met) {
            armed_state = true;
        } else {
            return false; // 繼續保持鎖定
        }
    }

    // 檢查倒地 (基本保護)
    if (abs(pitch_rad) > CRITICAL_ANGLE_RAD) {
        armed_state = false;
        return false;
    }

    return true; // ARMED & Safe
}


bool Failsafe::is_armed() {
    return armed_state;
}
