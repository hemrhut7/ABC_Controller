#include "app_mode.h"
#include <Arduino.h> // For constrain, abs, etc.

// 物理參數定義
// 假設輪徑 65mm => 半徑 0.0325m
// 速度 (m/s) = (RPM / 60) * 2 * PI * R
// Factor = 0.0325 * 2 * 3.14159 / 60 ~= 0.003403
#define RPM_TO_MS 0.003403f 

AppMode::AppMode(Processing_AHRS* ahrs, Processing_Motor* motor) 
    : _ahrs(ahrs), _motor(motor) {
    _cmd.mode = MODE_STOP;
    _cmd.target_value = 0.0f;
    _cmd.target_yaw_rate = 0.0f;
}

void AppMode::init() {
    // 初始化 PID 參數 (需根據實際機體調整)
    
    // Level 4: Velocity Loop (外環) - 輸入 m/s, 輸出 Target Pitch (rad)
    _pid_velocity.setTunings(0.2f, 0.01f, 0.0f);
    _pid_velocity.setOutputLimits(-0.4f, 0.4f); // 限制最大傾角約 23 度

    // Level 3: Angle Loop (直立環) - 輸入 Pitch (rad), 輸出 Target Rate (rad/s)
    _pid_angle.setTunings(4.5f, 0.0f, 0.1f);
    
    // Level 2: Rate Loop (角速度環) - 輸入 Rate (rad/s), 輸出 PWM/RPM 增量
    _pid_rate.setTunings(1.2f, 15.0f, 0.01f);
    
    // Yaw Loop (轉向環)
    _pid_yaw.setTunings(1.0f, 0.0f, 0.0f);
}

void AppMode::set_command(UserCommand_t cmd) {
    _cmd = cmd;
}

void AppMode::set_mode(Mode_t mode) {
    _cmd.mode = mode;
    // 切換模式時重置 PID 積分項是個好習慣，視需求添加
    // _pid_velocity.reset();
}

void AppMode::set_target_val(float val) {
    _cmd.target_value = val;
}

void AppMode::update(float dt) {
    // 1. 獲取狀態 (State Estimation)
    ahrs_data_t ahrs_state;
    motor_state_t motor_state;
    _ahrs->get_ahrs_data(&ahrs_state);
    _motor->get_motor_state(&motor_state);

    float current_velocity = (motor_state.rpm_L + motor_state.rpm_R) * 0.5f * RPM_TO_MS; // 需定義轉換係數
    float current_pitch = ahrs_state.euler[0]; // Rad
    float current_gyro_y = ahrs_state.imu_data.gyro[1]; // Rad/s

    // --- 串級控制邏輯 (The Cascade) ---
    
    // Level 4: Velocity Loop (速度環)
    if (_cmd.mode == MODE_VELOCITY || _cmd.mode == MODE_REMOTE) {
        // 輸入：目標速度 (m/s)，輸出：目標角度 (rad)
        _target_pitch = _pid_velocity.compute(dt, _cmd.target_value, current_velocity);
        
        // 安全限幅：物理上不可能傾斜超過 45 度還能救回來
        _target_pitch = constrain(_target_pitch, -0.5f, 0.5f); 
    } 
    else if (_cmd.mode == MODE_ANGLE) {
        _target_pitch = _cmd.target_value;
    }

    // Level 3: Angle Loop (直立環)
    if (_cmd.mode >= MODE_ANGLE) {
        // 輸入：目標角度，輸出：目標角速度
        _target_pitch_rate = _pid_angle.compute(dt, _target_pitch, current_pitch);
    } 
    else if (_cmd.mode == MODE_RATE) {
        _target_pitch_rate = _cmd.target_value;
    }

    // Level 2: Rate Loop (角速度/阻尼環)
    if (_cmd.mode >= MODE_RATE) {
        // 輸入：目標角速度，輸出：馬達 PWM 或 RPM 增量
        _output_balance = _pid_rate.compute(dt, _target_pitch_rate, current_gyro_y);
    }
    else if (_cmd.mode == MODE_MOTOR_TEST) {
        _output_balance = _cmd.target_value;
    }
    else {
        _output_balance = 0; // MODE_STOP
    }

    // Yaw Loop (獨立的轉向環)
    if (_cmd.mode == MODE_REMOTE || _cmd.mode == MODE_VELOCITY) {
        float current_yaw_rate = ahrs_state.imu_data.gyro[2];
        _output_turn = _pid_yaw.compute(dt, _cmd.target_yaw_rate, current_yaw_rate);
    } else {
        _output_turn = 0;
    }

    // --- Mixer (混合器) ---
    // 平衡輸出加在兩輪同向，轉向輸出加在兩輪反向
    // 注意：這裡假設 output 直接對應 RPM，如果 Processing_Motor 吃的是 PWM，這裡單位要注意
    float target_rpm_L = _output_balance + _output_turn;
    float target_rpm_R = _output_balance - _output_turn;

    // --- Actuation (執行) ---
    if (_cmd.mode != MODE_STOP) {
        _motor->set_target_rpms(target_rpm_L, target_rpm_R);
    } else {
        _motor->set_target_rpms(0, 0);
    }
}

void AppMode::set_pid_gains(uint8_t pid_id, float kp, float ki, float kd) {
    switch (pid_id) {
        case 0: _pid_velocity.setTunings(kp, ki, kd); break;
        case 1: _pid_angle.setTunings(kp, ki, kd); break;
        case 2: _pid_rate.setTunings(kp, ki, kd); break;
        case 3: _pid_yaw.setTunings(kp, ki, kd); break;
    }
}