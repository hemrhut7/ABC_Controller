#include "app_mode.h"
#include <Arduino.h> // For constrain, abs, etc.

// 物理參數定義
// 假設輪徑 65mm => 半徑 0.0325m
// 速度 (m/s) = (RPM / 60) * 2 * PI * R
// Factor = 0.0325 * 2 * 3.14159 / 60 ~= 0.003403
#define RPM_TO_MS 0.003403f 

AppMode::AppMode(Processing_AHRS* ahrs, Processing_Motor* motor, ConfigStore* config_store) 
    : _ahrs(ahrs), _motor(motor), _config_store(config_store) {
    _cmd.mode = MODE_STOP;
    _cmd.target_value = 0.0f;
    _cmd.target_yaw_rate = 0.0f;
}

void AppMode::init() {
    // 初始化 ConfigStore 並讀取參數
    _config_store->begin();
    _config_store->load_config();
    
    // Level 4: Velocity Loop (外環) - 輸入 m/s, 輸出 Target Pitch (rad)
    _pid_velocity.setTunings(_config_store->data.velocity.p, _config_store->data.velocity.i, _config_store->data.velocity.d);
    _pid_velocity.setOutputLimits(-0.4f, 0.4f); // 限制最大傾角約 23 度

    // Level 3: Angle Loop (直立環) - 輸入 Pitch (rad), 輸出 Target Rate (rad/s)
    _pid_angle.setTunings(_config_store->data.pitch.p, _config_store->data.pitch.i, _config_store->data.pitch.d);
    
    // Level 2: Rate Loop (角速度環) - 輸入 Rate (rad/s), 輸出 PWM/RPM 增量
    _pid_rate.setTunings(_config_store->data.rate.p, _config_store->data.rate.i, _config_store->data.rate.d);
    
    // Yaw Loop (轉向環)
    _pid_yaw.setTunings(_config_store->data.yaw.p, _config_store->data.yaw.i, _config_store->data.yaw.d);

    // Motor PID
    _motor->set_pid_gains(_config_store->data.motor.p, _config_store->data.motor.i, _config_store->data.motor.d);

    set_target(0.0f, 0.0f);
}

void AppMode::set_command(UserCommand_t cmd) {
    _cmd = cmd;
}

void AppMode::set_mode(Mode_t mode) {
    if (_cmd.mode == mode) return; // 模式相同則不執行

    _cmd.mode = mode;

    _pid_velocity.reset();
    _pid_angle.reset();
    _pid_rate.reset();
    _pid_yaw.reset();

    if (_cmd.mode == MODE_FREE)
        _motor->set_enable(false);
    else
        _motor->set_enable(true);
}

void AppMode::set_target(float val, float yaw) {
    _cmd.target_value = val;
    _cmd.target_yaw_rate = yaw;
}

void AppMode::update(float dt) {
    // 先處理MODE_PWM
    if (_cmd.mode == MODE_PWM) {
        _motor->set_pwm(_cmd.target_value, _cmd.target_value);
        return;
    }    

    // 1. 獲取狀態 (State Estimation)
    ahrs_data_t ahrs_state;
    motor_state_t motor_state;
    _ahrs->get_ahrs_data(&ahrs_state);
    _motor->get_motor_state(&motor_state);

    current_velocity = (motor_state.rpm_L + motor_state.rpm_R) * 0.5f * RPM_TO_MS; // 需定義轉換係數
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
    if (_cmd.mode >= MODE_ANGLE && _cmd.mode != MODE_FREE) {
        // 輸入：目標角度，輸出：目標角速度
        _target_pitch_rate = _pid_angle.compute(dt, _target_pitch, current_pitch);
    } 
    else if (_cmd.mode == MODE_RATE) {
        _target_pitch_rate = _cmd.target_value;
    }

    // Level 2: Rate Loop (角速度/阻尼環)
    if (_cmd.mode >= MODE_RATE && _cmd.mode != MODE_FREE) {
        // 輸入：目標角速度，輸出：馬達 PWM 或 RPM 增量
        _output_balance = _pid_rate.compute(dt, _target_pitch_rate, current_gyro_y);
    }
    else if (_cmd.mode == MODE_MOTOR) {
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

void AppMode::set_pid_gains(PID_id_t pid_id, float kp, float ki, float kd) {
    switch (pid_id) {
        case PID_MOTOR:
            _motor->set_pid_gains(kp, ki, kd);
            _config_store->data.motor = {kp, ki, kd};
            break;
        case PID_RATE: 
            _pid_rate.setTunings(kp, ki, kd); 
            _config_store->data.rate = {kp, ki, kd};
            break;
        case PID_ANGLE: 
            _pid_angle.setTunings(kp, ki, kd); 
            _config_store->data.pitch = {kp, ki, kd};
            break;
        case PID_VELOCITY: 
            _pid_velocity.setTunings(kp, ki, kd); 
            _config_store->data.velocity = {kp, ki, kd};
            break;
        case PID_YAW: 
            _pid_yaw.setTunings(kp, ki, kd); 
            _config_store->data.yaw = {kp, ki, kd};
            break;
        default:
            // Optional: handle invalid ID
            break;
    }
}

PID_Params AppMode::get_pid_gains(PID_id_t pid_id) {
    switch (pid_id) {
        case PID_MOTOR:
            return _config_store->data.motor;
        case PID_RATE:
            return _config_store->data.rate;
        case PID_ANGLE:
            return _config_store->data.pitch;
        case PID_VELOCITY:
            return _config_store->data.velocity;
        case PID_YAW:
            return _config_store->data.yaw;
        default:
            return {0, 0, 0}; // Should not happen
    }
}

const SystemConfig& AppMode::get_pid_config() const {
    return _config_store->data;
}