#include "app_mode.h"
#include <Arduino.h> // For constrain, abs, etc.


AppMode::AppMode(Processing_Motor* motor, ConfigStore* config_store, int interval_ms) 
    : _motor(motor), _config_store(config_store), 
    lpf_angle(1000 / interval_ms, 50),
    lpf_velocity(1000 / interval_ms, 5), 
    lpf_yaw(1000 / interval_ms, 10),
    lpf_current_velocity(1000 / interval_ms, 10) {
    _cmd.mode = MODE_STOP;
    _cmd.target_value = 0.0f;
    _cmd.target_yaw_rate = 0.0f;
}

void AppMode::init() {
    // 初始化 ConfigStore 並讀取參數
    _config_store->begin();
    _config_store->load_config();

    pitch_queue = xQueueCreate(1, sizeof(float));
    turn_queue = xQueueCreate(1, sizeof(float));
    
    // Level 4: Velocity Loop (外環) - 輸入 m/s, 輸出 Target Pitch (rad)
    _pid_velocity.setTunings(_config_store->data.velocity.p, _config_store->data.velocity.i, _config_store->data.velocity.d);
    _pid_velocity.setOutputLimits(-MAX_PITCH, MAX_PITCH);

    // Level 3: Angle Loop (直立環) - 輸入 Pitch (rad), 輸出 Target Rate (rad/s)
    _pid_angle.setTunings(_config_store->data.pitch.p, _config_store->data.pitch.i, _config_store->data.pitch.d);
    
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

void AppMode::update_internal(float dt, const ahrs_data_t &ahrs_state) {
    int mode = _cmd.mode;
    // 先處理MODE_PWM
    if (mode == MODE_PWM) {
        _motor->set_pwm(_cmd.target_value, _cmd.target_value);
        return;
    }    

    // 1. 獲取狀態 (State Estimation)
    motor_state_t motor_state;
    _motor->get_motor_state(&motor_state);

    current_velocity = (motor_state.rpm_L + motor_state.rpm_R) * 0.5f * RPM_TO_MS; // 需定義轉換係數
    current_velocity = lpf_current_velocity.update(current_velocity); // 50Hz 低通濾波
    float current_pitch = ahrs_state.euler[0]; // Rad
    float current_gyro_x = ahrs_state.imu_data_calibrated.gyro[0]; // Rad/s

    // --- 串級控制邏輯 (The Cascade) ---
    
    // Level 4: Velocity Loop (速度環)
    // 輸入：目標速度 (m/s)，輸出：目標角度 (rad)
    float target_velocity = 0;
    float target_pitch = 0;
    if (mode >= MODE_VELOCITY) {
        target_pitch = constrain(target_pitch, -MAX_PITCH, MAX_PITCH);
    } 
    else if (mode == MODE_ANGLE) {
        target_pitch = constrain(_cmd.target_value * DEG_TO_RAD, -MAX_PITCH, MAX_PITCH);
        target_pitch = lpf_angle.update(target_pitch); // 角度指令的低通濾波
    }

    // Level 3: Angle Loop (直立環)
    // 輸入：目標角度，輸出：目標角速度
    float output_balance;
    if (mode >= MODE_ANGLE) {
        output_balance = _pid_angle.compute(dt, target_pitch, current_pitch, -current_gyro_x);
    } 
    else if (mode == MODE_MOTOR) {
        output_balance = _cmd.target_value;
    }
    else {
        output_balance = 0; // MODE_STOP
    }

    float target_rpm_L = output_balance;
    float target_rpm_R = output_balance;
    // Yaw Loop (獨立的轉向環)
    if (mode == MODE_REMOTE) {
        float output_turn = 0;
        if (xQueueReceive(turn_queue, &output_turn, 0) == pdTRUE) {
            // --- Mixer (混合器) ---
            target_rpm_L -= output_turn;
            target_rpm_R += output_turn;
            if (target_rpm_R > MAX_RPM) {
                target_rpm_R = MAX_RPM;
                target_rpm_L = MAX_RPM - 2 * output_turn;
            } 
            else if (target_rpm_L < -MAX_RPM) {
                target_rpm_L = -MAX_RPM;
                target_rpm_R = -MAX_RPM + 2 * output_turn;
            }
        }
    } 

    // --- Actuation (執行) ---
    if (mode != MODE_STOP) {
        _motor->set_target_rpms(target_rpm_L, target_rpm_R);
    } else {
        target_rpm_L = 0;
        target_rpm_R = 0;
        target_pitch = 0;
        _motor->set_target_rpms(0, 0);
    }
    _pid_target.rpm_L = target_rpm_L;
    _pid_target.rpm_R = target_rpm_R;
    _pid_target.pitch = target_pitch;
}


void AppMode::update_external(float dt, const ahrs_data_t &ahrs_state) {
    int mode = _cmd.mode;
    // 先處理MODE_PWM
    if (mode < MODE_VELOCITY) {
        return;
    }

    // 1. 獲取狀態 (State Estimation)
    motor_state_t motor_state;
    _motor->get_motor_state(&motor_state);

    current_velocity = (motor_state.rpm_L + motor_state.rpm_R) * 0.5f * RPM_TO_MS; // 需定義轉換係數
    // current_velocity = lpf_current_velocity.update(current_velocity); // 50Hz 低通濾波
    float current_pitch = ahrs_state.euler[0]; // Rad
    float current_gyro_x = ahrs_state.imu_data_calibrated.gyro[0]; // Rad/s

    float target_velocity = constrain(_cmd.target_value, -MAX_VELOCITY, MAX_VELOCITY);
    target_velocity = lpf_velocity.update(target_velocity); // 速度指令的低通濾波
    float target_pitch = -_pid_velocity.compute(dt, target_velocity, current_velocity); 
    target_pitch = constrain(target_pitch, -MAX_PITCH, MAX_PITCH);
    if (pitch_queue != NULL) {
        xQueueSend(pitch_queue, &target_pitch, 0);
    }

    // Yaw Loop (獨立的轉向環)
    float output_turn = 0;
    float target_yaw_rate = 0;
    if (mode == MODE_REMOTE) {
        float target_yaw_rate_rads = _cmd.target_yaw_rate * DEG_TO_RAD;
        target_yaw_rate = constrain(target_yaw_rate_rads, -MAX_YAW_RATE, MAX_YAW_RATE);
        target_yaw_rate = lpf_yaw.update(target_yaw_rate); // 轉向指令的低通濾波
        output_turn = _pid_yaw.compute(dt, target_yaw_rate, ahrs_state.imu_data.gyro[2]);
    } 
    output_turn = constrain(output_turn, -MAX_TURN_RPM , MAX_TURN_RPM);
    if (turn_queue != NULL) {
        xQueueSend(turn_queue, &output_turn, 0);
    }

    _pid_target.velocity = target_velocity;
    _pid_target.yaw_rate = target_yaw_rate;
}

void AppMode::set_pid_gains(PID_id_t pid_id, float kp, float ki, float kd) {
    switch (pid_id) {
        case PID_MOTOR:
            _motor->set_pid_gains(kp, ki, kd);
            _config_store->data.motor = {kp, ki, kd};
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