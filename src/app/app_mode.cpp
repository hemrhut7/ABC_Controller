#include "app_mode.h"
#include <Arduino.h> // For constrain, abs, etc.


AppMode::AppMode(Processing_Motor* motor, ConfigStore* config_store, int interval_ms) 
    : _motor(motor), _config_store(config_store), _cmd_queue(nullptr),
    lpf_angle(1000 / interval_ms, 10),
    lpf_velocity(1000 / interval_ms, 5), 
    lpf_yaw(1000 / interval_ms, 5),
    lpf_current_velocity(1000 / interval_ms, 2),
    lpf_gyro_z(1000 / interval_ms, 20) {
    _cmd.mode = MODE_STOP;
    _cmd.target_value = 0.0f;
    _cmd.target_yaw_rate = 0.0f;
    _cmd_queue = xQueueCreate(COMMAND_QUEUE_LEN, sizeof(AppCommand));
}

void AppMode::init() {
    // 初始化 ConfigStore 並讀取參數
    _config_store->begin();
    _config_store->load_config();
    
    // Level 4: Velocity Loop (外環) - 輸入 m/s, 輸出 Target Pitch (rad)
    _pid_velocity.setTunings(_config_store->data.velocity.p, _config_store->data.velocity.i, _config_store->data.velocity.d);
    _pid_velocity.setOutputLimits(MAX_PITCH);

    // Level 3: Angle Loop (直立環) - 輸入 Pitch (rad), 輸出 RPM
    _pid_angle.setTunings(_config_store->data.pitch.p, _config_store->data.pitch.i, _config_store->data.pitch.d);
    _pid_angle.setOutputLimits(MAX_RPM);
    _pid_angle.setRamp(MAX_PITCH_RATE);

    // Yaw Loop (轉向環)
    _pid_yaw.setTunings(_config_store->data.yaw.p, _config_store->data.yaw.i, _config_store->data.yaw.d);
    _pid_yaw.setOutputLimits(MAX_TURN_RPM);

    // Motor PID
    _motor->set_pid_gains(_config_store->data.motor.p, _config_store->data.motor.i, _config_store->data.motor.d);

    set_target(0.0f, 0.0f);
}

bool AppMode::enqueue_mode(Mode_t mode) {
    if (_cmd_queue == nullptr) return false;
    AppCommand cmd = {APP_CMD_SET_MODE, static_cast<int>(mode), 0.0f, 0.0f};
    return xQueueSend(_cmd_queue, &cmd, 0) == pdTRUE;
}

bool AppMode::enqueue_target(float val, float yaw) {
    if (_cmd_queue == nullptr) return false;
    AppCommand cmd = {APP_CMD_SET_TARGET, 0, val, yaw};
    return xQueueSend(_cmd_queue, &cmd, 0) == pdTRUE;
}

void AppMode::process_command_queue() {
    if (_cmd_queue == nullptr) return;

    AppCommand cmd;
    while (xQueueReceive(_cmd_queue, &cmd, 0) == pdTRUE) {
        if (cmd.type == APP_CMD_SET_MODE) {
            set_mode(static_cast<Mode_t>(cmd.mode));
        } else if (cmd.type == APP_CMD_SET_TARGET) {
            set_target(cmd.target_value, cmd.target_yaw_rate);
        }
    }
}

void AppMode::set_mode(Mode_t mode) {
    if (_cmd.mode == mode) return; // 模式相同則不執行

    _cmd.mode = mode;
    
    if (_cmd.mode == MODE_FREE)
        _motor->set_enable(false);
    else
        _motor->set_enable(true);

    reset_control_state();
}

void AppMode::reset_control_state() {
    output_turn = 0;
    loop_counter = 0;
    velocity_loop_dt = 0.0f;
    yaw_loop_dt = 0.0f;
    _pid_target = {0};
    _cmd.target_value = 0.0f;
    _cmd.target_yaw_rate = 0.0f;

    _pid_velocity.reset();
    _pid_angle.reset();
    _pid_yaw.reset();
    lpf_angle.reset();
    lpf_velocity.reset();
    lpf_yaw.reset();
    lpf_gyro_z.reset();
    lpf_current_velocity.reset();
    _motor->reset();
}

void AppMode::set_target(float val, float yaw) {
    _cmd.target_value = val;
    _cmd.target_yaw_rate = yaw;
}

void AppMode::update(float dt, const ahrs_data_t &ahrs_state) {
    process_command_queue();

    // 先處理MODE_PWM
    if (_cmd.mode == MODE_PWM) {
        _motor->set_pwm(_cmd.target_value, _cmd.target_value);
        return;
    }

    // 1. 獲取狀態 (State Estimation)
    motor_state_t motor_state;
    _motor->get_motor_state(&motor_state);

    current_velocity = (motor_state.rpm_L + motor_state.rpm_R) * 0.5f * RPM_TO_MS; // 需定義轉換係數
    current_velocity = lpf_current_velocity.update(current_velocity); // 10Hz 低通濾波
    float current_pitch = ahrs_state.euler[0]; // Rad
    float current_gyro_x = ahrs_state.imu_data_calibrated.gyro[0]; // Rad/s
    velocity_loop_dt += dt;
    yaw_loop_dt += dt;
    const bool run_outer_loop = (loop_counter % OUTER_LOOP_DIVIDER) == 0;

    // --- 串級控制邏輯 (The Cascade) ---
    
    // Level 4: Velocity Loop (速度環)
    // 輸入：目標速度 (m/s)，輸出：目標角度 (rad)
    float target_velocity = _pid_target.velocity;
    float target_pitch = _pid_target.pitch;
    if (run_outer_loop) {
        if (_cmd.mode >= MODE_VELOCITY) {
            target_velocity = constrain(_cmd.target_value, -MAX_VELOCITY, MAX_VELOCITY);
            target_velocity = lpf_velocity.update(target_velocity);
            target_pitch = -_pid_velocity.compute(velocity_loop_dt, target_velocity, current_velocity); 
        } 
        else if (_cmd.mode == MODE_ANGLE) {
            target_pitch = constrain(_cmd.target_value * DEG_TO_RAD, -MAX_PITCH, MAX_PITCH);
        }
        velocity_loop_dt = 0.0f;
    }

    // Level 3: Angle Loop (直立環)
    // 輸入：目標角度，輸出：目標角速度
    float output_balance;
    if (_cmd.mode >= MODE_ANGLE) {
        output_balance = _pid_angle.compute(dt, target_pitch, current_pitch, -current_gyro_x);
    } 
    else if (_cmd.mode == MODE_MOTOR) {
        output_balance = _cmd.target_value;
    }
    else {
        output_balance = 0; // MODE_STOP
    }

    // Yaw Loop (獨立的轉向環)
    float target_yaw_rate = _pid_target.yaw_rate;
    if (run_outer_loop) {
        if (_cmd.mode == MODE_REMOTE) {
            float target_yaw_rate_rads = _cmd.target_yaw_rate * DEG_TO_RAD;
            target_yaw_rate = constrain(target_yaw_rate_rads, -MAX_YAW_RATE, MAX_YAW_RATE);
            float filtered_gyro_z = lpf_gyro_z.update(ahrs_state.imu_data_calibrated.gyro[2]);
            output_turn = _pid_yaw.compute(yaw_loop_dt, target_yaw_rate, filtered_gyro_z);
        } else {
            output_turn = 0;
        }
        output_turn = lpf_yaw.update(output_turn);
        yaw_loop_dt = 0.0f;
    }
    

    // --- Mixer (混合器) ---
    // 平衡輸出加在兩輪同向，轉向輸出加在兩輪反向
    // 注意：這裡假設 output 直接對應 RPM，如果 Processing_Motor 吃的是 PWM，這裡單位要注意
    float target_rpm_L = output_balance - output_turn;
    float target_rpm_R = output_balance + output_turn;
        
    if (target_rpm_R > MAX_RPM) {
        target_rpm_R = MAX_RPM;
        target_rpm_L = MAX_RPM - 2 * output_turn;
    } else if (target_rpm_L > MAX_RPM) {
        target_rpm_L = MAX_RPM;
        target_rpm_R = MAX_RPM + 2 * output_turn;
    } else if (target_rpm_L < -MAX_RPM) {
        target_rpm_L = -MAX_RPM;
        target_rpm_R = -MAX_RPM + 2 * output_turn;
    } else if (target_rpm_R < -MAX_RPM) {
        target_rpm_R = -MAX_RPM;
        target_rpm_L = -MAX_RPM - 2 * output_turn;
    }
    

    // --- Actuation (執行) ---
    if (_cmd.mode != MODE_STOP) {
        _motor->set_target_rpms(target_rpm_L, target_rpm_R);
    } else {
        target_rpm_L = 0;
        target_rpm_R = 0;
        target_pitch = 0;
        target_velocity = 0;
        target_yaw_rate = 0;
        _motor->set_target_rpms(0, 0);
    }
    _pid_target = {target_rpm_L, target_rpm_R, target_pitch, target_velocity, target_yaw_rate};
    loop_counter++;
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
