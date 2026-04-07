#include "app_mode.h"
#include <Arduino.h> // For constrain, abs, etc.


AppMode::AppMode(Processing_Motor* motor, ConfigStore* config_store, int interval_ms) 
    : _motor(motor), _config_store(config_store), _cmd_queue(nullptr),
    lpf_angle(1000 / interval_ms, 10),
    lpf_velocity(1000 / interval_ms, 5), 
    lpf_steer(1000 / interval_ms, 5),
    lpf_current_velocity(1000 / interval_ms, 2),
    lpf_gyro_z(1000 / interval_ms, 20) {
    _cmd.mode = MODE_STOP;
    _cmd.target_value = 0.0f;
    _cmd.target_steer = 0.0f;
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

    // Steer Loop (轉向環)
    _pid_steer.setTunings(_config_store->data.steer.p, _config_store->data.steer.i, _config_store->data.steer.d);
    _pid_steer.setOutputLimits(MAX_STEER_RPM);

    // Motor PID
    _motor->set_pid_gains(_config_store->data.motor.p, _config_store->data.motor.i, _config_store->data.motor.d);

    set_target(0.0f, 0.0f);
}

bool AppMode::enqueue_mode(Mode_t mode) {
    if (_cmd_queue == nullptr) return false;
    AppCommand cmd = {APP_CMD_SET_MODE, static_cast<int>(mode), 0.0f, 0.0f};
    return xQueueSend(_cmd_queue, &cmd, 0) == pdTRUE;
}

bool AppMode::enqueue_target(float val, float steer) {
    if (_cmd_queue == nullptr) return false;
    AppCommand cmd = {APP_CMD_SET_TARGET, 0, val, steer};
    return xQueueSend(_cmd_queue, &cmd, 0) == pdTRUE;
}

void AppMode::process_command_queue() {
    if (_cmd_queue == nullptr) return;

    AppCommand cmd;
    while (xQueueReceive(_cmd_queue, &cmd, 0) == pdTRUE) {
        if (cmd.type == APP_CMD_SET_MODE) {
            set_mode(static_cast<Mode_t>(cmd.mode));
        } else if (cmd.type == APP_CMD_SET_TARGET) {
            set_target(cmd.target_value, cmd.target_steer_rate);
        }
    }
}

void AppMode::set_mode(Mode_t mode) {
    if (_cmd.mode == mode) return;

    _cmd.mode = mode;
    pending_mode = mode;
    
    if (_cmd.mode == MODE_FREE) {
        _motor->set_enable(false);
        wheel_accumulator = 0;  // 進入 FREE 時歸零累積器
    } else {
        _motor->set_enable(true);
    }

    reset_control_state();
}

void AppMode::reset_control_state() {
    output_turn = 0;
    loop_counter = 0;
    velocity_loop_dt = 0.0f;
    steer_loop_dt = 0.0f;
    _pid_target = {0};
    _cmd.target_value = 0.0f;
    _cmd.target_steer = 0.0f;

    _pid_velocity.reset();
    _pid_angle.reset();
    _pid_steer.reset();
    lpf_angle.reset();
    lpf_velocity.reset();
    lpf_steer.reset();
    lpf_gyro_z.reset();
    lpf_current_velocity.reset();
    _motor->reset();
}

void AppMode::set_target(float val, float steer) {
    _cmd.target_value = val;
    _cmd.target_steer = steer;
}

void AppMode::update(float dt, const ahrs_data_t &ahrs_state) {
    process_command_queue();

    // 1. 獲取狀態 (State Estimation)
    motor_state_t motor_state;
    _motor->get_motor_state(&motor_state);

    float current_rpm = (motor_state.rpm_L + motor_state.rpm_R) * 0.5f;
    float current_pitch = ahrs_state.euler[0]; // Rad
    float current_gyro_x = ahrs_state.imu_data_calibrated.gyro[0]; // Rad/s

    // 檢測自動啟停 (Issue #10)
    check_auto_start_stop(dt, current_pitch, current_gyro_x, current_rpm);

    // MODE_FREE 下的輪胎切換模式邏輯
    uint32_t now_ms = ahrs_state.imu_data.timestamp * 1e-3f;
    if (_cmd.mode == MODE_FREE) {
        if ((now_ms - last_disarm_time_ms) > 3000) {
            update_mode_selection();
        }
    } else {
        last_disarm_time_ms = now_ms;
    }

    // 先處理MODE_PWM
    if (_cmd.mode == MODE_PWM) {
        _motor->set_pwm(_cmd.target_value, _cmd.target_value);
        return;
    }

    // 1. 獲取狀態 (State Estimation)
    current_velocity = current_rpm * RPM_TO_MS; // 需定義轉換係數
    current_velocity = lpf_current_velocity.update(current_velocity); // 10Hz 低通濾波
    velocity_loop_dt += dt;
    steer_loop_dt += dt;
    const bool run_outer_loop = (loop_counter % OUTER_LOOP_DIVIDER) == 0;

    // --- 串級控制邏輯 (The Cascade) ---
    float cmd_val = _cmd.target_value;
    float cmd_steer = _cmd.target_steer;
    
    // Level 4: Velocity Loop (速度環)
    // 輸入：目標速度 (m/s)，輸出：目標角度 (rad)
    float target_velocity = _pid_target.velocity;
    float target_pitch = _pid_target.pitch;
    float current_rpm_diff = motor_state.rpm_L - motor_state.rpm_R;
    float ratio_val;

    if (run_outer_loop) {
        if (_cmd.mode >= MODE_VELOCITY) {
            cmd_val = constrain(cmd_val, -MAX_VELOCITY, MAX_VELOCITY);
            cmd_steer = constrain(cmd_steer, -1, 1);
            ratio_val = abs(cmd_val / MAX_VELOCITY);
            float ratio_steer = abs(cmd_steer);
            float length_2 = ratio_val * ratio_val + ratio_steer * ratio_steer;
            if (length_2 > 1) {
                float length = sqrt(length_2);
                cmd_val /= length;
                cmd_steer /= length;
            }
            if (_cmd.mode < MODE_TURBO) cmd_val *= 0.9f;

            target_velocity = lpf_velocity.update(cmd_val);
            target_pitch = -_pid_velocity.compute(velocity_loop_dt, target_velocity, current_velocity); 
        } 
        else if (_cmd.mode == MODE_ANGLE) {
            target_pitch = constrain(cmd_val * DEG_TO_RAD, -MAX_PITCH, MAX_PITCH);
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
        output_balance = cmd_val;
    }
    else {
        output_balance = 0; // MODE_STOP
    }

    // Yaw Loop (獨立的轉向環)
    float target_steer_rpm = _pid_target.steer_rpm;
    if (run_outer_loop) {
        if (_cmd.mode == MODE_REMOTE) {
            target_steer_rpm = cmd_steer * abs(cmd_steer) *  MAX_STEER_RPM;
            if (_cmd.mode < MODE_TURBO) target_steer_rpm *= 0.9f;
            output_turn = _pid_steer.compute(steer_loop_dt, target_steer_rpm, current_rpm_diff);
        } else {
            output_turn = 0;
        }
        output_turn = lpf_steer.update(output_turn);
        steer_loop_dt = 0.0f;
    }
    

    // --- Mixer (混合器) ---
    // 平衡輸出加在兩輪同向，轉向輸出加在兩輪反向
    // 注意：這裡假設 output 直接對應 RPM，如果 Processing_Motor 吃的是 PWM，這裡單位要注意
    float target_rpm_L = output_balance + output_turn;
    float target_rpm_R = output_balance - output_turn;
        
    if (target_rpm_L > MAX_RPM) {
        target_rpm_L = MAX_RPM;
        target_rpm_R = MAX_RPM - 2 * output_turn;
    } else if (target_rpm_R > MAX_RPM) {
        target_rpm_R = MAX_RPM;
        target_rpm_L = MAX_RPM + 2 * output_turn;
    } else if (target_rpm_L < -MAX_RPM) {
        target_rpm_L = -MAX_RPM;
        target_rpm_R = -MAX_RPM - 2 * output_turn;
    } else if (target_rpm_R < -MAX_RPM) {
        target_rpm_R = -MAX_RPM;
        target_rpm_L = -MAX_RPM + 2 * output_turn;
    }
    

    // --- Actuation (執行) ---
    if (_cmd.mode != MODE_STOP) {
        _motor->set_target_rpms(target_rpm_L, target_rpm_R);
    } else {
        target_rpm_L = 0;
        target_rpm_R = 0;
        target_pitch = 0;
        target_velocity = 0;
        target_steer_rpm = 0;
        _motor->set_target_rpms(0, 0);
    }
    _pid_target = {target_rpm_L, target_rpm_R, target_pitch, target_velocity, target_steer_rpm};
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
        case PID_STEER: 
            _pid_steer.setTunings(kp, ki, kd); 
            _config_store->data.steer = {kp, ki, kd};
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
        case PID_STEER:
            return _config_store->data.steer;
        default:
            return {0, 0, 0}; // Should not happen
    }
}

const SystemConfig& AppMode::get_pid_config() const {
    return _config_store->data;
}

void AppMode::check_auto_start_stop(float dt, float current_pitch, float current_gyro_x, float current_rpm) {
    // PWM / MOTOR 為手動測試模式，跳過自動啟停
    if (_cmd.mode == MODE_PWM || _cmd.mode == MODE_MOTOR) {
        pickup_timer_ms = 0;
        drop_timer_ms = 0;
        return;
    }

    // --- 1. 懸空保護 (Pick-up Detection) ---
    // 條件：平衡中 + 姿態接近直立 + 輪胎高速空轉 → 判定為被撿起
    if (_cmd.mode >= MODE_ANGLE) {
        bool is_runaway = (abs(current_pitch) < 15.0f * DEG_TO_RAD)
                       && (abs(current_rpm) > MAX_RPM * 0.8f);

        pickup_timer_ms = is_runaway ? pickup_timer_ms + (uint32_t)(dt * 1000.0f) : 0;

        if (pickup_timer_ms > 500) {
            enqueue_mode(MODE_FREE);
            pickup_timer_ms = 0;
        }
    } else {
        pickup_timer_ms = 0;
    }

    // --- 2. 落地啟動 (Drop-to-Start) ---
    // 條件：停止中 + 直立 + 穩定 + 輪胎靜止 → 自動恢復到 pending_mode
    if (_cmd.mode == MODE_FREE) {
        bool is_ready = (abs(current_pitch)  < 3.0f  * DEG_TO_RAD)
                     && (abs(current_gyro_x) < 10.0f * DEG_TO_RAD)
                     && (abs(current_rpm)    < 10.0f);

        drop_timer_ms = is_ready ? drop_timer_ms + (uint32_t)(dt * 1000.0f) : 0;

        if (drop_timer_ms > 500 && pending_mode >= MODE_ANGLE) {
            reset_control_state();
            enqueue_mode(pending_mode);
            drop_timer_ms = 0;
        }
    } else {
        drop_timer_ms = 0;
    }
}

void AppMode::update_mode_selection() {
    // get_right_wheel_count() 回傳的是自上次讀取以來的「相對轉動量」
    // (PCNT 在 getPCNTCount() 中會被清零)
    // 因此需要累積到 wheel_accumulator 來追蹤絕對位置
    int16_t delta_count = _motor->get_right_wheel_count();
    wheel_accumulator += delta_count;

    // 每 30 度切換一個模式
    const int32_t cnt_per_step = (int32_t)(60.0f / DEG_PER_CNT);

    if (abs(wheel_accumulator) >= cnt_per_step) {
        int steps = wheel_accumulator / cnt_per_step;
        wheel_accumulator -= steps * cnt_per_step;  // 保留餘量

        // 確保起始 pending_mode 在合法循環範圍內
        if (pending_mode < MODE_ANGLE || pending_mode > MODE_TURBO) {
            pending_mode = MODE_ANGLE;
        }

        int next_mode = (int)pending_mode + steps;

        // 循環限制: MODE_ANGLE(4) ~ MODE_TURBO(7)
        if (next_mode > MODE_TURBO) next_mode = MODE_ANGLE;
        if (next_mode < MODE_ANGLE) next_mode = MODE_TURBO;

        pending_mode = (Mode_t)next_mode;
    }
}
