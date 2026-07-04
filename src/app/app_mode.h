#pragma once

#include <Arduino.h>
#include <freertos/queue.h>
#include "processing/prs_ahrs.h"
#include "processing/prs_motor.h"
#include "pid.h"
#include "LPF.h"
#include "hal/hal_storage.h"
#include "config.h"


class AppMode {
public:
    AppMode(Processing_Motor* motor, ConfigStore* config_store, float interval_ms);
    void init();
    void update(float dt, const ahrs_data_t &ahrs_state);
    void set_mode(Mode_t mode);
    void reset_control_state();
    void set_target(float val, float steer);
    bool enqueue_mode(Mode_t mode);
    bool enqueue_target(float val, float steer);

    Mode_t get_mode() const { return static_cast<Mode_t>(_cmd.mode); }
    Mode_t get_pending_mode() const { return pending_mode; }
    void set_pending_mode(Mode_t mode) { pending_mode = mode; }
    float get_velocity() const { return current_velocity; }
    PID_target_t get_pid_target () const { return _pid_target; }
    UserCommand_t get_user_command() const { return _cmd; }
    
    // 用於 Tuning 的接口
    void set_pid_gains(PID_id_t pid_id, float kp, float ki, float kd);
    void save_pid_gains() { _config_store->save_config(); }
    PID_Params get_pid_gains(PID_id_t pid_id);
    const SystemConfig& get_pid_config() const;

    void set_cut_off_freq(LPF_id_t lpf_id, float cut_off_freq);
    float get_lpf_freq(LPF_id_t lpf_id);

    void set_ramp(PARAM_RAMP_id_t id, float ramp);
    float get_ramp(PARAM_RAMP_id_t id) const;

private:
    enum AppCommandType : uint8_t {
        APP_CMD_SET_MODE = 0,
        APP_CMD_SET_TARGET,
    };

    struct AppCommand {
        AppCommandType type;
        int mode;
        float target_value;
        float target_steer_rate;
    };

    static constexpr uint8_t OUTER_LOOP_DIVIDER = 4;
    static constexpr uint8_t COMMAND_QUEUE_LEN = 16;

    void process_command_queue();
    void check_auto_start_stop(float dt, float current_pitch, float current_gyro_x, float current_rpm);
    void update_mode_selection();

    Processing_Motor* _motor;
    ConfigStore* _config_store;
    QueueHandle_t _cmd_queue;
    
    UserCommand_t _cmd;

    // Parameters (MUST be declared before LPF/PID objects for correct C++ initialization order)
    uint16_t loop_rate_hz = 400;
    float cut_off_freq_velocity = PARM_LPF_CUTOFF_FREQ_VELOCITY;
    float cut_off_freq_steer = PARM_LPF_CUTOFF_FREQ_STEER;
    float cut_off_freq_gyro_z = PARM_LPF_CUTOFF_FREQ_GYRO_Z;
    float cut_off_freq_current_velocity = PARM_LPF_CUTOFF_FREQ_CURRENT_VELOCITY;
    float _pitch_ramp = MAX_PITCH_RAMP;
    float _velocity_ramp = MAX_VELOCITY_RAMP;
    
    // PIDs
    PID _pid_velocity; // 外環
    PID _pid_angle;    // 中環
    PID _pid_steer;      // 轉向

    // LPFs
    LPF_1D lpf_velocity; // 速度的低通濾波器
    LPF_1D lpf_steer; // 轉向的低通濾波器
    LPF_1D lpf_gyro_z;
    LPF_1D lpf_current_velocity; // 當前速度的低通濾波器

    // 中間變數 (便於 Telemetry 觀察)
    PID_target_t _pid_target;
    float current_velocity = 0;
    float output_turn = 0;
    float velocity_loop_dt = 0.0f;
    float steer_loop_dt = 0.0f;

    uint32_t pickup_timer_ms = 0;
    uint32_t drop_timer_ms = 0;
    uint32_t last_disarm_time_ms = 0;

    int32_t wheel_accumulator = 0;  // 累積右輪相對轉動量 (PCNT count)
    Mode_t pending_mode = MODE_FREE;

#if defined(ESP_PLATFORM)
    portMUX_TYPE app_mode_mux = portMUX_INITIALIZER_UNLOCKED;
#endif

    // 靜止狀態與角度超限動態 PID 調節變數
    uint32_t static_timer_ms = 0;
    bool is_static_pid_active = false;
    bool is_angle_boost_active = false;

    // 基準 PID 參數紀錄（用於即時 Tuning，避免因 Flash 未寫入而被覆蓋）
    PID_Params baseline_velocity_pid = {0};
    PID_Params baseline_pitch_pid = {0};

    uint8_t loop_counter = 0;
};
