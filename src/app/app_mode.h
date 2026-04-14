#pragma once

#include <Arduino.h>
#include "processing/prs_ahrs.h"
#include "processing/prs_motor.h"
#include "pid.h"
#include "LPF.h"
#include "hal/hal_storage.h"
#include <freertos/queue.h>

// 物理參數定義
// 假設輪徑 65mm => 半徑 0.0325m
// 速度 (m/s) = (RPM / 60) * 2 * PI * R
// Factor = 0.0325 * 2 * 3.14159 / 60 ~= 0.003403
#define MAX_RPM      150
#define RPM_TO_MS 0.003403f 
#define MAX_PITCH 6.0f * DEG_TO_RAD
#define MAX_PITCH_RAMP 15.0f * DEG_TO_RAD
#define MAX_VELOCITY_RAMP 0.1f
#define MAX_VELOCITY MAX_RPM * 0.65f * RPM_TO_MS
#define MAX_STEER_RPM MAX_RPM * 0.65f

class AppMode {
public:
    AppMode(Processing_Motor* motor, ConfigStore* config_store, int interval_ms);
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
    uint8_t loop_rate_hz = 200;
    float cut_off_freq_velocity = 5.0f;
    float cut_off_freq_steer = 5.0f;
    float cut_off_freq_gyro_z = 10.0f;
    float cut_off_freq_current_velocity = 2.0f;
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

    uint8_t loop_counter = 0;
};
