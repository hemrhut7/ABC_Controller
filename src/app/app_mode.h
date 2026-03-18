#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "processing/prs_ahrs.h"
#include "processing/prs_motor.h"
#include "pid.h"
#include "LPF.h"
#include "hal/hal_storage.h"

// 物理參數定義
// 假設輪徑 65mm => 半徑 0.0325m
// 速度 (m/s) = (RPM / 60) * 2 * PI * R
// Factor = 0.0325 * 2 * 3.14159 / 60 ~= 0.003403
#define MAX_RPM      150
#define RPM_TO_MS 0.003403f 
#define MAX_PITCH 0.25f // rad, 約 14.3 deg
#define MAX_VELOCITY 0.5f // m/s, equal to ~150 RPM for 65mm wheel
#define MAX_YAW_RATE 10.0f / 180.0f * 3.1416f // rad/s, 約 10 deg/s 
#define MAX_TURN_RPM (MAX_RPM*0.25f)


class AppMode {
public:
    AppMode(Processing_Motor* motor, ConfigStore* config_store, int interval_ms);
    void init();
    void update_internal(float dt, const ahrs_data_t &ahrs_state);
    void update_external(float dt, const ahrs_data_t &ahrs_state);
    void set_command(UserCommand_t cmd);
    void set_mode(Mode_t mode);
    void set_target(float val, float yaw);
    float get_target_val() const { return _cmd.target_value; }
    Mode_t get_mode() const { return static_cast<Mode_t>(_cmd.mode); }
    float get_velocity() const { return current_velocity; }
    PID_target_t get_pid_target () const { return _pid_target; }
    
    // 用於 Tuning 的接口
    void set_pid_gains(PID_id_t pid_id, float kp, float ki, float kd);
    void save_pid_gains() { _config_store->save_config(); }
    PID_Params get_pid_gains(PID_id_t pid_id);
    const SystemConfig& get_pid_config() const;

private:
    Processing_Motor* _motor;
    ConfigStore* _config_store;
    
    UserCommand_t _cmd;
    
    // PIDs
    PID _pid_velocity; // 外環
    PID _pid_angle;    // 中環
    PID _pid_yaw;      // 轉向

    LPF_1D lpf_angle; // 傾角的低通濾波器
    LPF_1D lpf_velocity; // 速度的低通濾波器
    LPF_1D lpf_yaw; // 轉向的低通濾波器
    LPF_1D lpf_current_velocity; // 當前速度的低通濾波器

    QueueHandle_t pitch_queue;
    QueueHandle_t turn_queue;
    
    // 中間變數 (便於 Telemetry 觀察)
    PID_target_t _pid_target;
    float current_velocity;
};