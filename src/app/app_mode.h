#pragma once

#include "processing/prs_ahrs.h"
#include "processing/prs_motor.h"
#include "pid.h"
#include "app_data_types.h"
#include "hal/hal_storage.h"

class AppMode {
public:
    AppMode(Processing_AHRS* ahrs, Processing_Motor* motor, ConfigStore* config_store);
    void init();
    void update(float dt); // 必須傳入固定 dt
    void set_command(UserCommand_t cmd);
    void set_mode(Mode_t mode);
    void set_target_val(float val);
    void set_target(float val, float yaw);
    
    // 用於 Tuning 的接口
    void set_pid_gains(uint8_t pid_id, float kp, float ki, float kd);

private:
    Processing_AHRS* _ahrs;
    Processing_Motor* _motor;
    ConfigStore* _config_store;
    
    UserCommand_t _cmd;
    
    // PIDs
    PID _pid_velocity; // 外環
    PID _pid_angle;    // 中環
    PID _pid_rate;     // 內環 (阻尼)
    PID _pid_yaw;      // 轉向

    // 中間變數 (便於 Telemetry 觀察)
    float _target_pitch;
    float _target_pitch_rate;
    float _output_balance;
    float _output_turn;
};