#pragma once

typedef enum {
    MODE_STOP = 0,      // 安全停機
    MODE_MOTOR_TEST,    // 1. 直接控制馬達轉速 (Open/Close Loop)
    MODE_RATE,          // 2. 控制角速度 (極難平衡，僅供調試 D term)
    MODE_ANGLE,         // 3. 控制傾角 (最常用的調試模式)
    MODE_VELOCITY,      // 4. 控制前進速度 (加上速度環)
    MODE_REMOTE,        // 5. 綜合遙控 (速度 + 轉向)
} Mode_t;

typedef struct {
    Mode_t mode;
    float target_value; // 根據模式不同，單位可能是 RPM, rad/s, rad, m/s
    float target_yaw_rate; // 轉向指令 (rad/s)
} UserCommand_t;