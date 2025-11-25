#pragma once

#include <Arduino.h>
#include "type_defs.h"
#include "hal/hal_motor.h"

// 為所有可呼叫的函式定義一個統一的簽名
// 參數 args 將是命令後面的整個參數字串
typedef void (*CommandHandler)(const char* args);

// 定義命令的結構
struct Command {
    const char* name;       // 命令名稱 (例如 "setpid")
    CommandHandler handler; // 指向處理此命令的函式指標
};

// 函式宣告
void setup_script(HAL_Motor* left, HAL_Motor* right);
void handle_serial_commands(Stream &port);

// --- 命令處理函式宣告 ---
void cmd_hello(const char* args);
void cmd_set_motor_pid(const char* args);
void cmd_set_rpm(const char* args);