#include "task_script.h"

// --- 全域變數 ---
// 讓命令處理函式可以存取馬達物件
static HAL_Motor* motor_l = nullptr;
static HAL_Motor* motor_r = nullptr;

// --- 命令列表 ---
// 在這裡新增你的命令。陣列大小會自動計算。
const Command command_list[] = {
    {"hello",   cmd_hello},
    {"setpid_mo",  cmd_set_motor_pid},
    {"setrpm",  cmd_set_rpm},
    // 未來可以在此處繼續增加新的命令...
};
const int num_commands = sizeof(command_list) / sizeof(Command);


/**
 * @brief 初始化腳本系統，傳入馬達物件的指標
 */
void setup_script(HAL_Motor* left, HAL_Motor* right) {
    motor_l = left;
    motor_r = right;
}

/**
 * @brief 處理傳入的字串命令
 * @param command_line 從序列埠讀取到的完整字串 (例如 "$setpid 1,2,3")
 */
void process_command(char* command_line) {
    if (command_line[0] != '$') {
        return; // 如果不是以 '$' 開頭，則忽略
    }

    // 跳過 '$'
    char* command = command_line + 1;
    char* arg_ptr = strchr(command, ' '); // 尋找第一個空格來分離命令和參數
    const char* args;

    if (arg_ptr != nullptr) {
        *arg_ptr = '\0'; // 將空格替換為字串結束符，分離出命令
        args = arg_ptr + 1;       // 參數字串的起始位置
    } else {
        args = ""; // 如果沒有參數，傳入空字串
    }

    // 在命令列表中尋找匹配的命令
    for (int i = 0; i < num_commands; i++) {
        if (strcmp(command, command_list[i].name) == 0) {
            command_list[i].handler(args); // 找到後執行對應的函式
            return;
        }
    }

    Serial.print("Unknown command: ");
    Serial.println(command);
}

/**
 * @brief 從指定的序列埠讀取並處理命令
 */
void handle_serial_commands(Stream &port) {
    static char command_buffer[64];
    static uint8_t buffer_pos = 0;

    while (port.available()) {
        char c = port.read();
        if (c == '\n' || c == '\r') {
            if (buffer_pos > 0) {
                command_buffer[buffer_pos] = '\0';
                process_command(command_buffer);
                buffer_pos = 0; // 重置緩衝區
            }
        } else if (buffer_pos < sizeof(command_buffer) - 1) {
            command_buffer[buffer_pos++] = c;
        }
    }
}

// --- 命令處理函式實作 ---

void cmd_hello(const char* args) {
    Serial.println("Hello from your device!");
}

void cmd_set_motor_pid(const char* args) {
    int p, i, d;
    // sscanf 會從字串中解析格式化的輸入
    if (sscanf(args, "%d,%d,%d", &p, &i, &d) == 3) {
        motor_l->set_pid_gains(p, i, d);
        motor_r->set_pid_gains(p, i, d);
        Serial.print("Set PID gains to: P=");
        Serial.print(p);
        Serial.print(", I=");
        Serial.print(i);
        Serial.print(", D=");
        Serial.println(d);
    } else {
        Serial.println("Invalid arguments for setpid. Use: $setpid p,i,d");
    }
}

void cmd_set_rpm(const char* args) {
    char motor_char;
    int rpm;
    if (sscanf(args, "%c,%d", &motor_char, &rpm) == 2) {
        if (motor_char == 'l' || motor_char == 'L') {
            // motor_l->set_rpm(rpm); // 假設你有 set_rpm 函式
            Serial.print("Set Left Motor RPM to: ");
            Serial.println(rpm);
        } else if (motor_char == 'r' || motor_char == 'R') {
            // motor_r->set_rpm(rpm); // 假設你有 set_rpm 函式
            Serial.print("Set Right Motor RPM to: ");
            Serial.println(rpm);
        } else {
            Serial.println("Invalid motor specified. Use 'l' for left or 'r' for right.");
        }
    } else {
        Serial.println("Invalid arguments for setrpm. Use: $setrpm [l|r],rpm");
    }
}