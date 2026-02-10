#include "app_script.h"

AppScript::AppScript(AppMode* app_mode) : _app_mode(app_mode) {
}

void AppScript::check_serial(Stream& stream) {
    if (stream.available()) {
        String rx_line = stream.readStringUntil('\n');
        rx_line.trim(); // 去除換行符號
        if (rx_line.length() > 0) {
            parse_packet(rx_line);
        }
    }
}

void AppScript::parse_packet(const String& packet) {
    if (!_app_mode) return;

    // 1. 預處理：建立副本、轉大寫、去除空白，提升指令容錯率
    String cmd_line = packet;
    cmd_line.trim();
    cmd_line.toUpperCase();

    if (cmd_line.length() == 0) return;

    // 1. PID Tuning 指令: "PID <id> <kp> <ki> <kd>"
    // ID Mapping: 0=Velocity, 1=Angle, 2=Rate, 3=Yaw
    if (cmd_line.startsWith("PID")) {
        int id;
        float kp, ki, kd;
        if (sscanf(cmd_line.c_str(), "PID %d %f %f %f", &id, &kp, &ki, &kd) == 4) {
            // 安全檢查: ID 範圍 (0-3)
            if (id >= 0 && id <= 3) {
                _app_mode->set_pid_gains((uint8_t)id, kp, ki, kd);
                Serial.printf("[OK] PID %d Updated: P=%.3f I=%.3f D=%.3f\n", id, kp, ki, kd);
            } else {
                Serial.println("[ERR] PID ID out of range (0-3)");
            }
        } else {
            Serial.println("[ERR] Invalid PID format. Usage: PID <id> <kp> <ki> <kd>");
        }
    }
    // 2. 模式切換指令: "MODE <mode>"
    // Mode Mapping: 0=STOP, 3=ANGLE, 4=VELOCITY, 5=REMOTE
    else if (cmd_line.startsWith("MODE")) {
        int mode;
        if (sscanf(cmd_line.c_str(), "MODE %d", &mode) == 1) {
            // 安全檢查: Mode 範圍 (0-6)
            if (mode >= 0 && mode <= 6) {
                _app_mode->set_mode((Mode_t)mode);
                Serial.printf("[OK] Mode Set: %d\n", mode);
            } else {
                Serial.println("[ERR] Invalid Mode (0-6)");
            }
        } else {
            Serial.println("[ERR] Invalid MODE format. Usage: MODE <id>");
        }
    }
    // 3. 控制指令: "VAL <val> <yaw>"
    else if (cmd_line.startsWith("VAL")) {
        float val, yaw;
        if (sscanf(cmd_line.c_str(), "VAL %f %f", &val, &yaw) == 2) {
            _app_mode->set_target(val, yaw);
            // 高頻指令通常不回傳 Log 以節省頻寬
        }
    }
}