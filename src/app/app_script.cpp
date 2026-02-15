#include "app_script.h"

AppScript::AppScript(AppMode* app_mode) : _app_mode(app_mode) {
}

void AppScript::check_serial(Stream& stream, Telemetry *telemetry) {
    if (stream.available()) {
        String rx_line = stream.readStringUntil('\n');
        rx_line.trim(); // 去除換行符號
        if (rx_line.length() > 0) {
            parse_packet(rx_line, telemetry);
        }
    }
}

void AppScript::parse_packet(const String& packet, Telemetry *telemetry) {
    if (!_app_mode || !telemetry) return;

    // 1. 預處理：建立副本、轉大寫、去除空白，提升指令容錯率
    String cmd_line = packet;
    cmd_line.trim();
    cmd_line.toUpperCase();

    if (cmd_line.length() == 0) return;

    char tx_buffer[256]; // Buffer for formatting response strings

    // 1. PID Tuning 指令: "PID <id> <kp> <ki> <kd>"
    // ID Mapping: 0=MOTOR, 1=RATE, 2=ANGLE, 3=VELOCITY, 4=YAW
    if (cmd_line.startsWith("PID")) {
        int id;
        float kp, ki, kd;
        if (sscanf(cmd_line.c_str(), "PID %d %f %f %f", &id, &kp, &ki, &kd) == 4) {
            // 安全檢查: ID 範圍
            if (id >= 0 && id < PID_ID_COUNT) {
                _app_mode->set_pid_gains((PID_id_t)id, kp, ki, kd);
                snprintf(tx_buffer, sizeof(tx_buffer), "[OK] PID %d Updated: P=%.3f I=%.3f D=%.3f\n", id, kp, ki, kd);
                telemetry->queue_string(tx_buffer);
            } else {
                snprintf(tx_buffer, sizeof(tx_buffer), "[ERR] PID ID out of range (0-%d)\n", PID_ID_COUNT - 1);
                telemetry->queue_string(tx_buffer);
            }
        } else {
            telemetry->queue_string("[ERR] Invalid PID format. Usage: PID <id> <kp> <ki> <kd>\n");
        }
    }
    // 2. 模式切換指令: "MODE <mode>"
    else if (cmd_line.startsWith("MODE")) {
        int mode;
        if (sscanf(cmd_line.c_str(), "MODE %d", &mode) == 1) {
            // 安全檢查: Mode 範圍 (0-6)
            if (mode >= MODE_STOP && mode <= MODE_FREE) {
                _app_mode->set_mode((Mode_t)mode);
                snprintf(tx_buffer, sizeof(tx_buffer), "[OK] Mode Set: %d\n", mode);
                telemetry->queue_string(tx_buffer);
            } else {
                snprintf(tx_buffer, sizeof(tx_buffer), "[ERR] Invalid Mode (0-%d)\n", MODE_FREE);
                telemetry->queue_string(tx_buffer);
            }
        } else {
            telemetry->queue_string("[ERR] Invalid MODE format. Usage: MODE <id>\n");
        }
    }
    // 3. 控制指令: "VAL <val> <yaw>"
    else if (cmd_line.startsWith("VAL")) {
        float val, yaw;
        if (sscanf(cmd_line.c_str(), "VAL %f %f", &val, &yaw) == 2) {
            _app_mode->set_target(val, yaw);
        }
    }
    // 4. 讀取 PID 指令
    else if (cmd_line.startsWith("GET PID")) {
        int id;
        // 檢查是 "GET PID <id>" 還是 "GET PID"
        if (sscanf(cmd_line.c_str(), "GET PID %d", &id) == 1) {
            // 獲取單個 PID
            if (id >= 0 && id < PID_ID_COUNT) {
                PID_Params p = _app_mode->get_pid_gains((PID_id_t)id);
                snprintf(tx_buffer, sizeof(tx_buffer), "[OK] PID %d: P=%.3f, I=%.3f, D=%.3f\n", id, p.p, p.i, p.d);
                telemetry->queue_string(tx_buffer);
            } else {
                snprintf(tx_buffer, sizeof(tx_buffer), "[ERR] PID ID out of range (0-%d)\n", PID_ID_COUNT - 1);
                telemetry->queue_string(tx_buffer);
            }
        } else {
            // 獲取所有 PID
            const char* pid_names[] = {"MOTOR", "RATE", "ANGLE", "VELOCITY", "YAW"};
            for (int i = 0; i < PID_ID_COUNT; i++) {
                PID_Params p = _app_mode->get_pid_gains((PID_id_t)i);
                snprintf(tx_buffer, sizeof(tx_buffer), "[OK] PID %d (%s): P=%.3f, I=%.3f, D=%.3f\n", i, pid_names[i], p.p, p.i, p.d);
                telemetry->queue_string(tx_buffer);
            }
        }
    }
}