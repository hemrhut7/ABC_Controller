#include "app_script.h"

AppScript::AppScript(AppMode *app_mode) : _app_mode(app_mode) {}

void AppScript::check_serial(Stream &stream, Telemetry *telemetry) {
  if (stream.available()) {
    String rx_line = stream.readStringUntil('\n');
    rx_line.trim(); // 去除換行符號
    if (rx_line.length() > 0) {
      parse_packet(rx_line, telemetry);
    }
  }
}

void AppScript::parse_packet(const String &packet, Telemetry *telemetry) {
  if (!_app_mode || !telemetry)
    return;

  // 1. 預處理：建立副本、轉大寫、去除空白，提升指令容錯率
  String cmd_line = packet;
  cmd_line.trim();
  cmd_line.toUpperCase();

  if (cmd_line.length() == 0)
    return;

  char tx_buffer[256]; // Buffer for formatting response strings

  // 1. PID Tuning 指令: "PID <id> <kp> <ki> <kd>"
  // ID Mapping: 0=MOTOR, 1=RATE, 2=ANGLE, 3=VELOCITY, 4=STEER
  if (cmd_line.startsWith("PID")) {
    if (cmd_line == "PID SAVE") {
      _app_mode->save_pid_gains();
      snprintf(tx_buffer, sizeof(tx_buffer),
               "[OK] PID Config Saved to EEPROM\n");
      telemetry->queue_string(tx_buffer);
      return;
    }

    int id;
    float kp, ki, kd;
    if (sscanf(cmd_line.c_str(), "PID %d %f %f %f", &id, &kp, &ki, &kd) == 4) {
      // 安全檢查: ID 範圍
      if (id >= 0 && id < PID_ID_COUNT) {
        _app_mode->set_pid_gains((PID_id_t)id, kp, ki, kd);
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[OK] PID %d Updated: P=%.3f I=%.3f D=%.3f\n", id, kp, ki, kd);
        telemetry->queue_string(tx_buffer);
      } else {
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[ERR] PID ID out of range (0-%d)\n", PID_ID_COUNT - 1);
        telemetry->queue_string(tx_buffer);
      }
    } else {
      telemetry->queue_string(
          "[ERR] Invalid PID format. Usage: PID <id> <kp> <ki> <kd>\n");
    }
  }
  // 2. 模式切換指令: "MODE <mode>"
  else if (cmd_line.startsWith("MODE")) {
    int mode;
    if (sscanf(cmd_line.c_str(), "MODE %d", &mode) == 1) {
      // 安全檢查: Mode 範圍 (0-6)
      if (mode >= MODE_STOP && mode <= MODE_REMOTE) {
        if (_app_mode->enqueue_mode((Mode_t)mode)) {
          snprintf(tx_buffer, sizeof(tx_buffer), "[OK] Mode Queued: %d\n", mode);
        } else {
          snprintf(tx_buffer, sizeof(tx_buffer),
                   "[ERR] Command queue full (MODE)\n");
        }
        telemetry->queue_string(tx_buffer);
      } else {
        snprintf(tx_buffer, sizeof(tx_buffer), "[ERR] Invalid Mode (0-%d)\n",
                 MODE_REMOTE);
        telemetry->queue_string(tx_buffer);
      }
    } else {
      telemetry->queue_string("[ERR] Invalid MODE format. Usage: MODE <id>\n");
    }
  }
  // 3. 控制指令: "VAL <val> <steer>"
  else if (cmd_line.startsWith("VAL")) {
    float val, steer;
    if (sscanf(cmd_line.c_str(), "VAL %f %f", &val, &steer) == 2) {
      if (!_app_mode->enqueue_target(val, steer)) {
        telemetry->queue_string("[ERR] Command queue full (VAL)\n");
      }
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
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[OK] PID %d: P=%.3f, I=%.3f, D=%.3f\n", id, p.p, p.i, p.d);
        telemetry->queue_string(tx_buffer);
      } else {
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[ERR] PID ID out of range (0-%d)\n", PID_ID_COUNT - 1);
        telemetry->queue_string(tx_buffer);
      }
    } else {
      // 獲取所有 PID
      const char *pid_names[] = {"MOTOR", "RATE", "ANGLE", "VELOCITY", "STEER"};
      for (int i = 0; i < PID_ID_COUNT; i++) {
        PID_Params p = _app_mode->get_pid_gains((PID_id_t)i);
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[OK] PID %d (%s): P=%.3f, I=%.3f, D=%.3f\n", i, pid_names[i],
                 p.p, p.i, p.d);
        telemetry->queue_string(tx_buffer);
      }
    }
  }
  // 5. 遙測控制指令: "TELE <enabled> <port_id> <format> <freq_hz>"
  else if (cmd_line.startsWith("TELE")) {
    int enabled, port_id, format, freq_hz;
    if (sscanf(cmd_line.c_str(), "TELE %d %d %d %d", &enabled, &port_id, &format,
               &freq_hz) == 4) {
      if (port_id != telemetry->get_port_id()) return;

      telemetry->set_config(enabled != 0, (uint8_t)format, (uint16_t)freq_hz);
      snprintf(tx_buffer, sizeof(tx_buffer),
               "[OK] Telemetry: En=%d, Fmt=%d, Freq=%dHz\n", enabled, format,
               freq_hz);
      telemetry->queue_string(tx_buffer);
    } else {
      telemetry->queue_string("[ERR] Invalid TELE format. Usage: TELE "
                              "<enabled> <divider> <format>\n");
    }
  }
  // 6. LPF 設定指令: "LPF <id> <freq>"
  else if (cmd_line.startsWith("LPF")) {
    int id;
    float freq;
    if (sscanf(cmd_line.c_str(), "LPF %d %f", &id, &freq) == 2) {
      if (id >= 0 && id < LPF_ID_COUNT) {
        _app_mode->set_cut_off_freq((LPF_id_t)id, freq);
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[OK] LPF %d Updated: Freq=%.2f Hz\n", id, freq);
        telemetry->queue_string(tx_buffer);
      } else {
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[ERR] LPF ID out of range (0-%d)\n", LPF_ID_COUNT - 1);
        telemetry->queue_string(tx_buffer);
      }
    } else {
      telemetry->queue_string(
          "[ERR] Invalid LPF format. Usage: LPF <id> <freq>\n");
    }
  }
  // 7. 讀取 LPF 指令: "GET LPF [<id>]"
  else if (cmd_line.startsWith("GET LPF")) {
    int id;
    if (sscanf(cmd_line.c_str(), "GET LPF %d", &id) == 1) {
      if (id >= 0 && id < LPF_ID_COUNT) {
        float f = _app_mode->get_lpf_freq((LPF_id_t)id);
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[OK] LPF %d: Freq=%.2f Hz\n", id, f);
        telemetry->queue_string(tx_buffer);
      } else {
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[ERR] LPF ID out of range (0-%d)\n", LPF_ID_COUNT - 1);
        telemetry->queue_string(tx_buffer);
      }
    } else {
        const char *lpf_names[] = {"VELOCITY", "STEER", "GYRO_Z", "CURR_VEL"};
        for (int i = 0; i < LPF_ID_COUNT; i++) {
            float f = _app_mode->get_lpf_freq((LPF_id_t)i);
            snprintf(tx_buffer, sizeof(tx_buffer),
                     "[OK] LPF %d (%s): Freq=%.2f Hz\n", i, lpf_names[i], f);
            telemetry->queue_string(tx_buffer);
        }
    }
  }

  // 8. 斜率設定指令: "RAMP <id> <val>"
  else if (cmd_line.startsWith("RAMP")) {
    int id;
    float val;
    if (sscanf(cmd_line.c_str(), "RAMP %d %f", &id, &val) == 2) {
      if (id >= 0 && id < PARAM_RAMP_ID_COUNT) {
        float internal_val = val;
        const char *unit = "";
        if (id == PARAM_RAMP_PITCH) {
          internal_val = val * DEG_TO_RAD;
          unit = " deg/s";
        }
        _app_mode->set_ramp((PARAM_RAMP_id_t)id, internal_val);
        snprintf(tx_buffer, sizeof(tx_buffer), "[OK] Ramp %d Updated: %.3f%s\n",
                 id, val, unit);
        telemetry->queue_string(tx_buffer);
      } else {
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[ERR] Ramp ID out of range (0-%d)\n", PARAM_RAMP_ID_COUNT - 1);
        telemetry->queue_string(tx_buffer);
      }
    } else {
      telemetry->queue_string(
          "[ERR] Invalid RAMP format. Usage: RAMP <id> <val>\n");
    }
  }
  // 9. 讀取斜率指令: "GET RAMP [<id>]"
  else if (cmd_line.startsWith("GET RAMP")) {
    int id;
    if (sscanf(cmd_line.c_str(), "GET RAMP %d", &id) == 1) {
      if (id >= 0 && id < PARAM_RAMP_ID_COUNT) {
        float val = _app_mode->get_ramp((PARAM_RAMP_id_t)id);
        const char *unit = "";
        if (id == PARAM_RAMP_PITCH) {
          val *= RAD_TO_DEG;
          unit = " deg/s";
        }
        snprintf(tx_buffer, sizeof(tx_buffer), "[OK] Ramp %d: %.3f%s\n", id, val,
                 unit);
        telemetry->queue_string(tx_buffer);
      } else {
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[ERR] Ramp ID out of range (0-%d)\n", PARAM_RAMP_ID_COUNT - 1);
        telemetry->queue_string(tx_buffer);
      }
    } else {
      const char *ramp_names[] = {"PITCH", "VELOCITY"};
      for (int i = 0; i < PARAM_RAMP_ID_COUNT; i++) {
        float val = _app_mode->get_ramp((PARAM_RAMP_id_t)i);
        const char *unit = "";
        if (i == PARAM_RAMP_PITCH) {
          val *= RAD_TO_DEG;
          unit = " deg/s";
        }
        snprintf(tx_buffer, sizeof(tx_buffer), "[OK] Ramp %d (%s): %.3f%s\n", i,
                 ramp_names[i], val, unit);
        telemetry->queue_string(tx_buffer);
      }
    }
  }
}
