#include "app_script.h"
#include "hal/hal_microros.h"
#if HAS_WIFI_SERIAL
#include "hal/hal_udp_stream.h"
#endif

extern HAL_MicroROS uros_telemetry;

AppScript::AppScript(AppMode *app_mode) : _app_mode(app_mode) {}

void AppScript::check_serial(Stream &stream, Telemetry *telemetry) {
  if (stream.available()) {
    String rx_line = stream.readStringUntil('\n');
    rx_line.trim(); // 去除換行符號
    if (rx_line.length() > 0) {
      parse_packet(rx_line, stream, telemetry);
    }
  }
}

void AppScript::parse_packet(const String &packet, Stream &response_stream, Telemetry *telemetry) {
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
    if (cmd_line == "PID SAVE" || cmd_line == "CONFIG SAVE" || cmd_line == "SAVE") {
      _app_mode->save_pid_gains();
      snprintf(tx_buffer, sizeof(tx_buffer),
               "[OK] Config Saved to EEPROM\n");
      response_stream.print(tx_buffer);
      return;
    }

    int id;
    float kp, ki, kd;
    if (sscanf(cmd_line.c_str(), "PID %d %f %f %f", &id, &kp, &ki, &kd) == 4) {
      // 安全檢查: ID 範圍
      if (id >= 0 && id < PID_ID_COUNT) {
        // Principal Engineer Safety Check: Reject negative, NaN, or infinite gains
        if (isnan(kp) || isinf(kp) || kp < 0.0f ||
            isnan(ki) || isinf(ki) || ki < 0.0f ||
            isnan(kd) || isinf(kd) || kd < 0.0f) {
          snprintf(tx_buffer, sizeof(tx_buffer),
                   "[ERR] Rejected PID %d: Values must be finite and non-negative\n", id);
          response_stream.print(tx_buffer);
          return;
        }
        _app_mode->set_pid_gains((PID_id_t)id, kp, ki, kd);
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[OK] PID %d Updated: P=%.3f I=%.3f D=%.3f\n", id, kp, ki, kd);
        response_stream.print(tx_buffer);
      } else {
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[ERR] PID ID out of range (0-%d)\n", PID_ID_COUNT - 1);
        response_stream.print(tx_buffer);
      }
    } else {
      response_stream.print(
          "[ERR] Invalid PID format. Usage: PID <id> <kp> <ki> <kd>\n");
    }
  }
  // 2. 模式切換指令: "MODE <mode>"
  else if (cmd_line.startsWith("MODE")) {
    int mode;
    if (sscanf(cmd_line.c_str(), "MODE %d", &mode) == 1) {
      // 安全檢查: Mode 範圍 (0-6)
      if (mode >= MODE_STOP && mode <= MODE_AUTO) {
        if (_app_mode->enqueue_mode((Mode_t)mode)) {
          snprintf(tx_buffer, sizeof(tx_buffer), "[OK] Mode Queued: %d\n", mode);
        } else {
          snprintf(tx_buffer, sizeof(tx_buffer),
                   "[ERR] Command queue full (MODE)\n");
        }
        response_stream.print(tx_buffer);
      } else {
        snprintf(tx_buffer, sizeof(tx_buffer), "[ERR] Invalid Mode (0-%d)\n",
                 MODE_AUTO);
        response_stream.print(tx_buffer);
      }
    } else {
      response_stream.print("[ERR] Invalid MODE format. Usage: MODE <id>\n");
    }
  }
  // 3. 控制指令: "VAL <val> <steer>"
  else if (cmd_line.startsWith("VAL")) {
    float val, steer;
    if (sscanf(cmd_line.c_str(), "VAL %f %f", &val, &steer) == 2) {
      if (!_app_mode->enqueue_target(val, steer)) {
        response_stream.print("[ERR] Command queue full (VAL)\n");
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
        response_stream.print(tx_buffer);
      } else {
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[ERR] PID ID out of range (0-%d)\n", PID_ID_COUNT - 1);
        response_stream.print(tx_buffer);
      }
    } else {
      // 獲取所有 PID
      const char *pid_names[] = {"MOTOR", "RATE", "ANGLE", "VELOCITY", "STEER"};
      for (int i = 0; i < PID_ID_COUNT; i++) {
        PID_Params p = _app_mode->get_pid_gains((PID_id_t)i);
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[OK] PID %d (%s): P=%.3f, I=%.3f, D=%.3f\n", i, pid_names[i],
                 p.p, p.i, p.d);
        response_stream.print(tx_buffer);
      }
    }
  }
  // 5. 遙測控制指令: "TELE <enabled> <port_id> <format> <freq_hz>"
  else if (cmd_line.startsWith("TELE")) {
    if (uros_telemetry.is_connected()) {
      response_stream.print("[ERR] Cannot modify Telemetry config while micro-ROS is active\n");
    } else {
      int enabled, port_id, format, freq_hz;
      if (sscanf(cmd_line.c_str(), "TELE %d %d %d %d", &enabled, &port_id, &format,
                 &freq_hz) == 4) {
        Stream *new_stream = nullptr;
        if (port_id == PORT_USB) {
          new_stream = &Serial;
        } else if (port_id == PORT_UART1) {
          new_stream = &Serial1;
        } else if (port_id == PORT_WIFI) {
#if HAS_WIFI_SERIAL
          extern UDPStream udp_stream;
          new_stream = &udp_stream;
#endif
        }

        if (new_stream != nullptr) {
          telemetry->set_port(*new_stream, (TelemetryPort_t)port_id);
          telemetry->set_config(enabled != 0, (uint8_t)format, (uint16_t)freq_hz);
          snprintf(tx_buffer, sizeof(tx_buffer),
                   "[OK] Telemetry switched to port %d: En=%d, Fmt=%d, Freq=%dHz\n", port_id, enabled, format,
                   freq_hz);
          response_stream.print(tx_buffer);
        } else {
          snprintf(tx_buffer, sizeof(tx_buffer),
                   "[ERR] Port %d not available or unsupported\n", port_id);
          response_stream.print(tx_buffer);
        }
      } else {
        response_stream.print("[ERR] Invalid TELE format. Usage: TELE <enabled> <port_id> <format> <freq_hz>\n");
      }
    }
  }
  // 6. LPF 設定指令: "LPF <id> <freq>"
  else if (cmd_line.startsWith("LPF")) {
    if (cmd_line == "LPF SAVE") {
      _app_mode->save_pid_gains();
      snprintf(tx_buffer, sizeof(tx_buffer),
               "[OK] LPF Config Saved to EEPROM\n");
      response_stream.print(tx_buffer);
      return;
    }

    int id;
    float freq;
    if (sscanf(cmd_line.c_str(), "LPF %d %f", &id, &freq) == 2) {
      if (id >= 0 && id < LPF_ID_COUNT) {
        _app_mode->set_cut_off_freq((LPF_id_t)id, freq);
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[OK] LPF %d Updated: Freq=%.2f Hz\n", id, freq);
        response_stream.print(tx_buffer);
      } else {
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[ERR] LPF ID out of range (0-%d)\n", LPF_ID_COUNT - 1);
        response_stream.print(tx_buffer);
      }
    } else {
      response_stream.print(
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
        response_stream.print(tx_buffer);
      } else {
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[ERR] LPF ID out of range (0-%d)\n", LPF_ID_COUNT - 1);
        response_stream.print(tx_buffer);
      }
    } else {
        const char *lpf_names[] = {"VELOCITY", "STEER", "GYRO_Z", "CURR_VEL"};
        for (int i = 0; i < LPF_ID_COUNT; i++) {
            float f = _app_mode->get_lpf_freq((LPF_id_t)i);
            snprintf(tx_buffer, sizeof(tx_buffer),
                     "[OK] LPF %d (%s): Freq=%.2f Hz\n", i, lpf_names[i], f);
            response_stream.print(tx_buffer);
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
        response_stream.print(tx_buffer);
      } else {
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[ERR] Ramp ID out of range (0-%d)\n", PARAM_RAMP_ID_COUNT - 1);
        response_stream.print(tx_buffer);
      }
    } else {
      response_stream.print(
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
        response_stream.print(tx_buffer);
      } else {
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[ERR] Ramp ID out of range (0-%d)\n", PARAM_RAMP_ID_COUNT - 1);
        response_stream.print(tx_buffer);
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
        response_stream.print(tx_buffer);
      }
    }
  }

  // 10. 靜止狀態 PID 倍率指令: "STATIC <kp_scale> <ki_scale> <kd_scale>"
  else if (cmd_line.startsWith("STATIC")) {
    if (cmd_line == "STATIC SAVE" || cmd_line == "STATIC_PID SAVE") {
      _app_mode->save_pid_gains();
      snprintf(tx_buffer, sizeof(tx_buffer),
               "[OK] Static PID Scale Saved to EEPROM\n");
      response_stream.print(tx_buffer);
      return;
    }

    float kp_s, ki_s, kd_s;
    if (sscanf(cmd_line.c_str(), "STATIC %f %f %f", &kp_s, &ki_s, &kd_s) == 3 ||
        sscanf(cmd_line.c_str(), "STATIC_PID %f %f %f", &kp_s, &ki_s, &kd_s) == 3) {
      if (isnan(kp_s) || isinf(kp_s) || kp_s < 0.0f ||
          isnan(ki_s) || isinf(ki_s) || ki_s < 0.0f ||
          isnan(kd_s) || isinf(kd_s) || kd_s < 0.0f) {
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[ERR] Rejected Static Scale: Values must be finite and non-negative\n");
        response_stream.print(tx_buffer);
        return;
      }
      _app_mode->set_static_pid_scale(kp_s, ki_s, kd_s);
      snprintf(tx_buffer, sizeof(tx_buffer),
               "[OK] Static Scale Updated: P=%.3f I=%.3f D=%.3f\n", kp_s, ki_s, kd_s);
      response_stream.print(tx_buffer);
    } else {
      response_stream.print(
          "[ERR] Invalid STATIC format. Usage: STATIC <kp_scale> <ki_scale> <kd_scale>\n");
    }
  }
  // 11. 讀取靜止狀態 PID 倍率指令: "GET STATIC"
  else if (cmd_line.startsWith("GET STATIC")) {
    PID_Params s = _app_mode->get_static_pid_scale();
    snprintf(tx_buffer, sizeof(tx_buffer),
             "[OK] Static Scale: P=%.3f, I=%.3f, D=%.3f\n", s.p, s.i, s.d);
    response_stream.print(tx_buffer);
  }
}
