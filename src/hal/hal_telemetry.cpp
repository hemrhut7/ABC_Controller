#include "hal_telemetry.h"
#include "hal_motor.h"
#include "hal_external_sensor.h"
#include <cstring>
#include <math.h>

// VOFA+ frame tail
const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};

#define TELEMETRY_QUEUE_LENGTH 5

Telemetry::Telemetry(Stream &stream, TelemetryPort_t port_id) : port(stream), port_id(port_id) {}

void Telemetry::init(uint16_t base_freq) {
  _base_freq = base_freq;
  data_queue = xQueueCreate(TELEMETRY_QUEUE_LENGTH, sizeof(system_state_t));
}

void Telemetry::push_data(const system_state_t &packet) {
  // If the queue is full, the oldest data will be overwritten.
  // This is crucial to ensure the high-frequency control loop is never blocked.
  if (data_queue != NULL) {
    xQueueSend(data_queue, &packet, 0);
  }
}

void Telemetry::set_config(bool enabled, uint8_t format, uint16_t freq_hz) {
  _enabled = enabled;
  _format = format;
  _packet_counter = 0; // Reset counter on config change

  if (port_id == PORT_UART1) {
    hal_external_sensor_set_bypass(enabled && (format == FORMAT_JETSON));
  }

  if (freq_hz == 0) {
    _divider = 1;
  } else {
    _divider = _base_freq / freq_hz;
    if (_divider == 0)
      _divider = 1;
  }
}

void Telemetry::send_vofa_packet(const float *data, size_t num_floats) {
  uint32_t data_size = num_floats * sizeof(float);
  uint8_t buffer[data_size + sizeof(vofa_tail)];
  memcpy(buffer, data, data_size);
  memcpy(buffer + data_size, vofa_tail, sizeof(vofa_tail));
  port.write(buffer, sizeof(buffer));
}

void Telemetry::process_serial_outgoing() {
  system_state_t pkt;

  // Use a short timeout to allow the task to check for incoming serial commands
  // even if no telemetry data is being pushed.
  if (xQueueReceive(data_queue, &pkt, pdMS_TO_TICKS(1)) == pdTRUE) {
    if (!_enabled) return;

    // 2. Control data rate using divider
    _packet_counter++;
    if (_packet_counter % _divider != 0) return;

    switch (_format) {
      case FORMAT_DEFAULT: {
        float data_packet[12];
        data_packet[0] = pkt.abc_state.ahrs_data.imu_data.timestamp * 1e-6f;
        data_packet[1] = pkt.abc_state.ahrs_data.euler[0] * RAD_TO_DEG;
        data_packet[2] = pkt.abc_state.ahrs_data.euler[1] * RAD_TO_DEG;
        data_packet[3] = pkt.abc_state.ahrs_data.euler[2] * RAD_TO_DEG;
        data_packet[4] = (float)pkt.abc_state.motor_state.rpm_L;
        data_packet[5] = (float)pkt.abc_state.motor_state.rpm_R;
        data_packet[6] = (float)pkt.abc_state.motor_state.pwm_out_L / (float)MAX_PWM_DUTY;
        data_packet[7] = (float)pkt.abc_state.motor_state.pwm_out_R / (float)MAX_PWM_DUTY;
        data_packet[8] = pkt.abc_state.velocity;
        data_packet[9] = (float)pkt.cmd.mode;
        data_packet[10] = (float)pkt.delay_count;
        data_packet[11] = pkt.battery_v;
        send_vofa_packet(data_packet, 12);
        break;
      }

      case FORMAT_PID: {
        float data_packet[15];
        data_packet[0] = pkt.abc_state.ahrs_data.imu_data.timestamp * 1e-6f;
        data_packet[1] = pkt.cmd.target_value;
        data_packet[2] = pkt.pid_target.rpm_L;
        data_packet[3] = pkt.pid_target.rpm_R;
        data_packet[4] = pkt.pid_target.pitch * RAD_TO_DEG;
        data_packet[5] = pkt.pid_target.velocity;
        data_packet[6] = pkt.pid_target.steer_rpm;
        data_packet[7] = (float)pkt.abc_state.motor_state.pwm_out_L;
        data_packet[8] = (float)pkt.abc_state.motor_state.pwm_out_R;
        data_packet[9] = (float)pkt.abc_state.motor_state.rpm_L;
        data_packet[10] = (float)pkt.abc_state.motor_state.rpm_R;
        data_packet[11] = pkt.abc_state.ahrs_data.euler[0] * RAD_TO_DEG;
        data_packet[12] = pkt.abc_state.velocity;
        data_packet[13] = pkt.abc_state.ahrs_data.imu_data_calibrated.gyro[2] * RAD_TO_DEG;
        data_packet[14] = pkt.battery_v;
        send_vofa_packet(data_packet, 15);
        break;
      }

      case FORMAT_SENSOR: {
        float data_packet[15];
        data_packet[0] = pkt.abc_state.ahrs_data.imu_data.timestamp * 1e-6f;
        data_packet[1] = pkt.abc_state.ahrs_data.imu_data.accl[0];
        data_packet[2] = pkt.abc_state.ahrs_data.imu_data.accl[1];
        data_packet[3] = pkt.abc_state.ahrs_data.imu_data.accl[2];
        data_packet[4] = pkt.abc_state.ahrs_data.imu_data.gyro[0] * RAD_TO_DEG;
        data_packet[5] = pkt.abc_state.ahrs_data.imu_data.gyro[1] * RAD_TO_DEG;
        data_packet[6] = pkt.abc_state.ahrs_data.imu_data.gyro[2] * RAD_TO_DEG;
        data_packet[7] = pkt.abc_state.ahrs_data.euler[0] * RAD_TO_DEG;
        data_packet[8] = pkt.abc_state.ahrs_data.euler[1] * RAD_TO_DEG;
        data_packet[9] = pkt.abc_state.ahrs_data.euler[2] * RAD_TO_DEG;
        data_packet[10] = pkt.abc_state.ahrs_data.mag[0];
        data_packet[11] = pkt.abc_state.ahrs_data.mag[1];
        data_packet[12] = pkt.abc_state.ahrs_data.mag[2];
        data_packet[13] = pkt.abc_state.ahrs_data.baro;
        data_packet[14] = pkt.battery_v;
        send_vofa_packet(data_packet, 15);
        break;
      }

      case FORMAT_JETSON: {
        // 1. Send Default packet (Status bypass)
        float data_packet[12];
        data_packet[0] = pkt.abc_state.ahrs_data.imu_data.timestamp * 1e-6f;
        data_packet[1] = pkt.abc_state.ahrs_data.euler[0] * RAD_TO_DEG;
        data_packet[2] = pkt.abc_state.ahrs_data.euler[1] * RAD_TO_DEG;
        data_packet[3] = pkt.abc_state.ahrs_data.euler[2] * RAD_TO_DEG;
        data_packet[4] = (float)pkt.abc_state.motor_state.rpm_L;
        data_packet[5] = (float)pkt.abc_state.motor_state.rpm_R;
        data_packet[6] = (float)pkt.abc_state.motor_state.pwm_out_L / (float)MAX_PWM_DUTY;
        data_packet[7] = (float)pkt.abc_state.motor_state.pwm_out_R / (float)MAX_PWM_DUTY;
        data_packet[8] = pkt.abc_state.velocity;
        data_packet[9] = (float)pkt.cmd.mode;
        data_packet[10] = (float)pkt.delay_count;
        data_packet[11] = pkt.battery_v;
        send_vofa_packet(data_packet, 12);

        // 2. Bypass raw sensor packets from StreamBuffer
        if (_sensor_stream != NULL) {
          uint8_t buffer[256];
          size_t received;
          while ((received = xStreamBufferReceive(_sensor_stream, buffer, sizeof(buffer), 0)) > 0) {
            port.write(buffer, received);
          }
        }
        break;
      }

      default:
        break;
    }
  }
}

void Telemetry::queue_string(const char *str) {
  if (!str)
    return;
  // This function is called from a communication task, so direct writing is
  // safe and won't interfere with the Control_Task.
  port.print(str);
}

void Telemetry::queue_string(const String &str) { queue_string(str.c_str()); }
