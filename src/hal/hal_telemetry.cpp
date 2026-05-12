#include "hal_telemetry.h"
#include "hal_motor.h"
#include <cstring>
#include <math.h>

// VOFA+ frame tail
const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};

#define TELEMETRY_QUEUE_LENGTH 5

Telemetry::Telemetry(Stream &stream, TelemetryPort_t port_id) : port(stream), port_id(port_id) {}
 
Telemetry::~Telemetry() {
  if (_lidar_scan) {
    free(_lidar_scan);
    _lidar_scan = nullptr;
  }
  if (data_queue) {
    vQueueDelete(data_queue);
    data_queue = nullptr;
  }
  if (_lidar_mutex) {
    vSemaphoreDelete(_lidar_mutex);
    _lidar_mutex = nullptr;
  }
}
 void Telemetry::init(uint16_t base_freq) {
  _base_freq = base_freq;
  data_queue = xQueueCreate(TELEMETRY_QUEUE_LENGTH, sizeof(system_state_t));
  _lidar_mutex = xSemaphoreCreateMutex();
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

  if (freq_hz == 0) {
    _divider = 1;
  } else {
    _divider = _base_freq / freq_hz;
    if (_divider == 0)
      _divider = 1;
  }
  
  // Manage lidar buffer memory
  if (_format == FORMAT_LIDAR) {
    if (!_lidar_scan) {
      _lidar_scan = (lidar_scan_t *)heap_caps_malloc(sizeof(lidar_scan_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      if (!_lidar_scan) {
          Serial.println("[TELE] PSRAM alloc failed for Lidar buffer!");
      }
    }
  } else {
    if (_lidar_scan) {
      free(_lidar_scan);
      _lidar_scan = nullptr;
    }
  }
}

void Telemetry::process_serial_outgoing() {
  system_state_t pkt;

  if (xQueueReceive(data_queue, &pkt, pdMS_TO_TICKS(1)) == pdTRUE) {
    if (!_enabled) return;

    _packet_counter++;
    if (_packet_counter % _divider != 0) return;

    if (_format == FORMAT_DEFAULT) {
      // A packet was successfully received.
      // Now, manually flatten the nested struct into a float array for VOFA+.
      float data_packet[18];
      data_packet[0] = pkt.abc_state.ahrs_data.imu_data.timestamp * 1e-6f;
      data_packet[1] = pkt.abc_state.ahrs_data.euler[0] * RAD_TO_DEG;
      data_packet[2] = pkt.abc_state.ahrs_data.euler[1] * RAD_TO_DEG;
      data_packet[3] = pkt.abc_state.ahrs_data.euler[2] * RAD_TO_DEG;
      data_packet[4] = (float)pkt.abc_state.motor_state.rpm_L;
      data_packet[5] = (float)pkt.abc_state.motor_state.rpm_R;
      data_packet[6] = (float)pkt.abc_state.motor_state.pwm_out_L / (float)MAX_PWM_DUTY;
      data_packet[7] = (float)pkt.abc_state.motor_state.pwm_out_R / (float)MAX_PWM_DUTY;
      data_packet[8] = pkt.abc_state.velocity;
      data_packet[9] = pkt.abc_state.ahrs_data.imu_data.gyro[0] * RAD_TO_DEG;
      data_packet[10] = pkt.abc_state.ahrs_data.imu_data.gyro[1] * RAD_TO_DEG;
      data_packet[11] = pkt.abc_state.ahrs_data.imu_data.gyro[2] * RAD_TO_DEG;
      data_packet[12] = pkt.abc_state.ahrs_data.imu_data.accl[0];
      data_packet[13] = pkt.abc_state.ahrs_data.imu_data.accl[1];
      data_packet[14] = pkt.abc_state.ahrs_data.imu_data.accl[2];
      data_packet[15] = (float)pkt.cmd.mode;
      data_packet[16] = (float)pkt.delay_count;
      data_packet[17] = pkt.battery_v;

      // Write the data packet and the tail to the serial port.
      uint8_t send_buffer[sizeof(data_packet) + sizeof(vofa_tail)];
      memcpy(send_buffer, data_packet, sizeof(data_packet));
      memcpy(send_buffer + sizeof(data_packet), vofa_tail, sizeof(vofa_tail));
      port.write(send_buffer, sizeof(send_buffer));
    } 
    else if (_format == FORMAT_PID) {
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

      // Write the data packet and the tail to the serial port.
      uint8_t send_buffer[sizeof(data_packet) + sizeof(vofa_tail)];
      memcpy(send_buffer, data_packet, sizeof(data_packet));
      memcpy(send_buffer + sizeof(data_packet), vofa_tail, sizeof(vofa_tail));
      port.write(send_buffer, sizeof(send_buffer));
    }
    else if (_format == FORMAT_LIDAR) {
      bool has_lidar = (_lidar_updated && _lidar_mutex && _lidar_scan);
      
      uint16_t pt_count = 0;
      if (has_lidar) {
          if (xSemaphoreTake(_lidar_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
              pt_count = _lidar_scan->count;
          } else {
              has_lidar = false; 
          }
      }

      // Preparation of base status data
      float status_packet[18];
      status_packet[0] = pkt.abc_state.ahrs_data.imu_data.timestamp * 1e-6f;
      status_packet[1] = pkt.abc_state.ahrs_data.euler[0] * RAD_TO_DEG;
      status_packet[2] = pkt.abc_state.ahrs_data.euler[1] * RAD_TO_DEG;
      status_packet[3] = pkt.abc_state.ahrs_data.euler[2] * RAD_TO_DEG;
      status_packet[4] = (float)pkt.abc_state.motor_state.rpm_L;
      status_packet[5] = (float)pkt.abc_state.motor_state.rpm_R;
      status_packet[6] = (float)pkt.abc_state.motor_state.pwm_out_L / (float)MAX_PWM_DUTY;
      status_packet[7] = (float)pkt.abc_state.motor_state.pwm_out_R / (float)MAX_PWM_DUTY;
      status_packet[8] = pkt.abc_state.velocity;
      status_packet[9] = pkt.abc_state.ahrs_data.imu_data.gyro[0] * RAD_TO_DEG;
      status_packet[10] = pkt.abc_state.ahrs_data.imu_data.gyro[1] * RAD_TO_DEG;
      status_packet[11] = pkt.abc_state.ahrs_data.imu_data.gyro[2] * RAD_TO_DEG;
      status_packet[12] = pkt.abc_state.ahrs_data.imu_data.accl[0];
      status_packet[13] = pkt.abc_state.ahrs_data.imu_data.accl[1];
      status_packet[14] = pkt.abc_state.ahrs_data.imu_data.accl[2];
      status_packet[15] = (float)pkt.cmd.mode;
      status_packet[16] = (float)pkt.delay_count;
      status_packet[17] = pkt.battery_v;

      // Chunked Sending Logic (Optimized for UDP/Network performance)
      static uint8_t* tx_chunk = nullptr;
      if (!tx_chunk) tx_chunk = (uint8_t*)heap_caps_malloc(1472, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      
      if (tx_chunk) {
          size_t offset = 0;
          float count_f = (float)pt_count;
          
          // Copy Status (72) + Count (4)
          memcpy(tx_chunk + offset, status_packet, 72); offset += 72;
          memcpy(tx_chunk + offset, &count_f, 4);       offset += 4;

          if (has_lidar) {
              for (int i = 0; i < pt_count; i++) {
                  float pt[5];
                  pt[0] = _lidar_scan->points[i].x;
                  pt[1] = _lidar_scan->points[i].y;
                  pt[2] = _lidar_scan->points[i].distance;
                  pt[3] = _lidar_scan->points[i].angle;
                  pt[4] = (float)_lidar_scan->points[i].intensity;
                  
                  memcpy(tx_chunk + offset, pt, 20); offset += 20;
                  
                  // If buffer near MTU (approx 1400 bytes), send it
                  if (offset >= 1400) {
                      port.write(tx_chunk, offset);
                      offset = 0;
                  }
              }
              _lidar_updated = false;
              xSemaphoreGive(_lidar_mutex);
          }
          
          // Final chunk: Append tail and send
          memcpy(tx_chunk + offset, vofa_tail, 4); offset += 4;
          port.write(tx_chunk, offset);
      }
    }
  }
}

void Telemetry::update_lidar_data(const lidar_scan_t &scan) {
  if (_lidar_scan && _lidar_mutex) {
    if (xSemaphoreTake(_lidar_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      *_lidar_scan = scan;
      _lidar_updated = true;
      xSemaphoreGive(_lidar_mutex);
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
