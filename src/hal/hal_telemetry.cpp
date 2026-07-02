#include "hal_telemetry.h"
#include "hal_motor.h"
#include <cstring>
#include <math.h>

// Telemetry packet protocol constants
static const uint8_t packet_header[2] = {0xAA, 0x55};
static const uint8_t packet_tail[4]   = {0x00, 0x00, 0x80, 0x7f}; // Keep original tail bytes for compatibility

enum MSG_ID {
    MSG_ID_DEFAULT = 0x01,
    MSG_ID_PID     = 0x02,
    MSG_ID_LIDAR   = 0x03,
};

// Helper function to calculate CRC-16 CCITT
static uint16_t calculate_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

#define TELEMETRY_QUEUE_LENGTH 5
#define LEN_PAYLOAD_DEFAULT 23
#define LEN_PAYLOAD_PID 15


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
      float payload[LEN_PAYLOAD_DEFAULT];
      payload[0] = pkt.abc_state.ahrs_data.imu_data.timestamp * 1e-6f;
      payload[1] = pkt.abc_state.ahrs_data.euler[0] * RAD_TO_DEG;
      payload[2] = pkt.abc_state.ahrs_data.euler[1] * RAD_TO_DEG;
      payload[3] = pkt.abc_state.ahrs_data.euler[2] * RAD_TO_DEG;
      payload[4] = (float)pkt.abc_state.motor_state.rpm_L;
      payload[5] = (float)pkt.abc_state.motor_state.rpm_R;
      payload[6] = (float)pkt.abc_state.motor_state.pwm_out_L / (float)MAX_PWM_DUTY;
      payload[7] = (float)pkt.abc_state.motor_state.pwm_out_R / (float)MAX_PWM_DUTY;
      payload[8] = pkt.abc_state.velocity;
      payload[9] = pkt.abc_state.ahrs_data.imu_data.gyro[0] * RAD_TO_DEG;
      payload[10] = pkt.abc_state.ahrs_data.imu_data.gyro[1] * RAD_TO_DEG;
      payload[11] = pkt.abc_state.ahrs_data.imu_data.gyro[2] * RAD_TO_DEG;
      payload[12] = pkt.abc_state.ahrs_data.imu_data.accl[0];
      payload[13] = pkt.abc_state.ahrs_data.imu_data.accl[1];
      payload[14] = pkt.abc_state.ahrs_data.imu_data.accl[2];
      payload[15] = (float)pkt.cmd.mode;
      payload[16] = (float)pkt.delay_count;
      payload[17] = pkt.battery_v;
      payload[18] = pkt.mag_data.mag[0];
      payload[19] = pkt.mag_data.mag[1];
      payload[20] = pkt.mag_data.mag[2];
      payload[21] = pkt.baro_data.pressure;
      payload[22] = pkt.baro_data.temperature;

      // Calculate CRC-16 over payload
      uint16_t crc = calculate_crc16((const uint8_t*)payload, sizeof(payload));

      // Construct and send packet: Header + ID + Payload + CRC + Tail
      uint8_t send_buffer[sizeof(packet_header) + 1 + sizeof(payload) + sizeof(crc) + sizeof(packet_tail)];
      size_t offset = 0;
      
      memcpy(send_buffer + offset, packet_header, sizeof(packet_header));
      offset += sizeof(packet_header);

      send_buffer[offset++] = MSG_ID_DEFAULT;
      
      memcpy(send_buffer + offset, payload, sizeof(payload));
      offset += sizeof(payload);
      
      memcpy(send_buffer + offset, &crc, sizeof(crc));
      offset += sizeof(crc);
      
      memcpy(send_buffer + offset, packet_tail, sizeof(packet_tail));
      offset += sizeof(packet_tail);

      port.write(send_buffer, sizeof(send_buffer));
    } 
    else if (_format == FORMAT_PID) {
      float payload[LEN_PAYLOAD_PID];
      payload[0] = pkt.abc_state.ahrs_data.imu_data.timestamp * 1e-6f;
      payload[1] = pkt.cmd.target_value;
      payload[2] = pkt.pid_target.rpm_L;
      payload[3] = pkt.pid_target.rpm_R;
      payload[4] = pkt.pid_target.pitch * RAD_TO_DEG;
      payload[5] = pkt.pid_target.velocity;
      payload[6] = pkt.pid_target.steer_rpm;
      payload[7] = (float)pkt.abc_state.motor_state.pwm_out_L;
      payload[8] = (float)pkt.abc_state.motor_state.pwm_out_R;
      payload[9] = (float)pkt.abc_state.motor_state.rpm_L;
      payload[10] = (float)pkt.abc_state.motor_state.rpm_R;
      payload[11] = pkt.abc_state.ahrs_data.euler[0] * RAD_TO_DEG;
      payload[12] = pkt.abc_state.velocity;
      payload[13] = pkt.abc_state.ahrs_data.imu_data_calibrated.gyro[2] * RAD_TO_DEG;
      payload[14] = pkt.battery_v;

      // Calculate CRC-16 over payload
      uint16_t crc = calculate_crc16((const uint8_t*)payload, sizeof(payload));

      // Construct and send packet: Header + ID + Payload + CRC + Tail
      uint8_t send_buffer[sizeof(packet_header) + 1 + sizeof(payload) + sizeof(crc) + sizeof(packet_tail)];
      size_t offset = 0;
      
      memcpy(send_buffer + offset, packet_header, sizeof(packet_header));
      offset += sizeof(packet_header);
      
      send_buffer[offset++] = MSG_ID_PID;

      memcpy(send_buffer + offset, payload, sizeof(payload));
      offset += sizeof(payload);
      
      memcpy(send_buffer + offset, &crc, sizeof(crc));
      offset += sizeof(crc);
      
      memcpy(send_buffer + offset, packet_tail, sizeof(packet_tail));
      offset += sizeof(packet_tail);

      port.write(send_buffer, sizeof(send_buffer));
    }
    else if (_format == FORMAT_LIDAR) {
      float payload[LEN_PAYLOAD_DEFAULT];
      payload[0] = pkt.abc_state.ahrs_data.imu_data.timestamp * 1e-6f;
      payload[1] = pkt.abc_state.ahrs_data.euler[0] * RAD_TO_DEG;
      payload[2] = pkt.abc_state.ahrs_data.euler[1] * RAD_TO_DEG;
      payload[3] = pkt.abc_state.ahrs_data.euler[2] * RAD_TO_DEG;
      payload[4] = (float)pkt.abc_state.motor_state.rpm_L;
      payload[5] = (float)pkt.abc_state.motor_state.rpm_R;
      payload[6] = (float)pkt.abc_state.motor_state.pwm_out_L / (float)MAX_PWM_DUTY;
      payload[7] = (float)pkt.abc_state.motor_state.pwm_out_R / (float)MAX_PWM_DUTY;
      payload[8] = pkt.abc_state.velocity;
      payload[9] = pkt.abc_state.ahrs_data.imu_data.gyro[0] * RAD_TO_DEG;
      payload[10] = pkt.abc_state.ahrs_data.imu_data.gyro[1] * RAD_TO_DEG;
      payload[11] = pkt.abc_state.ahrs_data.imu_data.gyro[2] * RAD_TO_DEG;
      payload[12] = pkt.abc_state.ahrs_data.imu_data.accl[0];
      payload[13] = pkt.abc_state.ahrs_data.imu_data.accl[1];
      payload[14] = pkt.abc_state.ahrs_data.imu_data.accl[2];
      payload[15] = (float)pkt.cmd.mode;
      payload[16] = (float)pkt.delay_count;
      payload[17] = pkt.battery_v;
      payload[18] = pkt.mag_data.mag[0];
      payload[19] = pkt.mag_data.mag[1];
      payload[20] = pkt.mag_data.mag[2];
      payload[21] = pkt.baro_data.pressure;
      payload[22] = pkt.baro_data.temperature;

      // Calculate CRC-16 over payload
      uint16_t crc = calculate_crc16((const uint8_t*)payload, sizeof(payload));

      // Construct and send packet: Header + ID + Payload + CRC + Tail
      uint8_t send_buffer[sizeof(packet_header) + 1 + sizeof(payload) + sizeof(crc) + sizeof(packet_tail)];
      size_t offset = 0;
      
      memcpy(send_buffer + offset, packet_header, sizeof(packet_header));
      offset += sizeof(packet_header);
      
      send_buffer[offset++] = MSG_ID_DEFAULT;

      memcpy(send_buffer + offset, payload, sizeof(payload));
      offset += sizeof(payload);
      
      memcpy(send_buffer + offset, &crc, sizeof(crc));
      offset += sizeof(crc);
      
      memcpy(send_buffer + offset, packet_tail, sizeof(packet_tail));
      offset += sizeof(packet_tail);
      
      bool has_lidar = (_lidar_updated && _lidar_mutex && _lidar_scan);
      
      uint16_t pt_count = 0;
      if (has_lidar) {
          if (xSemaphoreTake(_lidar_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
              pt_count = _lidar_scan->count;
              if (pt_count > MAX_LIDAR_POINTS) {
                  pt_count = MAX_LIDAR_POINTS;
              }
          } else {
              has_lidar = false; 
          }
      }

      if (!has_lidar) {
        port.write(send_buffer, offset);
        return;
      }

      // LwIP Automatic IP Fragmentation: Allocate a single buffer large enough for a full scan (approx 4.7 KB)
      static uint8_t* tx_chunk = nullptr;
      const size_t max_buf_size = sizeof(send_buffer) + sizeof(packet_header) + 1 + 2 + MAX_LIDAR_POINTS * sizeof(lidar_point_packed_t) + 2 + sizeof(packet_tail);
      if (!tx_chunk) {
          tx_chunk = (uint8_t*)heap_caps_malloc(max_buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
          if (!tx_chunk) {
              Serial.println("[TELE] Failed to allocate large PSRAM buffer for Lidar UDP!");
          }
      }
      
      if (tx_chunk) {
          offset = 0;
          
          // 1. Payload: default_payload
          memcpy(tx_chunk + offset, send_buffer, sizeof(send_buffer));
          offset += sizeof(send_buffer);
          
          size_t lidar_payload_start = offset;  // mark start of lidar sub-packet
          
          // 2. Lidar Header + ID + Point Count
          memcpy(tx_chunk + offset, packet_header, sizeof(packet_header));
          offset += sizeof(packet_header);

          tx_chunk[offset++] = MSG_ID_LIDAR;

          memcpy(tx_chunk + offset, &pt_count, sizeof(pt_count));
          offset += sizeof(pt_count);

          // 3. Lidar Points
          for (int i = 0; i < pt_count; i++) {
              lidar_point_packed_t packed_pt;
              packed_pt.x = (int16_t)(_lidar_scan->points[i].x * 1000.0f);
              packed_pt.y = (int16_t)(_lidar_scan->points[i].y * 1000.0f);
              packed_pt.distance = (uint16_t)(_lidar_scan->points[i].distance * 1000.0f);
              packed_pt.angle = (uint16_t)(_lidar_scan->points[i].angle * (65535.0f / 360.0f));
              packed_pt.intensity = _lidar_scan->points[i].intensity;
              
              memcpy(tx_chunk + offset, &packed_pt, sizeof(lidar_point_packed_t)); 
              offset += sizeof(lidar_point_packed_t);
          }
          _lidar_updated = false;
          xSemaphoreGive(_lidar_mutex);

          // 4. CRC-16 (over lidar_payload)
          uint16_t crc = calculate_crc16(tx_chunk + lidar_payload_start, offset - lidar_payload_start);
          memcpy(tx_chunk + offset, &crc, sizeof(crc));
          offset += sizeof(crc);

          // 5. Tail
          memcpy(tx_chunk + offset, packet_tail, sizeof(packet_tail));
          offset += sizeof(packet_tail);
          port.write(tx_chunk, offset);
      } else {
          _lidar_updated = false;
          xSemaphoreGive(_lidar_mutex);
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
