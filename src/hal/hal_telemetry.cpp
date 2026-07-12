#include "hal_telemetry.h"
#include "hal_motor.h"
#include <cstring>
#include <math.h>

// Telemetry packet protocol constants
static const uint8_t packet_header[2] = {0xAA, 0x55};
static const uint8_t packet_tail[4]   = {0x00, 0x00, 0x80, 0x7f}; // Keep original tail bytes for compatibility
static const float ANGLE_SCALE = 65535.0f / 360.0f; // Scale factor for angle conversion to uint16_t

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


Telemetry& Telemetry::getInstance() {
  static Telemetry instance;
  return instance;
}

Telemetry::Telemetry() : port(nullptr), port_id(PORT_USB), _tx_chunk(nullptr) {
  for (uint8_t i = 0; i < LIDAR_BUF_COUNT; i++) {
    _lidar_buf[i] = nullptr;
  }
}

void Telemetry::set_port(Stream &stream, TelemetryPort_t port_id) {
  this->port = &stream;
  this->port_id = port_id;
}
 
Telemetry::~Telemetry() {
  for (uint8_t i = 0; i < LIDAR_BUF_COUNT; i++) {
    if (_lidar_buf[i]) {
      free(_lidar_buf[i]);
      _lidar_buf[i] = nullptr;
    }
  }
  if (_tx_chunk) {
    free(_tx_chunk);
    _tx_chunk = nullptr;
  }
  if (data_queue) {
    vQueueDelete(data_queue);
    data_queue = nullptr;
  }
}

 void Telemetry::init(uint16_t base_freq) {
  _base_freq = base_freq;
  data_queue = xQueueCreate(TELEMETRY_QUEUE_LENGTH, sizeof(system_state_t));

  // Pre-allocate Lidar buffers to avoid runtime allocation/deallocation race conditions and fragmentation
  for (uint8_t i = 0; i < LIDAR_BUF_COUNT; i++) {
    if (!_lidar_buf[i]) {
      _lidar_buf[i] = (lidar_scan_t *)heap_caps_malloc(sizeof(lidar_scan_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      if (!_lidar_buf[i]) {
        Serial.printf("[TELE] PSRAM alloc failed for Lidar buffer %u!\n", i);
      }
    }
  }

  // Pre-allocate Lidar transmit chunk
  const size_t max_buf_size = sizeof(packet_header) + 1 + 4 + 2 + MAX_LIDAR_POINTS * sizeof(lidar_point_packed_t) + 2 + sizeof(packet_tail);
  if (!_tx_chunk) {
    _tx_chunk = (uint8_t*)heap_caps_malloc(max_buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!_tx_chunk) {
      Serial.println("[TELE] Failed to allocate large PSRAM buffer for Lidar UDP!");
    }
  }
}

void Telemetry::push_data(const system_state_t &packet) {
  // If the queue is full, the oldest data will be overwritten.
  // This is crucial to ensure the high-frequency control loop is never blocked.
  if (data_queue != NULL) {
    if (xQueueSend(data_queue, &packet, 0) != pdTRUE) {
      system_state_t dummy;
      xQueueReceive(data_queue, &dummy, 0);
      xQueueSend(data_queue, &packet, 0);
    }
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
  
  // Memory allocation/deallocation is handled in init() and destructor to avoid race conditions
}

void Telemetry::process_serial_outgoing() {
  system_state_t pkt;

  if (xQueueReceive(data_queue, &pkt, pdMS_TO_TICKS(1)) == pdTRUE) {
    if (!_enabled) return;

    _packet_counter++;
    if (_packet_counter % _divider != 0) return;

    if (_format == FORMAT_DEFAULT || _format == FORMAT_LIDAR) {
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

      if (port) {
        port->write(send_buffer, sizeof(send_buffer));
      }
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

      if (port) {
        port->write(send_buffer, sizeof(send_buffer));
      }
    }
  
    if (_format == FORMAT_LIDAR) {
      if (!_lidar_updated || !_lidar_buf[0] || !_lidar_buf[1] || !_tx_chunk) {
        return;
      }

      uint8_t read_idx; 
      bool has_data;
      portENTER_CRITICAL(&_lidar_spinlock);
      read_idx = _lidar_ready_idx;
      has_data = _lidar_updated;
      _lidar_updated = false;
      portEXIT_CRITICAL(&_lidar_spinlock);

      if (!has_data) return;
      lidar_scan_t *_lidar_scan = _lidar_buf[read_idx];
      if (!_lidar_scan) return;

      uint16_t pt_count = _lidar_scan->count;
      size_t offset = 0;

      if (pt_count > MAX_LIDAR_POINTS) {
        pt_count = MAX_LIDAR_POINTS;
      }

      // 1. Lidar Header + ID + Point Count
      memcpy(_tx_chunk + offset, packet_header, sizeof(packet_header));
      offset += sizeof(packet_header);

      _tx_chunk[offset++] = MSG_ID_LIDAR;

      uint32_t time_stampe_ms = (uint32_t)(_lidar_scan->timestamp * 1e-3f);
      memcpy(_tx_chunk + offset, &time_stampe_ms, sizeof(time_stampe_ms));
      offset += sizeof(time_stampe_ms);

      memcpy(_tx_chunk + offset, &pt_count, sizeof(pt_count));
      offset += sizeof(pt_count);

      // 2. Lidar Points
      lidar_point_packed_t* packed_pts = (lidar_point_packed_t*)(_tx_chunk + offset);
      for (int i = 0; i < pt_count; i++) {
        packed_pts[i].distance = (uint16_t)(_lidar_scan->points[i].distance * 1000.0f);
        packed_pts[i].angle = (uint16_t)(_lidar_scan->points[i].angle * ANGLE_SCALE);
        packed_pts[i].intensity = _lidar_scan->points[i].intensity;
      }
      offset += pt_count * sizeof(lidar_point_packed_t);

      // 3. CRC-16 (over lidar_payload)
      uint16_t crc = calculate_crc16(_tx_chunk + sizeof(packet_header) + 1, offset - (sizeof(packet_header) + 1));
      memcpy(_tx_chunk + offset, &crc, sizeof(crc));
      offset += sizeof(crc);

      // 4. Tail
      memcpy(_tx_chunk + offset, packet_tail, sizeof(packet_tail));
      offset += sizeof(packet_tail);
      if (port) {
        port->write(_tx_chunk, offset);
      }
      
    }
  }
}

void Telemetry::update_lidar_data(const lidar_scan_t &scan) {
  uint8_t write_idx = 1 - _lidar_ready_idx;
  if (_lidar_buf[write_idx]) {
    *_lidar_buf[write_idx] = scan;              // Copy to offline buffer safely

    portENTER_CRITICAL(&_lidar_spinlock);
    _lidar_ready_idx = write_idx;
    _lidar_updated = true;
    portEXIT_CRITICAL(&_lidar_spinlock);
  }
}

void Telemetry::queue_string(const char *str) {
  if (!str)
    return;
  // This function is called from a communication task, so direct writing is
  // safe and won't interfere with the Control_Task.
  if (port) {
    port->print(str);
  }
}

void Telemetry::queue_string(const String &str) { queue_string(str.c_str()); }
