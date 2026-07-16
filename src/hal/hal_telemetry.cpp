#include "hal_telemetry.h"
#include "hal_motor.h"
#include <cstring>
#include <math.h>

// Telemetry packet protocol constants
static const uint8_t packet_header[2] = {0xAA, 0x55};
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
#define LIDAR_QUEUE_LENGTH 5

#define LEN_PAYLOAD_DEFAULT 23
#define LEN_PAYLOAD_PID 15
#define LEN_PAYLOAD_LIDAR (MAX_LIDAR_POINTS * sizeof(lidar_point_packed_t) + sizeof(uint32_t)) // Timestamp + points


Telemetry& Telemetry::getInstance() {
  static Telemetry instance;
  return instance;
}

Telemetry::Telemetry() : port(nullptr), port_id(PORT_USB) {}

void Telemetry::set_port(Stream &stream, TelemetryPort_t port_id) {
  this->port = &stream;
  this->port_id = port_id;
}
 
Telemetry::~Telemetry() {
  if (data_queue) {
    vQueueDelete(data_queue);
    data_queue = nullptr;
  }
  if (lidar_queue) {
    vQueueDelete(lidar_queue);
    lidar_queue = nullptr;
  }
}

 void Telemetry::init(uint16_t base_freq) {
  _base_freq = base_freq;
  data_queue = xQueueCreate(TELEMETRY_QUEUE_LENGTH, sizeof(system_state_t));
  lidar_queue = xQueueCreate(LIDAR_QUEUE_LENGTH, sizeof(lidar_scan_t));
}

void Telemetry::push_data(const system_state_t &packet) {
  // If the queue is full, the oldest data will be overwritten.
  // This is crucial to ensure the high-frequency control loop is never blocked.
  if (data_queue == nullptr) return;

  if (xQueueSend(data_queue, &packet, 0) != pdTRUE) {
    system_state_t dummy;
    xQueueReceive(data_queue, &dummy, 0);
    xQueueSend(data_queue, &packet, 0);
  }
}

void Telemetry::push_lidar_data(const lidar_scan_t &scan) {
  if (lidar_queue == nullptr || _format != FORMAT_LIDAR) return;

  if (xQueueSend(lidar_queue, &scan, 0) != pdTRUE) {
    lidar_scan_t dummy;
    xQueueReceive(lidar_queue, &dummy, 0);
    xQueueSend(lidar_queue, &scan, 0);
  }
}

void Telemetry::set_config(bool enabled, uint8_t format, uint16_t freq_hz) {
  _enabled = enabled;
  _format = format;
  _packet_counter = 0; // Reset counter on config change
  _tx_packet_counter = 0;
  _tx_lidar_packet_counter = 0;

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

  if (data_queue != nullptr && xQueueReceive(data_queue, &pkt, 0) == pdTRUE) {
    if (!_enabled) return;
    if (++_packet_counter % _divider != 0) return;

    if (_format == FORMAT_DEFAULT || _format == FORMAT_LIDAR) {
      uint64_t start = micros();
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
      // payload[15] = (float)pkt.cmd.mode;
      payload[15] = (float)dt_lpf / 1000.0f;
      // payload[16] = (float)pkt.delay_count;
      // payload[16] = (float)uart_failed_counter;
      // payload[16] = (float)dt_lpf / 1000.0f + (float)dt_lpf2 / 1000.0f; // Convert to milliseconds
      payload[16] = (float)dt_lpf2 / 1000.0f; // Convert to milliseconds
      payload[17] = pkt.battery_v;
      payload[18] = pkt.mag_data.mag[0];
      payload[19] = pkt.mag_data.mag[1];
      payload[20] = pkt.mag_data.mag[2];
      payload[21] = pkt.baro_data.pressure;
      payload[22] = pkt.baro_data.temperature;

      // Construct and send packet: Header + ID + Counter + Payload + CRC + Tail
      uint8_t send_buffer[sizeof(packet_header) + 1 + 1 + sizeof(payload) + sizeof(uint16_t)];
      size_t offset = 0;
      
      memcpy(send_buffer + offset, packet_header, sizeof(packet_header));
      offset += sizeof(packet_header);

      send_buffer[offset++] = MSG_ID_DEFAULT;
      send_buffer[offset++] = _tx_packet_counter;
      
      memcpy(send_buffer + offset, payload, sizeof(payload));
      offset += sizeof(payload);
      
      // Calculate CRC-16 over Counter + Payload
      uint16_t crc = calculate_crc16(send_buffer + sizeof(packet_header) + 1, 1 + sizeof(payload));

      memcpy(send_buffer + offset, &crc, sizeof(crc));
      offset += sizeof(crc);

      if (port) {
        size_t pass_bytes = port->write(send_buffer, sizeof(send_buffer));
        if (pass_bytes != sizeof(send_buffer)) {
          uart_failed_counter++;
        }
        _tx_packet_counter++;
      }
      
      uint64_t dt = micros() - start;
      dt_lpf = (dt_lpf * 0.9f) + (dt * 0.1f);
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

      // Construct and send packet: Header + ID + Counter + Payload + CRC + Tail
      uint8_t send_buffer[sizeof(packet_header) + 1 + 1 + sizeof(payload) + sizeof(uint16_t)];
      size_t offset = 0;
      
      memcpy(send_buffer + offset, packet_header, sizeof(packet_header));
      offset += sizeof(packet_header);
      
      send_buffer[offset++] = MSG_ID_PID;
      send_buffer[offset++] = _tx_packet_counter;

      memcpy(send_buffer + offset, payload, sizeof(payload));
      offset += sizeof(payload);
      
      // Calculate CRC-16 over Counter + Payload
      uint16_t crc = calculate_crc16(send_buffer + sizeof(packet_header) + 1, 1 + sizeof(payload));

      memcpy(send_buffer + offset, &crc, sizeof(crc));
      offset += sizeof(crc);

      if (port) {
        port->write(send_buffer, sizeof(send_buffer));
        _tx_packet_counter++;
      }
    }
  }

  lidar_scan_t lidar_pkt;
  if (_format == FORMAT_LIDAR && lidar_queue != nullptr && xQueueReceive(lidar_queue, &lidar_pkt, 0) == pdTRUE) {
    uint64_t start = micros();
    size_t offset = 0;
    uint8_t send_buffer[sizeof(packet_header) + 1 + 1 + LEN_PAYLOAD_LIDAR + sizeof(uint16_t)];

    // 1. Lidar Header + ID + Counter + Timestamp
    memcpy(send_buffer + offset, packet_header, sizeof(packet_header));
    offset += sizeof(packet_header);

    send_buffer[offset++] = MSG_ID_LIDAR;
    send_buffer[offset++] = _tx_lidar_packet_counter;

    uint32_t time_stampe_ms = (uint32_t)(lidar_pkt.timestamp * 1e-3f);
    memcpy(send_buffer + offset, &time_stampe_ms, sizeof(time_stampe_ms));
    offset += sizeof(time_stampe_ms);

    // 2. Lidar Points
    lidar_point_packed_t* packed_pts = (lidar_point_packed_t*)(send_buffer + offset);
    for (int i = 0; i < MAX_LIDAR_POINTS; i++) {
      packed_pts[i].distance = (uint16_t)(lidar_pkt.points[i].distance * 1000.0f);
      packed_pts[i].angle = (uint16_t)(lidar_pkt.points[i].angle * ANGLE_SCALE);
      packed_pts[i].intensity = lidar_pkt.points[i].intensity;
    }
    offset += MAX_LIDAR_POINTS * sizeof(lidar_point_packed_t);

    // 3. CRC-16 (over Counter + lidar_payload)
    uint16_t crc = calculate_crc16(send_buffer + sizeof(packet_header) + 1, offset - (sizeof(packet_header) + 1));
    memcpy(send_buffer + offset, &crc, sizeof(crc));
    offset += sizeof(crc);

    if (port) {
      size_t pass_bytes = port->write(send_buffer, offset);
      if (pass_bytes != sizeof(send_buffer)) {
        uart_failed_counter++;
      }
      _tx_lidar_packet_counter++;
    }
    
    uint64_t dt = micros() - start;
    dt_lpf2 = (dt_lpf2 * 0.9f) + (dt * 0.1f);
  }
}

void Telemetry::queue_string(const char *str) {
  if (!str) return;
  if (port) port->print(str);
}

void Telemetry::queue_string(const String &str) { 
  queue_string(str.c_str()); 
}
