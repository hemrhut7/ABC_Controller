#include "hal_telemetry.h"
#include "hal_motor.h"
#include "hal_microros.h"
#include <cstring>
#include <math.h>

extern HAL_MicroROS uros_telemetry;

// Telemetry packet protocol constants
static const uint8_t packet_header[2] = {0xAA, 0x55};

enum MSG_ID {
    MSG_ID_DEFAULT = 0x01,
    MSG_ID_PID     = 0x02,
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
}

 void Telemetry::init(uint16_t base_freq) {
  _base_freq = base_freq;
  data_queue = xQueueCreate(TELEMETRY_QUEUE_LENGTH, sizeof(system_state_t));
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

void Telemetry::set_config(bool enabled, uint8_t format, uint16_t freq_hz) {
  _enabled = enabled;
  _format = format;
  _packet_counter = 0; // Reset counter on config change
  _tx_packet_counter = 0;

  if (freq_hz == 0) {
    _divider = 1;
  } else {
    _divider = _base_freq / freq_hz;
    if (_divider == 0)
      _divider = 1;
  }
  
  // Memory allocation/deallocation is handled in init() and destructor to avoid race conditions
}

bool Telemetry::is_transmitting() const {
  return _enabled && !uros_telemetry.is_connected();
}

void Telemetry::process_serial_outgoing() {
  if (!_enabled) return;

  // Global Mutual exclusion: Pause ALL Telemetry transmission whenever micro-ROS is connected
  if (uros_telemetry.is_connected()) {
    // Clear/drain queues to prevent stale accumulation while paused
    if (data_queue != nullptr) {
      system_state_t dummy;
      while (xQueueReceive(data_queue, &dummy, 0) == pdTRUE) {}
    }
    return;
  }

  uint8_t tx_buffer[512];
  size_t total_len = 0;

  // 1. Process Default or PID packet from data_queue
  system_state_t pkt;
  if (data_queue != nullptr && xQueueReceive(data_queue, &pkt, 0) == pdTRUE) {
    if (++_packet_counter % _divider == 0) {
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

        memcpy(tx_buffer + total_len, packet_header, sizeof(packet_header));
        total_len += sizeof(packet_header);

        tx_buffer[total_len++] = MSG_ID_DEFAULT;
        tx_buffer[total_len++] = _tx_packet_counter++;

        memcpy(tx_buffer + total_len, payload, sizeof(payload));
        total_len += sizeof(payload);

        uint16_t crc = calculate_crc16(tx_buffer + sizeof(packet_header) + 1, 1 + sizeof(payload));
        memcpy(tx_buffer + total_len, &crc, sizeof(crc));
        total_len += sizeof(crc);
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

        memcpy(tx_buffer + total_len, packet_header, sizeof(packet_header));
        total_len += sizeof(packet_header);

        tx_buffer[total_len++] = MSG_ID_PID;
        tx_buffer[total_len++] = _tx_packet_counter++;

        memcpy(tx_buffer + total_len, payload, sizeof(payload));
        total_len += sizeof(payload);

        uint16_t crc = calculate_crc16(tx_buffer + sizeof(packet_header) + 1, 1 + sizeof(payload));
        memcpy(tx_buffer + total_len, &crc, sizeof(crc));
        total_len += sizeof(crc);
      }
    }
  }

  // 2. ATOMIC SINGLE WRITE for all queued packets combined!
  if (port && total_len > 0) {
    size_t pass_bytes = port->write(tx_buffer, total_len);
  }
}

void Telemetry::queue_string(const char *str) {
  if (!str) return;
  if (port) port->print(str);
}

void Telemetry::queue_string(const String &str) { 
  queue_string(str.c_str()); 
}

