#pragma once

#include "hal_type_define.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#pragma pack(push, 1)
typedef struct {
    uint16_t distance; // mm
    uint16_t angle;    // 0-65535 for 0-360 deg
    uint8_t intensity;
} lidar_point_packed_t;
#pragma pack(pop)


enum TelemetryFormat {
  FORMAT_DEFAULT = 0,
  FORMAT_PID = 1,
  FORMAT_LIDAR = 2,
};

typedef enum TelemetryPort {
  PORT_USB = 0,
  PORT_BT = 1,
  PORT_WIFI = 2,
  PORT_UART1 = 3,
  PORT_UART2 = 4,
} TelemetryPort_t;


class Telemetry {
public:
  static Telemetry& getInstance();

  // Disable copy constructor and assignment operator to enforce Singleton pattern
  Telemetry(const Telemetry&) = delete;
  Telemetry& operator=(const Telemetry&) = delete;

  // Dynamically switch/set the output stream and port ID
  void set_port(Stream &stream, TelemetryPort_t port_id);

  void init(uint16_t base_freq = 200);

  // Called by Control_Task to push data into the queue (non-blocking).
  void push_data(const system_state_t &packet);
  // Update lidar data for FORMAT_LIDAR
  void push_lidar_data(const lidar_scan_t &scan);

  // Dynamic configuration
  void set_config(bool enabled, uint8_t format, uint16_t freq_hz);

  // Called by a dedicated task to process and send data from the queue
  // (blocking).
  void process_serial_outgoing();

  // For logging strings, can be handled separately or integrated if needed.
  void queue_string(const char *str);
  void queue_string(const String &str);

  bool connected() const {return _enabled;};
  bool is_transmitting() const;
  TelemetryPort_t get_port_id() const { return port_id; };

private:
  // Private constructor and destructor to ensure single instance
  Telemetry();
  ~Telemetry();

  QueueHandle_t data_queue = nullptr;
  QueueHandle_t lidar_queue = nullptr;

  Stream *port = nullptr;
  TelemetryPort_t port_id = PORT_USB;

  // Config
  bool _enabled = false;
  uint16_t _base_freq = 200;
  uint8_t _divider = 1;
  uint8_t _format = 0;
  uint32_t _packet_counter = 0;
  uint8_t _tx_packet_counter = 0;
  uint8_t _tx_lidar_packet_counter = 0;
  uint64_t dt_lpf = 0;
  uint64_t dt_lpf2 = 0;
  uint32_t uart_failed_counter = 0;
};