#pragma once

#include "hal_type_define.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>


enum TelemetryFormat {
  FORMAT_DEFAULT = 0,
  FORMAT_PID = 1,
  FORMAT_IMU = 2,
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
  Telemetry(Stream &stream, TelemetryPort_t port_id);
  void init(uint16_t base_freq = 200);

  // Called by Control_Task to push data into the queue (non-blocking).
  void push_data(const system_state_t &packet);

  // Dynamic configuration
  void set_config(bool enabled, uint8_t format, uint16_t freq_hz);

  // Called by a dedicated task to process and send data from the queue
  // (blocking).
  void process_serial_outgoing();

  // For logging strings, can be handled separately or integrated if needed.
  void queue_string(const char *str);
  void queue_string(const String &str);

  TelemetryPort_t get_port_id() const { return port_id; };

private:
  QueueHandle_t data_queue;
  Stream &port;
  TelemetryPort_t port_id = PORT_USB;

  // Config
  bool _enabled = false;
  uint16_t _base_freq = 200;
  uint8_t _divider = 1;
  uint8_t _format = 0;
  uint32_t _packet_counter = 0;
};