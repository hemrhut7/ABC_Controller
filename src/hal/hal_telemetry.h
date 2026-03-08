#pragma once

#include "hal_type_define.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class Telemetry {
public:
  Telemetry(Stream &stream);
  void init(uint16_t base_freq = 200);

  // Called by Control_Task to push data into the queue (non-blocking).
  void push_data(const system_state_t &packet);

  // Dynamic configuration
  void set_config(bool enabled, uint8_t format, uint16_t freq_hz);

  // Called by a dedicated task to process and send data from the queue
  // (blocking).
  void process_serial_outgoing();
  void process_bt_outgoing();

  // For logging strings, can be handled separately or integrated if needed.
  void queue_string(const char *str);
  void queue_string(const String &str);

private:
  QueueHandle_t data_queue;
  Stream &port;

  // Config
  bool _enabled = false;
  uint16_t _base_freq = 200;
  uint8_t _divider = 1;
  uint8_t _format = 0;
  uint32_t _packet_counter = 0;
};