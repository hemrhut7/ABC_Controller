#pragma once

#include <Arduino.h>
#include "hal_type_define.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class Telemetry {
public:
    Telemetry(Stream& stream);
    void init();
    
    // Called by Control_Task to push data into the queue (non-blocking).
    void push_data(const system_state_t& packet);

    // Called by a dedicated task to process and send data from the queue (blocking).
    void process_serial_outgoing();
    void process_bt_outgoing();
    
    // For logging strings, can be handled separately or integrated if needed.
    void queue_string(const char* str);
    void queue_string(const String& str);

private:
    QueueHandle_t data_queue;
    Stream& port;
};