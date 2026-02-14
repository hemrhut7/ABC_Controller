#pragma once

#include <Arduino.h>
#include "ring_buffer.h"
#include "hal_type_define.h"


#define TELEMETRY_BUFFER_SIZE 1024

class Telemetry {
public:
    Telemetry(HardwareSerial& serial);
    void init();
    void queue_vofa_data(ABC_state_t &abc_state, uint32_t loop_time_ms);
    void queue_string(const char* str);
    void queue_string(const String& str);
    void send_data();
    void set_target_val(float val) { current_target_val = val; }
    void add_channel(Stream& stream);

private:
    RingBuffer_t rb;
    uint8_t buffer[TELEMETRY_BUFFER_SIZE];
    HardwareSerial& serial_port;
    portMUX_TYPE spinlock;
    float current_target_val = 0;
    Stream* extra_stream = nullptr;
};