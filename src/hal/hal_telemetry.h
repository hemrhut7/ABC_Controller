#pragma once

#include <Arduino.h>
#include "ring_buffer.h"
#include "hal_type_define.h"


#define TELEMETRY_BUFFER_SIZE 1024

class Telemetry {
public:
    Telemetry(HardwareSerial& serial);
    void init();
    void queue_vofa_data(ABC_state_t &ahrs_data);
    void send_data();

private:
    RingBuffer_t rb;
    uint8_t buffer[TELEMETRY_BUFFER_SIZE];
    HardwareSerial& serial_port;
};