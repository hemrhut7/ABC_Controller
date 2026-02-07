#pragma once

#include "ring_buffer.h"
#include <Arduino.h>

#define TELEMETRY_BUFFER_SIZE 1024

class Telemetry {
public:
    Telemetry(HardwareSerial& serial);
    void init();
    void queue_vofa_data(float euler[3], int rpm_L, int target_rpm_L, int rpm_R, int target_rpm_R, float gyro[3], float accl[3]);
    void send_data();

private:
    RingBuffer_t rb;
    uint8_t buffer[TELEMETRY_BUFFER_SIZE];
    HardwareSerial& serial_port;
};