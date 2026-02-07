#include "hal_telemetry.h"

// VOFA+ frame tail
const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};

Telemetry::Telemetry(HardwareSerial& serial) : serial_port(serial) {
}

void Telemetry::init() {
    RingBuffer_Init(&rb, buffer, TELEMETRY_BUFFER_SIZE);
}

void Telemetry::queue_vofa_data(float euler[3], int rpm_L, int target_rpm_L, int rpm_R, int target_rpm_R, float gyro[3], float accl[3]) {
    // Pack data as floats for VOFA+
    float data_packet[14];
    float now = millis() * 1e-3f;
    data_packet[0] = now;
    data_packet[1] = euler[0];
    data_packet[2] = euler[1];
    data_packet[3] = euler[2];
    data_packet[4] = (float)rpm_L;
    data_packet[5] = (float)target_rpm_L;
    data_packet[6] = (float)rpm_R;
    data_packet[7] = (float)target_rpm_R;
    data_packet[8] = gyro[0];
    data_packet[9] = gyro[1];
    data_packet[10] = gyro[2];
    data_packet[11] = accl[0];
    data_packet[12] = accl[1];
    data_packet[13] = accl[2];

    RingBuffer_Write(&rb, (uint8_t*)data_packet, sizeof(data_packet));
    RingBuffer_Write(&rb, vofa_tail, sizeof(vofa_tail));
}

void Telemetry::send_data() {
    size_t data_len = RingBuffer_GetDataLength(&rb);
    if (data_len > 0) {
        uint8_t temp_buffer[256]; // Send in chunks
        size_t to_read = data_len > sizeof(temp_buffer) ? sizeof(temp_buffer) : data_len;
        
        if (RingBuffer_Read(&rb, temp_buffer, to_read)) {
            serial_port.write(temp_buffer, to_read);
        }
    }
}
