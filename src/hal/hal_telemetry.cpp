#include "hal_telemetry.h"


// VOFA+ frame tail
const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};

Telemetry::Telemetry(HardwareSerial& serial) : serial_port(serial) {
}

void Telemetry::init() {
    RingBuffer_Init(&rb, buffer, TELEMETRY_BUFFER_SIZE);
}

void Telemetry::queue_vofa_data(ABC_state_t &abc_state) {
    // Pack data as floats for VOFA+
    float data_packet[14];
    float now = abc_state.ahrs_data.imu_data.timestamp;
    data_packet[0] = now;
    data_packet[1] = abc_state.ahrs_data.euler[0];
    data_packet[2] = abc_state.ahrs_data.euler[1];
    data_packet[3] = abc_state.ahrs_data.euler[2];
    data_packet[4] = (float)abc_state.motor_state.rpm_L;
    data_packet[5] = (float)abc_state.motor_state.target_rpm_L;
    data_packet[6] = (float)abc_state.motor_state.rpm_R;
    data_packet[7] = (float)abc_state.motor_state.target_rpm_R;
    data_packet[8] = abc_state.ahrs_data.imu_data.gyro[0];
    data_packet[9] = abc_state.ahrs_data.imu_data.gyro[1];
    data_packet[10] = abc_state.ahrs_data.imu_data.gyro[2];
    data_packet[11] = abc_state.ahrs_data.imu_data.accl[0];
    data_packet[12] = abc_state.ahrs_data.imu_data.accl[1];
    data_packet[13] = abc_state.ahrs_data.imu_data.accl[2];

    RingBuffer_Write(&rb, (uint8_t*)data_packet, sizeof(data_packet), true);
    RingBuffer_Write(&rb, vofa_tail, sizeof(vofa_tail), true);
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
