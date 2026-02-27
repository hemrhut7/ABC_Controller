#include "hal_telemetry.h"
#include <math.h>
#include <cstring>

// VOFA+ frame tail
const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};

#define TELEMETRY_QUEUE_LENGTH 5

Telemetry::Telemetry(Stream& stream) : port(stream) {
}

void Telemetry::init() {
    data_queue = xQueueCreate(TELEMETRY_QUEUE_LENGTH, sizeof(system_state_t));
}

void Telemetry::push_data(const system_state_t& packet) {
    // If the queue is full, the oldest data will be overwritten.
    // This is crucial to ensure the high-frequency control loop is never blocked.
    if (data_queue != NULL) {
        xQueueSend(data_queue, &packet, 0);
    }
}

void Telemetry::process_serial_outgoing() {
    system_state_t pkt;

    // Block and wait indefinitely for a packet to arrive in the queue.
    if (xQueueReceive(data_queue, &pkt, portMAX_DELAY) == pdTRUE) {
        // A packet was successfully received. 
        // Now, manually flatten the nested struct into a float array for VOFA+.
        float data_packet[15];
        data_packet[0] = pkt.abc_state.ahrs_data.imu_data.timestamp * 1e-6f;
        data_packet[1] = pkt.abc_state.ahrs_data.euler[0] * RAD_TO_DEG;
        data_packet[2] = pkt.abc_state.ahrs_data.euler[1] * RAD_TO_DEG;
        data_packet[3] = pkt.abc_state.ahrs_data.euler[2] * RAD_TO_DEG;
        data_packet[4] = (float)pkt.abc_state.motor_state.rpm_L;
        data_packet[5] = (float)pkt.abc_state.motor_state.rpm_R;
        data_packet[6] = pkt.abc_state.ahrs_data.imu_data.gyro[0] * RAD_TO_DEG;
        data_packet[7] = pkt.abc_state.ahrs_data.imu_data.gyro[1] * RAD_TO_DEG;
        data_packet[8] = pkt.abc_state.ahrs_data.imu_data.gyro[2] * RAD_TO_DEG;
        data_packet[9] = pkt.abc_state.ahrs_data.imu_data.accl[0];
        data_packet[10] = pkt.abc_state.ahrs_data.imu_data.accl[1];
        data_packet[11] = pkt.abc_state.ahrs_data.imu_data.accl[2];
        data_packet[12] = pkt.target_val;
        data_packet[13] = (float)pkt.loop_time_ms;
        data_packet[14] = (float)pkt.mode;

        // Write the data packet and the tail to the serial port.
        uint8_t send_buffer[sizeof(data_packet) + sizeof(vofa_tail)];
        memcpy(send_buffer, data_packet, sizeof(data_packet));
        memcpy(send_buffer + sizeof(data_packet), vofa_tail, sizeof(vofa_tail));
        port.write(send_buffer, sizeof(send_buffer));
    }
}

void Telemetry::process_bt_outgoing() {
    system_state_t pkt;
    
    if (xQueueReceive(data_queue, &pkt, portMAX_DELAY) == pdTRUE) {
        // A packet was successfully received. 
        // Now, manually flatten the nested struct into a float array for VOFA+.
        float data_packet[14];
        data_packet[0] = pkt.abc_state.ahrs_data.imu_data.timestamp * 1e-6f;
        data_packet[1] = pkt.target_val;
        data_packet[2] = pkt.pid_target.rpm_L;
        data_packet[3] = pkt.pid_target.rpm_R;
        data_packet[4] = pkt.pid_target.pitch * RAD_TO_DEG;
        data_packet[5] = pkt.pid_target.velocity;
        data_packet[6] = pkt.pid_target.yaw_rate * RAD_TO_DEG;
        data_packet[7] = (float)pkt.abc_state.motor_state.pwm_out_L;
        data_packet[8] = (float)pkt.abc_state.motor_state.pwm_out_R;
        data_packet[9] = (float)pkt.abc_state.motor_state.rpm_L;
        data_packet[10] = (float)pkt.abc_state.motor_state.rpm_R;
        data_packet[11] = pkt.abc_state.ahrs_data.euler[0] * RAD_TO_DEG;
        data_packet[12] = pkt.abc_state.velocity;
        data_packet[13] = pkt.abc_state.ahrs_data.imu_data_calibrated.gyro[2] * RAD_TO_DEG;

        // Write the data packet and the tail to the serial port.
        uint8_t send_buffer[sizeof(data_packet) + sizeof(vofa_tail)];
        memcpy(send_buffer, data_packet, sizeof(data_packet));
        memcpy(send_buffer + sizeof(data_packet), vofa_tail, sizeof(vofa_tail));
        port.write(send_buffer, sizeof(send_buffer));
    }
}

void Telemetry::queue_string(const char* str) {
    if (!str) return;
    // This function is called from a communication task, so direct writing is safe
    // and won't interfere with the Control_Task.
    port.print(str);
}

void Telemetry::queue_string(const String& str) {
    queue_string(str.c_str());
}
