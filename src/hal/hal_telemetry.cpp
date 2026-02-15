#include "hal_telemetry.h"
#include <math.h>

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
        data_packet[0] = pkt.abc_state.ahrs_data.imu_data.timestamp;
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
        port.write((uint8_t*)data_packet, sizeof(data_packet));
        port.write(vofa_tail, sizeof(vofa_tail));
    }
}

void Telemetry::process_bt_outgoing() {
    system_state_t pkt;
    
    if (xQueueReceive(data_queue, &pkt, portMAX_DELAY) == pdTRUE) {
        // A packet was successfully received. 
        // Now, manually flatten the nested struct into a float array for VOFA+.
        float data_packet[4];
        data_packet[0] = pkt.abc_state.ahrs_data.imu_data.timestamp;
        data_packet[1] = pkt.target_val;

        float val3 = NAN;
        float val4 = NAN;

        switch (pkt.mode) {
            case MODE_MOTOR_TEST:
                val3 = (float)pkt.abc_state.motor_state.rpm_L;
                val4 = (float)pkt.abc_state.motor_state.rpm_R;
                break;
            case MODE_RATE:
                val3 = pkt.abc_state.ahrs_data.imu_data.gyro[0];
                break;
            case MODE_ANGLE:
                val3 = pkt.abc_state.ahrs_data.euler[0];
                break;
            case MODE_VELOCITY:
                val3 = pkt.abc_state.velocity;
                break;
            case MODE_REMOTE:
                val3 = pkt.abc_state.ahrs_data.imu_data.gyro[2];
                break;
            default:
                break;
        }

        data_packet[2] = val3;
        data_packet[3] = val4;

        // Write the data packet and the tail to the serial port.
        port.write((uint8_t*)data_packet, sizeof(data_packet));
        port.write(vofa_tail, sizeof(vofa_tail));
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
