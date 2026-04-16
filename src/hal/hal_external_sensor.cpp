#include "hal_external_sensor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

static QueueHandle_t imu_queue = NULL;
static bool is_healthy = false;
static uint32_t last_receive_time = 0;

struct __attribute__((packed)) IMUPacket {
    float type;           // Type: 1.0f = IMU
    float accel[3];       // ax, ay, az (m/s^2)
    float gyro[3];        // gx, gy, gz (rad/s)
    uint32_t tail;        // 0x7f800000
};

void hal_external_sensor_init() {
    if (imu_queue == NULL) {
        imu_queue = xQueueCreate(1, sizeof(imu_data_t));
    }
}

bool hal_external_sensor_healthy() {
    return is_healthy && (millis() - last_receive_time < 500);
}

void hal_external_sensor_read(imu_data_t *data) {
    if (imu_queue != NULL) {
        // Use Peek to get the latest data without removing it, 
        // ensuring subsequent reads still get the last known state.
        xQueuePeek(imu_queue, data, 0);
    }
}

void hal_external_sensor_task(void *pvParameters) {
    HardwareSerial* serial = (HardwareSerial*)pvParameters;
    
    // Buffer for reading
    uint8_t buffer[sizeof(IMUPacket)];
    size_t index = 0;
    imu_data_t imu_to_push;

    for (;;) {
        while (serial->available()) {
            buffer[index++] = serial->read();

            if (index == sizeof(IMUPacket)) {
                IMUPacket* packet = (IMUPacket*)buffer;

                // Validate packet tail and type
                if (packet->tail == 0x7f800000 && packet->type == 1.0f) {
                    // Valid packet received
                    imu_to_push.timestamp = micros();
                    
                    // Transform into ABC_Controller coordinate system
                    imu_to_push.accl[0] = packet->accel[0];
                    imu_to_push.accl[1] = packet->accel[1];
                    imu_to_push.accl[2] = packet->accel[2];

                    imu_to_push.gyro[0] = packet->gyro[0];
                    imu_to_push.gyro[1] = packet->gyro[1];
                    imu_to_push.gyro[2] = packet->gyro[2];

                    if (imu_queue != NULL) {
                        xQueueOverwrite(imu_queue, &imu_to_push);
                        last_receive_time = millis();
                        is_healthy = true;
                    }
                    index = 0; // reset for next packet
                } else {
                    // Misaligned, shift buffer by 1 byte to re-align
                    memmove(buffer, buffer + 1, sizeof(IMUPacket) - 1);
                    index--; 
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2)); // Avoid tight loop
    }
}
