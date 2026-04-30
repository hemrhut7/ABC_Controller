#include "hal_external_sensor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/stream_buffer.h>
#include "driver/uart.h"
#include <string.h>

#define EX_UART_NUM UART_NUM_2
#define EX_UART_TX_PIN 17
#define EX_UART_RX_PIN 16
#define EX_UART_BAUD 921600
#define EX_UART_BUF_SIZE 2048

static QueueHandle_t imu_queue = NULL;
static QueueHandle_t mag_queue = NULL;
static QueueHandle_t baro_queue = NULL;
static StreamBufferHandle_t sensor_stream_buf = NULL;

static bool is_imu_healthy = false;
static bool is_mag_healthy = false;
static bool is_baro_healthy = false;

static uint32_t last_imu_time = 0;
static uint32_t last_mag_time = 0;
static uint32_t last_baro_time = 0;

static volatile bool is_bypass_enabled = false;

void hal_external_sensor_init() {
    if (imu_queue == NULL) imu_queue = xQueueCreate(1, sizeof(imu_data_t));
    if (mag_queue == NULL) mag_queue = xQueueCreate(1, sizeof(float) * 3);
    if (baro_queue == NULL) baro_queue = xQueueCreate(1, sizeof(float));
    if (sensor_stream_buf == NULL) sensor_stream_buf = xStreamBufferCreate(512, 1);

    const uart_config_t uart_config = {
        .baud_rate = EX_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(EX_UART_NUM, EX_UART_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(EX_UART_NUM, &uart_config);
    uart_set_pin(EX_UART_NUM, EX_UART_TX_PIN, EX_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

bool hal_external_sensor_healthy() {
    return is_imu_healthy && (millis() - last_imu_time < 500);
}

bool hal_external_sensor_mag_healthy() {
    return is_mag_healthy && (millis() - last_mag_time < 500);
}

bool hal_external_sensor_baro_healthy() {
    return is_baro_healthy && (millis() - last_baro_time < 500);
}

void hal_external_sensor_read(imu_data_t *data) {
    if (imu_queue != NULL) xQueuePeek(imu_queue, data, 0);
}

void hal_external_sensor_read_mag(float *mag) {
    if (mag_queue != NULL) xQueuePeek(mag_queue, mag, 0);
}

void hal_external_sensor_read_baro(float *baro) {
    if (baro_queue != NULL) xQueuePeek(baro_queue, baro, 0);
}

void hal_external_sensor_set_bypass(bool enable) {
    is_bypass_enabled = enable;
}

StreamBufferHandle_t hal_external_sensor_get_streambuffer() {
    return sensor_stream_buf;
}

void hal_external_sensor_task(void *pvParameters) {
    (void)pvParameters;
    uint8_t buffer[256];
    size_t length = 0;

    for (;;) {
        int read_len = uart_read_bytes(EX_UART_NUM, buffer + length, sizeof(buffer) - length, pdMS_TO_TICKS(5));
        if (read_len > 0) {
            length += read_len;
        }

        bool packet_processed = true;
        while (packet_processed && length >= 12) {
            packet_processed = false;
            for (size_t i = 0; i <= length - 12; i++) {
                // Peek potential type
                float type;
                memcpy(&type, buffer + i, 4);

                size_t packet_size = 0;
                if (type == 1.0f) packet_size = sizeof(IMUPacket);
                else if (type == 2.0f) packet_size = sizeof(MagPacket);
                else if (type == 3.0f) packet_size = sizeof(BaroPacket);

                if (packet_size > 0 && (i + packet_size) <= length) {
                    // Check tail
                    uint32_t tail;
                    memcpy(&tail, buffer + i + packet_size - 4, 4);
                    if (tail == 0x7f800000) {
                        // Bypass raw packet if enabled
                        if (is_bypass_enabled && sensor_stream_buf != NULL) {
                            xStreamBufferSend(sensor_stream_buf, buffer + i, packet_size, 0);
                        }

                        // Process locally
                        if (type == 1.0f) {
                            IMUPacket *p = (IMUPacket *)(buffer + i);
                            imu_data_t imu;
                            imu.timestamp = micros();
                            memcpy(imu.accl, p->accel, sizeof(imu.accl));
                            memcpy(imu.gyro, p->gyro, sizeof(imu.gyro));
                            if (imu_queue) xQueueOverwrite(imu_queue, &imu);
                            last_imu_time = millis();
                            is_imu_healthy = true;
                        } else if (type == 2.0f) {
                            MagPacket *p = (MagPacket *)(buffer + i);
                            if (mag_queue) xQueueOverwrite(mag_queue, p->mag);
                            last_mag_time = millis();
                            is_mag_healthy = true;
                        } else if (type == 3.0f) {
                            BaroPacket *p = (BaroPacket *)(buffer + i);
                            if (baro_queue) xQueueOverwrite(baro_queue, &p->press);
                            last_baro_time = millis();
                            is_baro_healthy = true;
                        }

                        // Remove from buffer
                        memmove(buffer, buffer + i + packet_size, length - (i + packet_size));
                        length -= (i + packet_size);
                        packet_processed = true;
                        break;
                    }
                }
            }
            
            // If no packet found in a reasonably full buffer, discard the oldest data to find alignment
            if (!packet_processed && length > 128) {
                size_t discard_len = length - 128;
                memmove(buffer, buffer + discard_len, length - discard_len);
                length -= discard_len;
                packet_processed = true; // Re-evaluate the new shifted buffer
            }
        }
    }
}
