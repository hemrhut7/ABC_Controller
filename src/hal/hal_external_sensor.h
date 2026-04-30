#pragma once

#include "hal_type_define.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>

void hal_external_sensor_init();
bool hal_external_sensor_healthy();
bool hal_external_sensor_mag_healthy();
bool hal_external_sensor_baro_healthy();

void hal_external_sensor_read(imu_data_t *data);
void hal_external_sensor_read_mag(float *mag);
void hal_external_sensor_read_baro(float *baro);

void hal_external_sensor_set_bypass(bool enable);
StreamBufferHandle_t hal_external_sensor_get_streambuffer();

void hal_external_sensor_task(void *pvParameters);
