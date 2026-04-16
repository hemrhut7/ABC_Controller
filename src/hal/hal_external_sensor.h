#pragma once

#include "hal_type_define.h"
#include <Arduino.h>

void hal_external_sensor_init();
bool hal_external_sensor_healthy();
void hal_external_sensor_read(imu_data_t *data);
void hal_external_sensor_task(void *pvParameters);
