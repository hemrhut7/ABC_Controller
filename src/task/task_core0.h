#pragma once

#include <Arduino.h>
#include "hal/hal_message.h"

void task_read_AHRS(void * pvParameters);
void task_get_AHRS(AHRS_Data *data);