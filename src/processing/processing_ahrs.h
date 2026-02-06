#ifndef PROCESSING_AHRS_H
#define PROCESSING_AHRS_H

#include "hal/hal_type_define.h"

void processing_ahrs_init();
void processing_ahrs_update();
void processing_ahrs_get_euler(float euler[3]);
void processing_ahrs_get_imu(imu_data_t *data);

#endif // PROCESSING_AHRS_H
