#ifndef HAL_MAG_H
#define HAL_MAG_H

#include "hal_type_define.h"

void hal_mag_init();
bool hal_mag_healthy();
bool hal_mag_read(mag_data_t *data);

#endif // HAL_MAG_H
