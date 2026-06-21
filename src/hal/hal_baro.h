#ifndef HAL_BARO_H
#define HAL_BARO_H

#include "hal_type_define.h"

void hal_baro_init();
bool hal_baro_healthy();
bool hal_baro_read(baro_data_t *data);

#endif // HAL_BARO_H
