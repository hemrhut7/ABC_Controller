#include "hal_mag.h"
#include "hal_imu.h"

void hal_mag_init() {
    hal_imu_mag_init();
}

bool hal_mag_healthy() {
    return hal_imu_mag_healthy();
}

bool hal_mag_read(mag_data_t *data) {
    return hal_imu_mag_read(data);
}
